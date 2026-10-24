"""Task lifecycle regressions: mock all sockets and models, run real worker scheduling."""

from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import MagicMock, Mock, patch

from server import decision_service as ds
from server.control_runtime import error_details
from server.policies.kev_policy import KevPolicy


def task(count=2, completed=0):
    points = [dict(routeLon=.01 * (i + 1), routeLat=0., routeSpeed=60., routeRadius=20.)
              for i in range(count)]
    return dict(simId='offline', id='test-task', code='trackingMode', params=dict(
        platCode=1010, currentRoute=completed, planWaypointNum=count, planWaypointList=points))


def state():
    return dict(platformId=1010, longitude=0., latitude=0., targetSpeed=10., targetCourse=90.)


class WaypointContractTests(unittest.TestCase):
    def test_zero_radius_uses_the_same_arrival_tolerance_in_session_and_all_backends(self):
        from server.jev_policy import JevPolicy
        from server.policies.julia_policy import JuliaPolicy

        packet = task()
        for point in packet['params']['planWaypointList']:
            point['routeRadius'] = 0.
        session = ds.ControlSession(packet, ('127.0.0.1', 7612))
        policies = [KevPolicy(client=Mock()), JevPolicy(), JuliaPolicy('.', engine=Mock())]
        # About 5.6 metres from each point: no exact coordinate match required.
        for index in range(2):
            own = dict(state(), longitude=.01 * (index + 1) - .00005)
            session.update_situation(own)
            self.assertEqual(session.current_route, index + 1)
            for policy in policies:
                observation = policy.observation(dict(packet, situation=(own,)))
                self.assertEqual(policy.current_route(), index + 1)
                self.assertEqual(observation['task']['arrival_radius_m'], 20.)
        self.assertTrue(session.route_complete)
        self.assertEqual(session.status_fields()[1]['completionRate'], 100.)
        self.assertEqual([p['routeRadius'] for p in packet['params']['planWaypointList']], [0., 0.])

    def test_invalid_radii_are_rejected_by_session_and_policy(self):
        for radius in (-1, float('nan'), float('inf'), True, None, '0'):
            with self.subTest(radius=radius):
                packet = task()
                packet['params']['planWaypointList'][1]['routeRadius'] = radius
                with self.assertRaisesRegex(ValueError, 'routeRadius'):
                    ds.ControlSession(packet, ('127.0.0.1', 7612))
                with self.assertRaisesRegex(ValueError, 'routeRadius'):
                    KevPolicy(client=Mock()).observation(dict(packet, situation=(state(),)))

    def test_first_point_and_completed_count_are_not_confused_with_origin(self):
        for completed in (0, 1):
            with self.subTest(completed=completed):
                packet = dict(task(completed=completed), situation=(state(),))
                policy = KevPolicy(client=Mock())
                observation = policy.observation(packet)
                self.assertEqual(observation['task']['next_waypoint_index'], completed)
                self.assertEqual(policy.current_route(), completed)

    def test_single_waypoint_stops_without_another_model_call_and_latches_arrival(self):
        client = Mock(return_value={'answers': {'action': {'type': 'choice', 'choice': 'heading_07'}}})
        policy = KevPolicy(client=client)
        packet = dict(task(count=1), situation=(state(),))
        self.assertGreater(policy(packet)[0], 0.)
        packet['situation'][0]['longitude'] = .01
        self.assertEqual(policy(packet)[0], 0.)
        packet['situation'][0]['longitude'] = .02
        self.assertEqual(policy(packet)[0], 0.)
        self.assertEqual(policy.current_route(), 1)
        client.assert_called_once()

    def test_being_at_second_point_does_not_skip_unvisited_first_point(self):
        own = dict(state(), longitude=.02)
        obs = KevPolicy(client=Mock()).observation(dict(task(), situation=(own,)))
        self.assertEqual(obs['task']['next_waypoint_index'], 0)
        self.assertFalse(obs['task']['route_complete'])

    def test_progress_includes_initial_position_to_first_point(self):
        session = ds.ControlSession(task(), ('127.0.0.1', 7612))
        session.update_situation(state())
        _, fields = session.status_fields()
        self.assertEqual(fields['nextWaypointIndex'], 0)
        self.assertEqual(fields['remainingWaypointNum'], 2)
        self.assertEqual(fields['completionRate'], 0.)
        self.assertAlmostEqual(fields['remainingDistanceM'], 2226.39, delta=.1)
        session.update_situation(dict(state(), longitude=.005))
        self.assertAlmostEqual(session.status_fields()[1]['completionRate'], 25., delta=.01)

    def test_invalid_completed_count_is_rejected(self):
        for completed in (-1, 3, True):
            with self.subTest(completed=completed), self.assertRaises(ValueError):
                ds.ControlSession(task(completed=completed), ('127.0.0.1', 7612))

    def test_invalid_platform_and_noninteger_point_count_are_rejected_before_control(self):
        for field, value in (('platCode', True), ('platCode', -1), ('platCode', None),
                             ('planWaypointNum', True), ('planWaypointNum', 1.5)):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                packet = task(count=1)
                packet['params'][field] = value
                ds.ControlSession(packet, ('127.0.0.1', 7612))


class TaskControlTests(unittest.TestCase):
    def run_task(self, policy, *, count=2, completed=0, on_control=None,
                 on_get=None, hold=.5, send_error=False, status_errors=0):
        self.own = state()
        self.session = ds.ControlSession(task(count, completed), ('127.0.0.1', 7612))
        self.registry = ds.TaskRegistry()
        self.registry.add(self.session)
        self.commands = []
        self.error_counts = []
        original_record = self.session.record_error

        def record(stage, error, limit):
            result = original_record(stage, error, limit)
            self.error_counts.append((stage, result))
            return result

        self.session.record_error = record
        get_count = 0

        def get(*_):
            nonlocal get_count
            get_count += 1
            if on_get is not None:
                return on_get(get_count)
            return dict(self.own)

        store = Mock(get=Mock(side_effect=get))

        def send(payload, _):
            if send_error:
                raise OSError('simulated UDP send failure')
            command = ds.decode_speed_course_packet(payload)
            self.commands.append((time.monotonic(), command))
            if command['speed'] == 0:
                self.own['targetSpeed'] = 0.
            if on_control is not None:
                on_control(command)

        control = Mock(sendto=Mock(side_effect=send))
        context = MagicMock()
        context.__enter__.return_value = control
        status = Mock()
        if status_errors:
            status.sendto.side_effect = [OSError('status unavailable')] * status_errors + [None]
        timed_out = threading.Event()

        def abort():
            timed_out.set()
            self.session.request_stop()

        timer = threading.Timer(3., abort)
        timer.daemon = True
        timer.start()
        try:
            with patch.object(ds.socket, 'socket', return_value=context), patch.object(ds, 'LOGGER'):
                ds.run_control_loop(self.session, status, store, ('127.0.0.1', 18123),
                                    policy, .01, 0, 2.5, .1, self.registry, 3, hold)
        finally:
            timer.cancel()
        self.assertFalse(timed_out.is_set(), 'task required watchdog cancellation')
        self.assertIsNone(self.registry.get(self.session.key))
        self.status = ds.analyze_task(status.sendto.call_args.args[0])
        return self.status

    def test_three_model_timeouts_fail_once_with_detailed_reason(self):
        policy = Mock(side_effect=TimeoutError('response timed out'))
        status = self.run_task(policy)
        self.assertEqual(policy.call_count, 3)
        self.assertEqual(status['status'], 3)
        self.assertIn('连续失败3/3', status['description'])
        self.assertIn('平台1010', status['description'])
        self.assertIn('目标航路点1/2', status['description'])
        self.assertIn('尚未确认实际停车', status['description'])
        self.assertEqual(status['params']['errorCode'], 'MODEL_TIMEOUT')
        self.assertTrue(status['params']['stopCommandSent'])
        self.assertEqual([c['speed'] for _, c in self.commands], [0.])

    def test_two_errors_recover_and_reset_before_a_later_retry(self):
        outcomes = [TimeoutError('first'), (60., 90.), TimeoutError('second'),
                    TimeoutError('third'), (60., 135.)]

        def decide(_):
            outcome = outcomes.pop(0) if outcomes else (60., 135.)
            if isinstance(outcome, Exception):
                raise outcome
            return outcome

        def control(command):
            if command['course'] == 135.:
                self.session.request_stop()

        status = self.run_task(decide, on_control=control)
        self.assertEqual(self.error_counts, [('inference', 1), ('inference', 1), ('inference', 2)])
        self.assertEqual(status['status'], 2)
        self.assertEqual(status['params']['consecutiveErrors'], 0)
        self.assertGreaterEqual(sum(c['speed'] > 0 for _, c in self.commands), 3)

    def test_invalid_model_result_is_not_retried(self):
        policy = Mock(side_effect=ValueError('choice outside allowed candidates'))
        status = self.run_task(policy)
        policy.assert_called_once()
        self.assertEqual(status['status'], 3)
        self.assertIn('不可恢复错误', status['description'])

    def test_periodic_commands_continue_while_model_is_blocked(self):
        release = threading.Event()
        calls = 0
        blocked = threading.Event()

        def decide(_):
            nonlocal calls
            calls += 1
            if calls == 1:
                return 60., 90.
            blocked.set()
            release.wait(1.)
            return 60., 135.

        def control(command):
            if command['speed'] > 0 and len(self.commands) >= 4:
                release.set()
            if command['speed'] > 0 and command['course'] == 135.:
                self.session.request_stop()

        try:
            status = self.run_task(decide, on_control=control)
        finally:
            release.set()
        self.assertTrue(blocked.is_set())
        self.assertEqual(status['status'], 2)
        self.assertGreaterEqual(sum(c['speed'] > 0 and c['course'] == 90. for _, c in self.commands), 4)

    def test_hold_expiry_sends_zero_and_does_not_accept_late_result_after_stop(self):
        release = threading.Event()
        calls = 0

        def decide(_):
            nonlocal calls
            calls += 1
            if calls == 1:
                return 60., 90.
            release.wait(1.)
            return 60., 135.

        def control(command):
            if command['speed'] == 0:
                self.session.request_stop()
                release.set()

        try:
            status = self.run_task(decide, on_control=control, hold=.035)
        finally:
            release.set()
        self.assertEqual(status['status'], 2)
        self.assertTrue(status['params']['stopConfirmed'])
        self.assertGreaterEqual(sum(c['speed'] > 0 for _, c in self.commands), 2)
        self.assertFalse(any(c['course'] == 135. and c['speed'] > 0 for _, c in self.commands))

    def test_final_arrival_interrupts_pending_model_and_waits_for_stopping(self):
        release = threading.Event()
        calls = 0

        def decide(_):
            nonlocal calls
            calls += 1
            if calls > 1:
                release.wait(1.)
            return 60., 90.

        def control(command):
            if command['speed'] > 0:
                self.own.update(longitude=.01, targetSpeed=60.)
            else:
                # Drift outside the circle while braking must not resume tracking.
                self.own.update(longitude=.011, targetSpeed=0.)
                release.set()

        try:
            status = self.run_task(decide, count=1, on_control=control)
        finally:
            release.set()
        self.assertEqual(status['status'], 2)
        self.assertIn('全部航路点已到达', status['description'])
        self.assertEqual(status['params']['completionRate'], 100.)
        self.assertEqual(status['params']['currentRoute'], 1)
        self.assertTrue(status['params']['stopConfirmed'])
        first_zero = next(i for i, (_, c) in enumerate(self.commands) if c['speed'] == 0.)
        self.assertTrue(all(c['speed'] == 0. for _, c in self.commands[first_zero:]))

    def test_three_missing_telemetry_cycles_fail_after_short_command_hold(self):
        offline = False

        def control(command):
            nonlocal offline
            if command['speed'] > 0:
                offline = True

        status = self.run_task(lambda _: (60., 90.), on_control=control,
                               on_get=lambda _: None if offline else dict(self.own))
        self.assertEqual(status['status'], 3)
        self.assertEqual(status['params']['errorCode'], 'SITUATION_TIMEOUT')
        self.assertEqual(self.error_counts, [('situation', 1), ('situation', 2), ('situation', 3)])
        self.assertGreaterEqual(sum(c['speed'] > 0 for _, c in self.commands), 3)
        self.assertEqual(self.commands[-1][1]['speed'], 0.)

    def test_telemetry_recovery_resets_consecutive_missing_cycles(self):
        def control(command):
            if command['speed'] > 0 and len(self.commands) >= 4:
                self.session.request_stop()

        def get(tick):
            return None if tick in (2, 4) else dict(self.own)

        status = self.run_task(lambda _: (60., 90.), on_control=control, on_get=get)
        self.assertEqual(status['status'], 2)
        self.assertEqual(self.error_counts, [('situation', 1), ('situation', 1)])

    def test_repeated_control_send_errors_are_not_reset_by_model_success(self):
        status = self.run_task(lambda _: (60., 90.), send_error=True)
        self.assertEqual(status['status'], 3)
        self.assertEqual(status['params']['errorCode'], 'CONTROL_SEND_FAILED')
        self.assertEqual(status['params']['consecutiveErrors'], 3)
        self.assertFalse(status['params']['stopCommandSent'])
        self.assertIn('停车指令发送失败', status['description'])

    def test_result_for_previous_waypoint_is_discarded_after_progress_advances(self):
        release = threading.Event()
        requests = []

        def decide(packet):
            requests.append(packet['params']['next_waypoint_index'])
            if len(requests) == 1:
                return 60., 90.
            if len(requests) == 2:
                release.wait(1.)
                return 60., 135.
            return 60., 180.

        def get(_):
            if self.session.current_route == 1:
                release.set()
            return dict(self.own)

        def control(command):
            if command['speed'] > 0:
                self.own['longitude'] = .01
                if command['course'] == 180.:
                    self.session.request_stop()

        try:
            status = self.run_task(decide, on_control=control, on_get=get)
        finally:
            release.set()
        self.assertEqual(status['status'], 2)
        self.assertEqual(requests[:3], [0, 0, 1])
        self.assertFalse(any(c['speed'] > 0 and c['course'] == 135. for _, c in self.commands))

    def test_failure_status_send_is_retried_without_another_model_request(self):
        policy = Mock(side_effect=TimeoutError('read timed out'))
        status = self.run_task(policy, status_errors=2)
        self.assertEqual(status['status'], 3)
        self.assertEqual(policy.call_count, 3)

    def test_already_completed_task_only_sends_stop(self):
        policy = Mock()
        status = self.run_task(policy, count=1, completed=1)
        policy.assert_not_called()
        self.assertEqual(status['status'], 2)
        self.assertEqual(status['params']['completionRate'], 100.)
        self.assertTrue(all(c['speed'] == 0. for _, c in self.commands))

    def test_error_metadata_does_not_expose_credentials(self):
        with patch.dict('os.environ', {'JEV_API_KEY': 'secret-for-test'}):
            details = error_details('inference', RuntimeError('secret-for-test Bearer hidden apikey_hidden'))
        self.assertNotIn('secret-for-test', details['lastError'])
        self.assertNotIn('hidden', details['lastError'])


class LocalUdpTaskTests(unittest.TestCase):
    def test_single_waypoint_task_completes_only_after_zero_speed_feedback(self):
        self.run_single_waypoint(20.)

    def test_zero_radius_task_is_accepted_and_completes_without_exact_position_match(self):
        self.run_single_waypoint(0.)

    def run_single_waypoint(self, radius):
        # Every socket uses loopback and an ephemeral port; never touch field endpoints.
        script = '''
import sys
from server.decision_service import start_server
def policy(packet):
    assert packet['params']['next_waypoint_index'] == 0
    return 60., 90.
start_server(host='127.0.0.1', port=int(sys.argv[1]), policy=policy,
             situation_port=0, multicast_interface='127.0.0.1',
             action_host='127.0.0.1', action_port=int(sys.argv[2]),
             control_interval=.02, status_interval=.02, platform_id=1010)
'''
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
            peer.bind(('127.0.0.1', 0))
            control.bind(('127.0.0.1', 0))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(('127.0.0.1', 0))
                task_port = reserve.getsockname()[1]
            target = ('127.0.0.1', task_port)
            process = subprocess.Popen(
                [sys.executable, '-c', script, str(task_port), str(control.getsockname()[1])],
                cwd=Path(__file__).resolve().parents[2], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
            own = dict(state(), direction=0, sign=0, height=0, type=0, batch=0)
            packet = task(count=1)
            packet['params']['planWaypointList'][0]['routeRadius'] = radius
            accepted = motion_seen = braking_seen = False
            terminal = None
            deadline = time.monotonic() + 4.
            try:
                while terminal is None and time.monotonic() < deadline:
                    peer.sendto(ds.build_situation_packet(own), target)
                    if not accepted:
                        peer.sendto(ds.build_json_packet(packet, ds.ALGORITHM_TASK_ID), target)
                    ready, _, _ = select.select([peer, control], [], [], .02)
                    for endpoint in ready:
                        payload, _ = endpoint.recvfrom(65507)
                        if endpoint is control:
                            command = ds.decode_speed_course_packet(payload)
                            if command['speed'] > 0:
                                self.assertFalse(braking_seen, 'positive command after terminal braking')
                                motion_seen = True
                                own.update(longitude=.00995, targetSpeed=60.)
                            else:
                                braking_seen = True
                                own.update(longitude=.011, targetSpeed=0.)
                        else:
                            status = ds.analyze_task(payload)
                            accepted = True
                            if status['status'] in (2, 3):
                                terminal = status
                self.assertTrue(motion_seen)
                self.assertTrue(braking_seen)
                self.assertIsNotNone(terminal)
                self.assertEqual(terminal['status'], 2)
                self.assertEqual(terminal['params']['currentRoute'], 1)
                self.assertEqual(terminal['params']['completionRate'], 100.)
                self.assertTrue(terminal['params']['stopConfirmed'])
            finally:
                process.terminate()
                process.communicate(timeout=3.)


if __name__ == '__main__':
    unittest.main()
