"""Task ownership and stale output regressions without field traffic or model loading."""

import threading
import time
import select
import socket
import subprocess
import sys
from pathlib import Path
import unittest
from unittest.mock import MagicMock, Mock, patch

from server import decision_service as ds
from server.policies.kev_policy import KevPolicy
from server.tests.test_task_control import task, state


def session(task_id='same', boat=1010):
    packet = task()
    packet['id'] = task_id
    packet['params']['platCode'] = boat
    return ds.ControlSession(packet, ('127.0.0.1', 12345))


class TaskTakeoverTests(unittest.TestCase):
    def test_identical_task_replaces_and_resets_progress(self):
        registry = ds.TaskRegistry()
        old, new = session(), session()
        registry.add(old)
        old.update_route(1)
        self.assertIs(registry.replace(new), old)
        self.assertTrue(old.superseded.is_set())
        self.assertEqual(new.current_route, 0)
        registry.remove(old)
        self.assertIs(registry.get(new.key), new)

    def test_different_id_replaces_only_its_own_boat(self):
        registry = ds.TaskRegistry()
        old, other, new = session('old'), session('other', 1001), session('new')
        registry.add(old)
        registry.add(other)
        self.assertIs(registry.replace(new), old)
        self.assertIsNone(registry.get(old.key))
        self.assertTrue(old.finished.is_set())
        self.assertFalse(other.finished.is_set())
        self.assertIs(registry.get(other.key), other)
        registry.remove(old)
        self.assertIs(registry.get(new.key), new)

    def test_takeover_waits_for_in_progress_send_before_revoking_owner(self):
        registry = ds.TaskRegistry()
        old, new = session('old'), session('new')
        registry.add(old)
        attempted, done = threading.Event(), threading.Event()

        def replace():
            attempted.set()
            registry.replace(new)
            done.set()

        thread = threading.Thread(target=replace, daemon=True)
        with old.output_lock:
            thread.start()
            self.assertTrue(attempted.wait(1.))
            self.assertFalse(done.wait(.02))
            self.assertFalse(old.superseded.is_set())
        thread.join(1.)
        self.assertFalse(thread.is_alive())
        self.assertTrue(old.superseded.is_set())
        self.assertIs(old.output_lock, new.output_lock)

    def test_late_result_or_error_cannot_send_old_control_stop_or_status(self):
        for fail in (False, True):
            with self.subTest(late_failure=fail):
                registry = ds.TaskRegistry()
                old, new = session('old'), session('new')
                registry.add(old)
                blocked, release, returned = threading.Event(), threading.Event(), threading.Event()
                commands = []
                calls = 0

                def policy(_):
                    nonlocal calls
                    calls += 1
                    if calls > 1:
                        blocked.set()
                        release.wait(2.)
                        returned.set()
                        if fail:
                            raise TimeoutError('old request failed after takeover')
                    return 15., 90.

                control = Mock(sendto=Mock(side_effect=lambda packet, _: commands.append(packet)))
                context = MagicMock()
                context.__enter__.return_value = control
                status = Mock()
                thread = threading.Thread(target=ds.run_control_loop, args=(
                    old, status, Mock(get=Mock(return_value=state())), ('127.0.0.1', 18123),
                    policy, .005, 0, 2.5, .1, registry), daemon=True)
                with patch.object(ds.socket, 'socket', return_value=context), patch.object(ds, 'LOGGER'):
                    try:
                        thread.start()
                        self.assertTrue(blocked.wait(1.))
                        self.assertTrue(commands)
                        registry.replace(new)
                        count = len(commands)
                        thread.join(1.)
                        self.assertFalse(thread.is_alive(), 'old control waited on its pending model')
                        release.set()
                        self.assertTrue(returned.wait(1.))
                        self.assertEqual(len(commands), count)
                        self.assertTrue(all(ds.decode_speed_course_packet(p)['speed'] > 0 for p in commands))
                        status.sendto.assert_not_called()
                        self.assertIs(registry.get(new.key), new)
                    finally:
                        release.set()
                        old.finished.set()
                        old.stop_requested.set()
                        thread.join(1.)

    def test_takeover_during_telemetry_error_or_arrival_does_not_emit_emergency_stop(self):
        for fail in (False, True):
            with self.subTest(telemetry_error=fail):
                registry = ds.TaskRegistry()
                old, new = session('old'), session('new')
                registry.add(old)

                def get(*_):
                    registry.replace(new)
                    if fail:
                        raise ValueError('late old-loop telemetry error')
                    return dict(state(), longitude=.01, targetSpeed=0.)

                control, status = Mock(), Mock()
                context = MagicMock()
                context.__enter__.return_value = control
                with patch.object(ds.socket, 'socket', return_value=context), patch.object(ds, 'LOGGER'):
                    ds.run_control_loop(old, status, Mock(get=Mock(side_effect=get)),
                                        ('127.0.0.1', 18123), Mock(), .005, 0, 2.5, .1, registry)
                control.sendto.assert_not_called()
                status.sendto.assert_not_called()
                self.assertIs(registry.get(new.key), new)

    def test_already_superseded_sessions_cannot_start_after_multiple_replacements(self):
        registry = ds.TaskRegistry()
        old, middle, new = session('old'), session('middle'), session('new')
        for item in (old, middle, new):
            registry.replace(item)
        control, status, policy = Mock(), Mock(), Mock()
        context = MagicMock()
        context.__enter__.return_value = control
        with patch.object(ds.socket, 'socket', return_value=context), patch.object(ds, 'LOGGER'):
            for item in (old, middle):
                ds.run_control_loop(item, status, Mock(), ('127.0.0.1', 18123),
                                    policy, .005, 0, 2.5, .1, registry)
        policy.assert_not_called()
        control.sendto.assert_not_called()
        status.sendto.assert_not_called()
        self.assertIs(registry.get(new.key), new)

    def test_slow_old_kev_request_does_not_lock_out_new_task_request(self):
        entered, release, second = threading.Event(), threading.Event(), threading.Event()
        policy = KevPolicy(client=Mock(), initial_course_deg=90.)

        def call(payload):
            if payload['state']['actual_speed_knots'] == 10.:
                entered.set()
                release.wait(2.)
            else:
                second.set()
            return {'answers': {'action': {'type': 'choice', 'choice': 'speed_50__heading_07'}}}

        policy.client = call
        first_packet = dict(task(), situation=(state(),))
        next_packet = dict(task(), id='new', situation=(dict(state(), targetSpeed=11.),))
        old = threading.Thread(target=policy, args=(first_packet,), daemon=True)
        new = threading.Thread(target=policy, args=(next_packet,), daemon=True)
        try:
            old.start()
            self.assertTrue(entered.wait(1.))
            new.start()
            self.assertTrue(second.wait(1.), 'new request blocked by superseded HTTP call')
        finally:
            release.set()
            old.join(1.)
            if new.ident is not None:
                new.join(1.)


class LocalUdpTakeoverTests(unittest.TestCase):
    def test_same_id_and_new_id_takeover_without_old_zero_commands(self):
        script = '''
import logging
import sys
from server.decision_service import start_server
logging.disable(logging.CRITICAL)
def policy(packet):
    return packet['params']['testSpeed'], 90.
start_server(host='127.0.0.1', port=int(sys.argv[1]), policy=policy,
             situation_port=0, multicast_interface='127.0.0.1',
             action_host='127.0.0.1', action_port=int(sys.argv[2]),
             control_interval=.01, status_interval=.02, platform_id=1010)
'''
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control:
            peer.bind(('127.0.0.1', 0))
            control.bind(('127.0.0.1', 0))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(('127.0.0.1', 0))
                port = reserve.getsockname()[1]
            target = ('127.0.0.1', port)
            process = subprocess.Popen(
                [sys.executable, '-c', script, str(port), str(control.getsockname()[1])],
                cwd=Path(__file__).resolve().parents[2],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
            own = dict(state(), direction=0, sign=0, height=0, type=0, batch=0)
            statuses = []

            def receive():
                peer.sendto(ds.build_situation_packet(own), target)
                ready, _, _ = select.select([peer, control], [], [], .01)
                commands = []
                for endpoint in ready:
                    payload, _ = endpoint.recvfrom(65507)
                    if endpoint is peer:
                        statuses.append(ds.analyze_task(payload))
                    else:
                        commands.append(ds.decode_speed_course_packet(payload)['speed'])
                return commands

            packet = task()
            packet['id'] = 'shared'
            try:
                for index, speed in enumerate((11., 22., 33.)):
                    if index == 2:
                        packet['id'] = 'new-id'
                    packet['params']['testSpeed'] = speed
                    encoded = ds.build_json_packet(packet, ds.ALGORITHM_TASK_ID)
                    # Repeated sends only bootstrap the subprocess before its first bind.
                    peer.sendto(encoded, target)
                    deadline = time.monotonic() + 4.
                    next_retry = time.monotonic() + .1
                    seen = False
                    while not seen and time.monotonic() < deadline:
                        if index == 0 and not statuses and time.monotonic() >= next_retry:
                            peer.sendto(encoded, target)
                            next_retry += .1
                        for value in receive():
                            self.assertGreater(value, 0., 'old task emitted a stop during takeover')
                            if value == speed:
                                seen = True
                    self.assertTrue(seen, 'new task failed to gain control')
                    deadline = time.monotonic() + .08
                    while time.monotonic() < deadline:
                        for value in receive():
                            self.assertEqual(value, speed, 'old command after new task gained control')
                    if index == 1:
                        self.assertFalse(any(s['status'] in (2, 3) for s in statuses),
                                         'same-ID old terminal state overwrote new task')
                superseded = [s for s in statuses if s['status'] == 3]
                self.assertEqual(len(superseded), 1)
                self.assertEqual(superseded[0]['id'], 'shared')
                self.assertEqual(superseded[0]['params']['errorCode'], 'TASK_SUPERSEDED')
                self.assertEqual(superseded[0]['params']['replacedByTaskId'], 'new-id')
                peer.sendto(ds.build_json_packet(
                    {key: packet[key] for key in ('simId', 'id', 'code')}, ds.ALGORITHM_STOP_ID), target)
                deadline = time.monotonic() + 2.
                completed = False
                while not completed and time.monotonic() < deadline:
                    for value in receive():
                        if value == 0:
                            own['targetSpeed'] = 0.
                    completed = any(s['id'] == 'new-id' and s['status'] == 2 for s in statuses)
                self.assertTrue(completed, 'latest task failed to handle explicit stop')
            finally:
                process.terminate()
                process.communicate(timeout=3.)


if __name__ == '__main__':
    unittest.main()
