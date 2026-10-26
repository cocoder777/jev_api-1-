"""Backend contract tests without downloading weights or making network calls."""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from server.decision_service import build_situation_packet, decode_situation_packet
from server.policies.factory import create_policy
from server.policies.julia_policy import JuliaPolicy
from server.experiments.simulate import scenario, telemetry
from tools.waypoint_geometry import KNOT_MPS


class LocalPolicyTests(unittest.TestCase):
    def packet(self):
        packet = scenario('crossing_km')
        packet['situation'] = [decode_situation_packet(build_situation_packet(telemetry(0, 0, 0, None)))]
        return packet

    def engine(self):
        def predict(*, state, questions):
            return {'answers': {key: dict(type='choice', choice=list(q['criteria'])[-1])
                                for key, q in questions.items()}}
        return Mock(predict=Mock(side_effect=predict))

    def test_sixteen_candidates_are_selected_in_one_call_without_changing_jev(self):
        engine = self.engine(); policy = JuliaPolicy('.', engine=engine, max_speed_knots=3.)
        speed, course = policy(self.packet())
        engine.predict.assert_called_once()
        request = engine.predict.call_args.kwargs
        self.assertEqual(set(request['questions']), {'action'})
        criteria = request['questions']['action']['criteria']
        self.assertEqual(len(criteria), 16)
        self.assertEqual(policy.observation(self.packet())['action_space']['speeds_knots'], [10., 20.])
        self.assertEqual((speed, course), (20., 315))
        self.assertNotIn('threatLevel', request['state'])
        self.assertNotIn('sign', request['state'])
        from server.jev_policy import JevPolicy
        from choice_control import ACTION_CHOICES
        self.assertEqual(len(ACTION_CHOICES), 64)
        self.assertEqual(len(JevPolicy().observation(self.packet())['action_space']['speeds_knots']), 8)

    def test_speed_unit_conversion_and_progress_are_shared(self):
        packet = self.packet()
        packet['params']['planWaypointList'][0]['routeSpeed'] = 4.
        for p in packet['situation']:
            p['longitude'] = packet['params']['planWaypointList'][0]['routeLon']
            p['latitude'] = packet['params']['planWaypointList'][0]['routeLat']
        policy = JuliaPolicy('.', engine=self.engine(), speed_unit='mps')
        speed, _ = policy(packet)
        self.assertAlmostEqual(speed, 20 * KNOT_MPS)
        self.assertEqual(policy.current_route(), 1)

    def test_invalid_model_choice_fails_instead_of_substituting_action(self):
        engine = Mock()
        engine.predict.return_value = {'answers': {'action': dict(type='choice', choice='invented')}}
        with self.assertRaisesRegex(ValueError, 'outside the 16 candidates'):
            JuliaPolicy('.', engine=engine)(self.packet())

    def test_lower_speed_decodes_to_ten_knots_and_malformed_response_fails(self):
        engine = Mock()
        engine.predict.return_value = {'answers': {'action': dict(type='choice', choice='speed_10knots__east')}}
        self.assertEqual(JuliaPolicy('.', engine=engine)(self.packet()), (10., 90))
        engine.predict.return_value = {'answers': {'action': dict(type='score', score=1)}}
        with self.assertRaisesRegex(ValueError, 'Expected Julia Choice'):
            JuliaPolicy('.', engine=engine)(self.packet())

    def test_missing_weights_and_unknown_backend_are_explicit_errors(self):
        with self.assertRaisesRegex(RuntimeError, 'weights missing'):
            JuliaPolicy('/tmp/jev-nonexistent-model-directory')
        with self.assertRaisesRegex(ValueError, 'Unknown policy'):
            create_policy('unknown')

    def test_binary_telemetry_roundtrip_coordinates_and_units(self):
        state = telemetry(35, 20, 3., 90.)
        decoded = decode_situation_packet(build_situation_packet(state))
        self.assertAlmostEqual(decoded['longitude'], state['longitude'], places=6)
        self.assertAlmostEqual(decoded['latitude'], state['latitude'], places=6)
        self.assertLessEqual(abs(decoded['targetSpeed'] - state['targetSpeed']), .05)
        self.assertEqual(decoded['targetCourse'], 90.)


if __name__ == '__main__':
    unittest.main()
