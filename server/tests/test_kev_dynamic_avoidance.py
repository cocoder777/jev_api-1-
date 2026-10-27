"""Regression contract: dynamic avoidance is deliberately disconnected from Kev."""
import copy
import unittest
from unittest.mock import Mock

from server.policies.kev_policy import KevPolicy
from server.tests.test_static_obstacles import choose_heading, packet, record


class KevDynamicAvoidanceDisconnectedTests(unittest.TestCase):
    def test_moving_and_stale_targets_do_not_change_model_candidates(self):
        base = packet()
        base['situation'] = base['situation'][:1]
        baseline = KevPolicy(client=choose_heading)
        expected = baseline(base)
        for age in (0., 30.):
            request = copy.deepcopy(base)
            request['situation'].append(record(2001, 300, 0, speed=20, course=270,
                                              _received_age_s=age))
            client = Mock(side_effect=choose_heading)
            policy = KevPolicy(client=client)
            self.assertEqual(policy(request), expected)
            self.assertEqual(policy.local.last_decision['options'], baseline.local.last_decision['options'])
            self.assertNotIn('dynamic_avoidance', client.call_args.args[0]['state'])
            self.assertFalse(hasattr(policy.local, 'dynamic_guard'))

    def test_send_guard_retains_static_only_behavior(self):
        request = packet()
        own = request['situation'][0]
        request['situation'] = [own, record(2001, 200, 0, speed=20, course=270)]
        policy = KevPolicy()
        self.assertTrue(policy.validate_control(request, own, request['situation'], 50, 90)['safe'])
        request['situation'].append(record(4001, 20, 0, speed=0))
        self.assertFalse(policy.validate_control(request, own, request['situation'], 50, 90)['safe'])

    def test_static_navigation_still_works_with_moving_targets_present(self):
        request = packet()
        baseline = KevPolicy(client=choose_heading)
        expected = baseline(request)
        request['situation'].append(record(2001, 300, 0, speed=20, course=270))
        policy = KevPolicy(client=choose_heading)
        self.assertEqual(policy(request), expected)
        self.assertEqual(policy.local.last_decision['options'], baseline.local.last_decision['options'])
        self.assertIn('static_avoidance', policy.local.last_decision['state'])


if __name__ == '__main__':
    unittest.main()
