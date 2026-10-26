"""Speed stages distinguish long-leg cruise from waypoint turns and limits."""
import math
from types import SimpleNamespace
import unittest

from server.policies.speed_profile import plan_speed
from tools.waypoint_geometry import KNOT_MPS


def profile(turn, distance, limit=50., final=False, heading=90., speeds=(10.,20.,30.,40.,50.)):
    outgoing = (1000 * math.cos(math.radians(turn)), -1000 * math.sin(math.radians(turn)))
    points = [(0., 0.)] if final else [(0., 0.), outgoing]
    geometry = SimpleNamespace(target=(distance, 0.), waypoint_positions=points,
                               waypoints=[(x + distance, y) for x, y in points])
    obs = dict(task=dict(next_waypoint_index=0, arrival_radius_m=100.,
                         planWaypointList=[dict(routeSpeed=limit*KNOT_MPS) for _ in points]),
               boat=dict(targetSpeed=50., targetCourse=heading),
               execution=dict(speed_transition_time_s=.5), planning=dict(action_duration_s=1.))
    return plan_speed(obs, geometry, speeds, 45.)


class SpeedProfileTests(unittest.TestCase):
    def test_far_aligned_legs_cruise_at_full_speed_even_before_a_hairpin(self):
        for turn in (0., 30., 90., 170.):
            with self.subTest(turn=turn):
                result = profile(turn, 1000.)
                self.assertEqual(result['phase'], 'cruise')
                self.assertEqual(result['speeds_knots'], [50.])
                self.assertAlmostEqual(result['next_turn_deg'], turn)
                self.assertEqual(result['next_waypoint_index'], 1)

    def test_turn_magnitude_sets_near_waypoint_speed_cap(self):
        for turn, cap in ((10.,50.), (30.,40.), (60.,30.), (120.,20.), (170.,10.)):
            with self.subTest(turn=turn):
                result = profile(turn, 140.)
                self.assertEqual(result['speed_cap_knots'], cap)
                self.assertEqual(result['phase'], 'cruise' if cap == 50. else 'approach')

    def test_waypoint_ceiling_reduces_speed_only_when_approaching(self):
        self.assertEqual(profile(0.,1000.,limit=20.)['speeds_knots'], [50.])
        near = profile(0.,140.,limit=20.)
        self.assertEqual(near['entry_speed_knots'], 20.)
        self.assertEqual(near['speeds_knots'], [10.,20.])

    def test_misaligned_boat_slows_before_reentering_cruise(self):
        self.assertEqual(profile(0.,1000.,heading=270.)['speeds_knots'], [10.])
        self.assertEqual(profile(0.,1000.,heading=90.)['speeds_knots'], [50.])

    def test_planned_turn_is_stable_when_observed_heading_changes(self):
        result = profile(60.,140.,heading=120.)
        self.assertAlmostEqual(result['next_leg_bearing_deg'], 150.)
        self.assertAlmostEqual(result['next_turn_deg'], 60.)
        self.assertAlmostEqual(result['observed_next_turn_deg'], 30.)
        self.assertEqual(result['entry_speed_knots'], 30.)

    def test_actual_course_angle_wraps_at_north(self):
        result = profile(-45.,1000.,heading=315.)
        self.assertAlmostEqual(result['next_leg_bearing_deg'], 45.)
        self.assertAlmostEqual(result['next_turn_deg'], 45.)
        self.assertAlmostEqual(result['observed_next_turn_deg'], 90.)

    def test_final_waypoint_has_no_preview_and_requires_slow_approach(self):
        far = profile(0.,1000.,final=True)
        near = profile(0.,140.,final=True)
        self.assertIsNone(near['next_waypoint_index'])
        self.assertIsNone(near['next_waypoint_xy_m'])
        self.assertEqual(far['speeds_knots'], [50.])
        self.assertEqual(near['speeds_knots'], [10.])

    def test_cruise_respects_configured_maximum_and_impossible_limits_are_visible(self):
        self.assertEqual(profile(0.,1000.,speeds=(10.,20.,30.))['speeds_knots'], [30.])
        result = profile(0.,140.,limit=5.)
        self.assertTrue(result['below_minimum_speed_limit'])
        self.assertEqual(result['speeds_knots'], [10.])


if __name__ == '__main__':
    unittest.main()
