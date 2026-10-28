"""Passing needs relative progress and clearance, beyond collision-free following."""
import unittest
from server.navigation.dynamic_avoidance import Action,AvoidanceConfig,Target,Vessel
from server.navigation.dynamic_avoidance.overtaking import OvertakingProgress
from server.experiments.kev_dynamic_avoidance import run_scene
from tools.waypoint_geometry import KNOT_MPS


class OvertakingTests(unittest.TestCase):
    def test_safe_port_progress_when_starboard_has_no_safe_candidates(self):
        progress=OvertakingProgress(AvoidanceConfig())
        own=Vessel((0.,0.),20*KNOT_MPS,90.)
        target=Target(4001,(250.,0.),9.6*KNOT_MPS,90.)
        self.assertEqual(progress.update(own,(target,),Action(50*KNOT_MPS,90.),(4001,)),target)
        options={'left':{'speed_knots':30.}}
        self.assertEqual(progress.select(own,target,options,{'left':Action(30*KNOT_MPS,60.)}),options)
        self.assertEqual(progress.details['side'],'port')

    def test_no_safe_relative_progress_retains_yielding_choices(self):
        progress=OvertakingProgress(AvoidanceConfig())
        own=Vessel((0.,0.),9.6*KNOT_MPS,90.)
        target=Target(4001,(250.,0.),9.6*KNOT_MPS,90.)
        progress.update(own,(target,),Action(50*KNOT_MPS,90.),(4001,))
        safe={'match':{'speed_knots':9.6}}
        self.assertEqual(progress.select(own,target,safe,{'match':Action(9.6*KNOT_MPS,90.)}),safe)
        self.assertEqual(progress.details['phase'],'waiting_safe_progress')

    def test_near_equal_speed_or_opposing_target_does_not_start_pass(self):
        for speed,course in ((50.,90.),(9.6,270.)):
            progress=OvertakingProgress(AvoidanceConfig())
            self.assertIsNone(progress.update(Vessel((0.,0.),20*KNOT_MPS,90.),
                (Target(4001,(300.,0.),speed*KNOT_MPS,course),),Action(50*KNOT_MPS,90.),(4001,)))

    def test_completed_pass_releases_and_missing_target_cancels(self):
        progress=OvertakingProgress(AvoidanceConfig());target=Target(4001,(200.,0.),5.,90.)
        progress.update(Vessel((0.,0.),10.,90.),(target,),Action(20.,90.),(4001,))
        self.assertTrue(progress.active)
        self.assertIsNone(progress.update(Vessel((400.,0.),20.,90.),(target,),Action(20.,90.),()))
        self.assertFalse(progress.active)
        progress.update(Vessel((0.,0.),10.,90.),(target,),Action(20.,90.),(4001,))
        progress.update(Vessel((0.,0.),10.,90.),(),Action(20.,90.),())
        self.assertFalse(progress.active)

    def test_two_slow_targets_are_both_passed(self):
        result=run_scene('passing_two',
            (Target(4001,(300.,0.),9.6*KNOT_MPS,90.),
             Target(4003,(700.,-30.),9.6*KNOT_MPS,98.)),
            steps=120,mock=True,response='physical',initial_speed_knots=50.)
        self.assertGreater(result['min_clearance_m']['dynamic'],0.)
        for target_id in (4001,4003):
            self.assertTrue(any(row['own']['position_m'][0]-next(t for t in row['targets']
                if t['target_id']==target_id)['position_m'][0]>150. for row in result['samples']))

    def test_closed_loop_passes_slow_target_then_rejoins(self):
        for initial in (20.,50.):
            with self.subTest(initial=initial):
                result=run_scene('passing_progress',
                    (Target(4001,(300.,0.),9.6*KNOT_MPS,90.),),steps=130,mock=True,
                    response='physical',initial_speed_knots=initial)
                self.assertGreater(result['min_clearance_m']['dynamic'],0.)
                samples=result['samples']
                passing=[r for r in samples if r['own']['position_m'][0]-r['targets'][0]['position_m'][0]>120.]
                self.assertTrue(passing,'Safe speed matching is not completed overtaking')
                self.assertLess(abs(samples[-1]['own']['position_m'][1]),10.)
                phases={r['decision'].get('dynamic_avoidance',{}).get('overtaking',{}).get('phase') for r in samples}
                self.assertIn('move_out',phases);self.assertIn('pass',phases)


if __name__=='__main__':unittest.main()
