"""Real-model diagnostics; production no-progress defaults are not modified."""
import copy
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from server.experiments import field_tasks as ft
from server.experiments.simulate import scenario
from server.policies.progress import ProgressConfig, NavigationStalled
from tools.waypoint_geometry import LocalFrame

OUTPUT = Path(__file__).resolve().parent


def segment_distance(position, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx*dx + dy*dy
    t = 0. if length2 == 0 else max(0., min(1., ((position[0]-start[0])*dx + (position[1]-start[1])*dy)/length2))
    return math.dist(position, (start[0] + t*dx, start[1] + t*dy))


class DiagnosticPolicy(ft.KevPolicy):
    def __init__(self, task, start, case, **kwargs):
        super().__init__(progress_config=ProgressConfig(recovery_limit=10000), **kwargs)
        frame = LocalFrame(task['params']['planWaypointList'])
        self.points = [frame.project(p['routeLon'],p['routeLat']) for p in task['params']['planWaypointList']]
        self.start = start
        self.case = case
        self.goal = None
        self.best_distance = float('inf')
        self.goal_decisions = 0

    def __call__(self, packet):
        action = super().__call__(packet)
        decision = self.local.last_decision
        state = decision['state']
        index = state['current_waypoint_index']
        if index != self.goal:
            self.goal, self.best_distance, self.goal_decisions = index, float('inf'), 0
        self.best_distance = min(self.best_distance, state['distance_to_target_m'])
        self.goal_decisions += 1
        start = self.points[index-1] if index else self.start
        cross_track = segment_distance(state['boat_xy_m'], start, self.points[index])
        growth = state['distance_to_target_m'] - self.best_distance
        decision['diagnostic'] = dict(cross_track_m=cross_track, distance_growth_m=growth)
        if self.goal_decisions >= 30 and (cross_track >= 800. or growth >= 1000.):
            note = dict(case=self.case, waypoint_index=index, cross_track_m=cross_track,
                        distance_growth_m=growth, goal_decisions=self.goal_decisions,
                        actual_course_deg=state['actual_course_deg'],
                        heading_error_deg=state['target_relative_bearing_deg'])
            (OUTPUT / f'{self.case}-deviation-stop.json').write_text(json.dumps(note, indent=2)+'\n')
            raise NavigationStalled(f'Diagnostic-only deviation limit: cross-track {cross_track:.1f}m, distance growth {growth:.1f}m')
        return action


def main():
    template = scenario('crossing_km')
    frame = LocalFrame(template['params']['planWaypointList'])
    points = [frame.project(p['routeLon'],p['routeLat']) for p in template['params']['planWaypointList']]
    dx, dy = points[1][0]-points[0][0], points[1][1]-points[0][1]
    length = math.hypot(dx, dy)
    probe_start = (points[1][0]-150*dx/length, points[1][1]-150*dy/length)
    probe_heading = math.degrees(math.atan2(dx,dy)) % 360
    results = []
    for name, start, heading, completed in (
        ('full_route', (-640.,0.), 90., 0),
    ):
        task = copy.deepcopy(template)
        task['id'] = f'crossing-km-{name}'
        task['params']['currentRoute'] = completed
        config = ft.TestConfig(steps=2000,dt=1.,initial_position_xy_m=start,
                               initial_course_deg=heading,max_turn_rate_deg_s=45.)
        policy = DiagnosticPolicy(task,start,name,duration=1.,initial_course_deg=heading)
        print('START',name,'initial_completed=',completed,'initial_position=',start,'initial_heading=',heading,flush=True)
        with patch.object(ft,'load_task',return_value=task):
            result = ft.run_task(name,OUTPUT,config,policy)
        result['initial_completed_waypoints'] = completed
        result['diagnostic_recovery_limit'] = 10000
        result['diagnostic_stop_cross_track_m'] = 800.
        result['diagnostic_stop_distance_growth_m'] = 1000.
        (OUTPUT/name/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        results.append(result)
        print('END',name,json.dumps({k:result[k] for k in ('reason','steps','reached_waypoints','total_waypoints','recovery_decisions','stop_confirmed')},ensure_ascii=False),flush=True)
    (OUTPUT/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')


if __name__ == '__main__':
    main()
