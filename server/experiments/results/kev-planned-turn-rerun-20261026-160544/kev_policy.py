"""Kev: five speed classes, waypoint lookahead and progress supervision."""
import copy
import json
import logging
import math
import time
from collections import deque
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener

from choice_control import _candidate_facts
from tools.waypoint_geometry import KNOT_MPS, WaypointGeometry
from .waypoint import WaypointPolicy
from .turning import DEFAULT_MAX_TURN_RATE_DEG_S, reachable_courses
from .progress import ProgressSupervisor
from .speed_profile import plan_speed

KEV_SPEEDS_KNOTS = (10.0, 20.0, 30.0, 40.0, 50.0)
DEFAULT_KEV_ENDPOINT = 'http://127.0.0.1:18028/v1/systemone'


def round_model_values(value):
    """Limit model input precision, without rounding internal geometry or physics."""
    if isinstance(value, float):
        result = round(value, 2)
        return 0.0 if result == 0 else result
    if isinstance(value, dict):
        return {key: round_model_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [round_model_values(item) for item in value]
    return value


class KevPolicy(WaypointPolicy):
    def __init__(self, endpoint=DEFAULT_KEV_ENDPOINT, timeout=120., client=None, max_turn_rate_deg_s=DEFAULT_MAX_TURN_RATE_DEG_S,
                 initial_course_deg=None, progress_config=None, **kwargs):
        super().__init__(**kwargs)
        self.speeds = [speed for speed in KEV_SPEEDS_KNOTS if speed <= self.max_speed]
        if not self.speeds:
            raise ValueError('Kev max speed must allow at least the 10-knot speed class')
        self.progress_config = progress_config
        if urlsplit(endpoint).scheme not in ('http', 'https'):
            raise ValueError('Kev endpoint must be an HTTP(S) URL')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Kev timeout must be positive and finite')
        if not math.isfinite(max_turn_rate_deg_s) or max_turn_rate_deg_s <= 0:
            raise ValueError('Max turn rate must be positive and finite')
        if initial_course_deg is not None and (not math.isfinite(initial_course_deg) or not 0 <= initial_course_deg < 360):
            raise ValueError('Initial course must be in [0, 360)')
        self.max_turn_rate_deg_s = max_turn_rate_deg_s
        self.initial_course_deg = initial_course_deg
        self.endpoint, self.timeout, self.client = endpoint, timeout, client

    def _observation_course(self, boat):
        """Use the current reported course even when reported speed is zero."""
        course = boat.get('targetCourse')
        if course is None and boat['targetSpeed'] < 1e-6:
            course = self.initial_course_deg
            if course is None:
                raise ValueError('Situation has no targetCourse; provide --initial-course-deg only if no course is reported')
        if (isinstance(course, bool) or not isinstance(course, (int, float))
                or not math.isfinite(course) or not 0 <= course < 360):
            raise ValueError('Missing or invalid situation targetCourse')
        return course

    def observation(self, packet):
        obs = super().observation(packet)
        course = obs['boat']['targetCourse']
        obs['execution'] = {**obs['execution'], 'max_turn_rate_deg_s': self.max_turn_rate_deg_s}
        obs['action_space'] = dict(speeds_knots=list(self.speeds), courses_deg=reachable_courses(
            course, self.duration, self.max_turn_rate_deg_s))
        return obs

    def _request(self, obs):
        geometry = WaypointGeometry(obs)
        task, boat = obs['task'], obs['boat']
        index = task['next_waypoint_index']
        distance = math.hypot(*geometry.target)
        limit = task['planWaypointList'][index]['routeSpeed']
        course = boat['targetCourse']
        bearing = math.degrees(math.atan2(geometry.target[0], geometry.target[1])) % 360
        relative_bearing = (bearing-course+180)%360-180
        target_side = 'ahead' if abs(relative_bearing) < .005 else ('left' if relative_bearing < 0 else 'right')
        progress = obs.get('navigation_progress', dict(mode='normal', recovery_cycles=0))
        profile = plan_speed(obs, geometry, self.speeds, self.max_turn_rate_deg_s,
                             recovering=progress['mode'] == 'recovery')
        speeds = profile['speeds_knots']
        self.local.turn_diagnostics = dict(observed_next_turn_deg=profile['observed_next_turn_deg'])
        state = dict(
            task='Visit waypoint circles in order.',
            coordinate_system='Metres, east/north; origin=first waypoint. Course: 0=N, 90=E, clockwise. Relative: -=left, +=right.',
            speeds_knots=speeds, progress=progress,
            speed_profile={key: value for key, value in profile.items()
                           if key not in ('speeds_knots', 'observed_next_turn_deg')},
            actual_speed_knots=boat['targetSpeed'], actual_course_deg=course,
            current_waypoint_index=index,
            boat_xy_m=list(geometry.boat_position), target_xy_m=list(geometry.waypoint_positions[index]),
            target_offset_m=list(geometry.target),
            target_bearing_deg=bearing,
            target_relative_bearing_deg=relative_bearing, target_turn_side=target_side,
            distance_to_target_m=distance, arrival_radius_m=task['arrival_radius_m'],
            waypoint_speed_limit_knots=limit / KNOT_MPS,
            command_duration_s=self.duration,
            max_turn_deg=min(self.max_turn_rate_deg_s*self.duration, 179.99),
            recent_decisions=list(getattr(self.local, 'history', ())))
        previous = index - 1
        if previous < 0 or math.hypot(*geometry.waypoints[previous]) > task['arrival_radius_m']:
            previous = None
        criteria = {}
        for speed in speeds:
            for j, target_course in enumerate(obs['action_space']['courses_deg']):
                name = f'speed_{speed:g}__heading_{j:02d}'
                facts = _candidate_facts(obs, geometry, speed*KNOT_MPS,
                    target_course, f'{target_course:.2f} deg', distance, limit, previous)['approximate_effect']
                signed_turn = (target_course-course+180)%360-180
                criteria[name] = dict(speed_knots=speed, absolute_course_deg=target_course,
                    maneuver='straight' if abs(signed_turn) < .005 else ('left' if signed_turn < 0 else 'right'),
                    signed_turn_deg=signed_turn,
                    target_heading_error_deg=abs((bearing-target_course+180)%360-180),
                    distance_after_m=facts['end_checkpoint_distance_m'],
                    distance_after_2_steps_m=facts['same_command_preview']['end_checkpoint_distance_m'],
                    speed_limit_violation=(facts['passing_speed_above_ceiling'] or facts.get('previous_circle_speed_above_ceiling', False)),
                    approach_speed_warning=facts['approach_margin_speed_above_ceiling'])
        instructions = (
            'Visit circles in order; next waypoint is turn preview. Obey speed_profile and speed limits. '
            'Use the fastest safe option. Turn toward target_turn_side; straight when aligned. '
            'For recovery or targets behind, reduce heading error before distance; avoid reversals.')
        return round_model_values(dict(model='kev-latest', state=state, questions=dict(action=dict(
            type='choice', instructions=instructions, criteria=criteria))))

    def _call(self, payload):
        if self.client is not None:
            return self.client(payload)
        if not hasattr(self.local, 'opener'):
            self.local.opener = build_opener(ProxyHandler({}))
        request = Request(self.endpoint, data=json.dumps(payload,allow_nan=False).encode(),
                          headers={'Content-Type':'application/json'}, method='POST')
        try:
            with self.local.opener.open(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as exc:
            raise RuntimeError(f'Kev HTTP {exc.code}: {exc.read(2048).decode(errors="replace")}') from exc
        except (URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'Local Kev request failed ({self.endpoint}): {exc}') from exc

    def __call__(self, packet):
        obs = self.observation(packet)
        if obs['task']['route_complete']:
            return 0., obs['boat']['targetCourse']
        if getattr(self.local,'kev_task_key',None) != self.local.task_key:
            self.local.kev_task_key = copy.deepcopy(self.local.task_key)
            self.local.supervisor = ProgressSupervisor(self.progress_config)
            self.local.kev_waypoint = None
        index = obs['task']['next_waypoint_index']
        if self.local.kev_waypoint != index:
            self.local.kev_waypoint = index
            self.local.history = deque(maxlen=4)
        geometry = WaypointGeometry(obs)
        distance = math.hypot(*geometry.target)
        bearing = math.degrees(math.atan2(geometry.target[0], geometry.target[1])) % 360
        error = abs((bearing - obs['boat']['targetCourse'] + 180) % 360 - 180)
        previous_mode = self.local.supervisor.mode
        obs['navigation_progress'] = self.local.supervisor.update(index, distance, error)
        if previous_mode != self.local.supervisor.mode:
            logging.getLogger(__name__).info('Kev navigation mode=%s waypoint=%d distance=%.1fm heading_error=%.1fdeg',
                self.local.supervisor.mode, index, distance, error)
        payload = self._request(obs)
        started = time.perf_counter()
        # Openers and navigation state are thread-local. A superseded task's
        # slow HTTP request must not hold a client-side lock on the new task.
        result = self._call(payload)
        if isinstance(result,dict) and result.get('truncated'):
            raise RuntimeError('Kev truncated the model input; refusing a partial-state decision')
        answers = result.get('answers') if isinstance(result,dict) else None
        answer = answers.get('action') if isinstance(answers,dict) else None
        if not isinstance(answer,dict) or answer.get('type') != 'choice':
            raise ValueError('Expected Kev Choice answer for action')
        choice = answer.get('choice')
        criteria = payload['questions']['action']['criteria']
        if not isinstance(choice,str) or choice not in criteria:
            raise ValueError('Kev returned a choice outside the available speed/course candidates')
        course = criteria[choice]['absolute_course_deg']
        speed = criteria[choice]['speed_knots']
        self.local.last_decision = dict(choice=choice, candidate_count=len(criteria), selection_mode='single_choice',
            speeds_knots=payload['state']['speeds_knots'], inference_s=time.perf_counter()-started,
            state=payload['state'], options=payload['questions']['action']['criteria'],
            instructions=payload['questions']['action']['instructions'], result=result,
            turn_diagnostics=dict(self.local.turn_diagnostics))
        self.local.history.append(dict(waypoint_index=obs['task']['next_waypoint_index'],
            distance_m=round(distance, 2), chosen_speed_knots=speed, chosen_course_deg=course))
        logging.getLogger(__name__).debug('Kev inference_s=%.6f choice=%s',
            self.local.last_decision['inference_s'],choice)
        return speed*(KNOT_MPS if self.speed_unit=='mps' else 1.),course
