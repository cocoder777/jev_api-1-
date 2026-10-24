"""Kev decision adapter: local System One HTTP service, fixed 15 knots x 16 locally reachable courses."""
import copy
import json
import logging
import math
import threading
import time
from collections import deque
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener

from choice_control import _candidate_facts
from tools.waypoint_geometry import KNOT_MPS, WaypointGeometry
from .waypoint import WaypointPolicy
from .turning import DEFAULT_MAX_TURN_RATE_DEG_S, reachable_courses

KEV_SPEED_KNOTS = 15.0
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
                 initial_course_deg=None, **kwargs):
        super().__init__(**kwargs)
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
        self._lock = threading.Lock()

    def observation(self, packet):
        obs = super().observation(packet)
        course = obs['boat']['targetCourse']
        if course is None:
            course = self.initial_course_deg
            if course is None:
                raise ValueError('No movement course: provide --initial-course-deg for stationary startup')
            obs['boat']['targetCourse'] = course
        obs['execution'] = {**obs['execution'], 'max_turn_rate_deg_s': self.max_turn_rate_deg_s}
        obs['action_space'] = dict(speeds_knots=[KEV_SPEED_KNOTS], courses_deg=reachable_courses(
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
        state = dict(
            task='Control an USV to visit waypoint circles in order at 15 knots while keeping sail stable and short.',
            command_speed_knots=KEV_SPEED_KNOTS,
            coordinate_system='East/north; course clockwise from north: 0=N, 90=E, 180=S, 270=W. Relative angles: negative=left, positive=right.',
            actual_speed_knots=boat['targetSpeed'], actual_course_deg=course,
            current_waypoint_index=index, # total_waypoints=len(geometry.waypoints)-1, 
            target_east_m=geometry.target[0], target_north_m=geometry.target[1],
            target_bearing_deg=bearing,
            target_relative_bearing_deg=relative_bearing, target_turn_side=target_side,
            distance_to_target_m=distance, arrival_radius_m=task['arrival_radius_m'],
            command_duration_s=self.duration,
            max_turn_deg=min(self.max_turn_rate_deg_s*self.duration, 179.99),
            recent_decisions=list(getattr(self.local, 'history', ())),
            forecast_note='Approximate forecasts obey the configured turn-rate limit.')
        previous = index - 1
        if previous <= 0 or math.hypot(*geometry.waypoints[previous]) > task['arrival_radius_m']:
            previous = None
        criteria = {}
        for j, target_course in enumerate(obs['action_space']['courses_deg']):
            name = f'heading_{j:02d}'
            facts = _candidate_facts(obs, geometry, KEV_SPEED_KNOTS*KNOT_MPS,
                target_course, f'{target_course:.2f} deg', distance, limit, previous)['approximate_effect']
            signed_turn = (target_course-course+180)%360-180
            turn = abs(signed_turn)
            criteria[name] = dict(absolute_course_deg=target_course,
                maneuver='straight' if abs(signed_turn) < .005 else ('left' if signed_turn < 0 else 'right'),
                turn_deg=turn, signed_turn_deg=signed_turn,
                target_heading_error_deg=abs((bearing-target_course+180)%360-180),
                distance_after_m=facts['end_checkpoint_distance_m'],
                distance_after_2_steps_m=facts['same_command_preview']['end_checkpoint_distance_m'],
                speed_limit_violation=(facts['passing_speed_above_ceiling'] or facts.get('previous_circle_speed_above_ceiling', False)),
                approach_speed_warning=facts['approach_margin_speed_above_ceiling'])
        instructions = (
            'Reach the current waypoint circle. Respect speed limits. Choose a maneuver on target_turn_side; '
            'straight when ahead. Prefer smaller target heading error, '
            'then smaller forecast distance and less turning. Keep straight when aligned. '
            'A target behind the boat may require turning while distance initially increases; avoid repeated reversals.')
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
        if getattr(self.local,'kev_task_key',None) != self.local.task_key:
            self.local.kev_task_key = copy.deepcopy(self.local.task_key)
            self.local.history = deque(maxlen=4)
        payload = self._request(obs)
        started = time.perf_counter()
        with self._lock:
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
            raise ValueError('Kev returned a choice outside the 16 courses')
        course = criteria[choice]['absolute_course_deg']
        self.local.last_decision = dict(choice=choice, candidate_count=16, selection_mode='single_choice',
            speeds_knots=[KEV_SPEED_KNOTS], inference_s=time.perf_counter()-started,
            state=payload['state'], options=payload['questions']['action']['criteria'],
            instructions=payload['questions']['action']['instructions'], result=result)
        self.local.history.append(dict(waypoint_index=obs['task']['next_waypoint_index'],
            distance_m=round(math.hypot(*WaypointGeometry(obs).target), 2), chosen_course_deg=course))
        logging.getLogger(__name__).info('Kev inference_s=%.6f choice=%s',
            self.local.last_decision['inference_s'],choice)
        return KEV_SPEED_KNOTS*(KNOT_MPS if self.speed_unit=='mps' else 1.),course
