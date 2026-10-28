"""Task-local progress preference for passing a slower, co-directional target.

Only already safe external candidates enter selection. This is a local maneuver,
not a maritime right-of-way model or a guarantee that a pass is feasible.
"""
import math
from tools.navigation_motion import predict_ground_motion
from .prediction import signed_turn, velocity


class OvertakingProgress:
    def __init__(self, config):
        self.config = config
        self.target_id = None
        self.side = 0
        self.details = None

    @property
    def active(self):
        return self.target_id is not None

    def reset(self):
        self.target_id = None
        self.side = 0
        self.details = None

    def _reliable(self, target):
        return (target.position_m is not None and target.speed_mps is not None
                and target.course_deg is not None and target.age_s <= self.config.max_target_age_s)

    def _coordinates(self, own, target):
        forward = velocity(1., target.course_deg)
        right = (forward[1], -forward[0])
        speed = velocity(target.speed_mps, target.course_deg)
        center = tuple(target.position_m[i]+speed[i]*target.age_s for i in (0, 1))
        delta = tuple(own.position_m[i]-center[i] for i in (0, 1))
        return forward, right, sum(delta[i]*forward[i] for i in (0, 1)), sum(delta[i]*right[i] for i in (0, 1))

    @property
    def lateral_goal_m(self):
        return self.config.safe_distance_m + self.config.uncertainty_mps*self.config.action_horizon_s + 5.

    def ready_to_pass(self, scene):
        if not self.active or scene is None or not self.side:
            return False
        own,targets=scene
        target=next((t for t in targets if t.target_id==self.target_id and self._reliable(t)),None)
        if target is None:
            return False
        _,_,_,cross=self._coordinates(own,target)
        return self.side*cross >= self.lateral_goal_m-10.

    def update(self, own, targets, desired, threat_ids):
        current = next((t for t in targets if t.target_id == self.target_id), None)
        if self.active and (current is None or not self._reliable(current)):
            self.reset()
        if current is not None and self.active:
            _, _, along, cross = self._coordinates(own, current)
            if along >= self.lateral_goal_m+20.:
                self.reset()
                return None
        if not self.active:
            eligible = []
            for target in targets:
                if target.target_id not in threat_ids or not self._reliable(target) or target.speed_mps <= .1:
                    continue
                if abs(signed_turn(desired.course_deg,target.course_deg)) > 30.:
                    continue
                if desired.speed_mps <= target.speed_mps+.5:
                    continue
                _, _, along, cross = self._coordinates(own,target)
                if along < 0 and abs(cross) <= self.lateral_goal_m:
                    eligible.append((-along,target))
            if not eligible:
                return None
            current = min(eligible,key=lambda item:item[0])[1]
            self.target_id = current.target_id
        return current

    def select(self, own, target, safe, actions):
        """Prefer lateral clearance, then forward relative progress until ahead."""
        forward, right, along, cross = self._coordinates(own,target)
        horizon = self.config.action_horizon_s
        goal = self.lateral_goal_m
        evaluated = []
        cfg = self.config
        for key, item in safe.items():
            action = actions[key]
            m = predict_ground_motion(own.speed_mps,own.course_deg,action.speed_mps,action.course_deg,
                0.,0.,horizon,dt=cfg.dt_s,initial_turn_rate_deg_s=own.turn_rate_deg_s,
                acceleration_mps2=cfg.acceleration_mps2,deceleration_mps2=cfg.deceleration_mps2,
                max_turn_rate_deg_s=cfg.max_turn_rate_deg_s,max_turn_acceleration_deg_s2=cfg.max_turn_acceleration_deg_s2,
                acceleration_curve=cfg.acceleration_curve,deceleration_curve=cfg.deceleration_curve,
                turn_rate_curve=cfg.turn_rate_curve,acceleration_response=cfg.acceleration_response)
            delta = tuple(m['endpoint'][i]-velocity(target.speed_mps,target.course_deg)[i]*horizon for i in (0,1))
            gain = sum(delta[i]*forward[i] for i in (0,1))
            future_cross = cross+sum(delta[i]*right[i] for i in (0,1))
            evaluated.append((key,gain,future_cross))
        sides = (self.side,) if self.side else (1,-1)
        selected = []
        phase = 'waiting_safe_progress'
        for side in sides:
            if side*cross < goal-10.:
                phase = 'move_out'
                pool = [r for r in evaluated if r[1] > .5 and side*(r[2]-cross) > 2.]
                if pool:
                    error = min(abs(goal-side*r[2]) for r in pool)
                    selected = [r for r in pool if abs(goal-side*r[2]) <= error+10.]
            else:
                phase = 'pass'
                pool = [r for r in evaluated if r[1] > .5 and goal-15. <= side*r[2] <= goal+35.]
                if pool:
                    # Keep the passing lane before optimizing speed. A wide
                    # corridor lets tiny outward turns miss a 100 m waypoint.
                    lateral_error = min(abs(goal-side*r[2]) for r in pool)
                    lane = [r for r in pool if abs(goal-side*r[2]) <= lateral_error+2.]
                    gain = max(r[1] for r in lane)
                    selected = [r for r in lane if r[1] >= max(.5,gain*.9)]
            if selected:
                self.side = side
                break
        if not selected:
            phase = 'waiting_safe_progress'
        self.details = dict(target_id=self.target_id,phase=phase,side='starboard' if self.side>0 else 'port' if self.side<0 else 'undecided',
                            relative_along_m=round(along,2),relative_across_m=round(cross,2),lateral_goal_m=round(goal,2))
        # No safe progress means retain the original safe yielding choices.
        return {row[0]:safe[row[0]] for row in selected} if selected else safe
