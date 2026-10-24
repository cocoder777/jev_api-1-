"""Reusable geographic east/north maps, independent of the selected model."""
import json
import math
from pathlib import Path


def load_scenarios(path=Path(__file__).with_name('scenarios.json')):
    scenarios = json.loads(Path(path).read_text())
    if not isinstance(scenarios, dict) or not scenarios:
        raise ValueError('Expected named scenarios')
    for name, spec in scenarios.items():
        points = spec['points_m']
        if len(points) < 2 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError(f'Invalid coordinates for {name}')
        if points[0] != [0, 0]:
            raise ValueError(f'{name}: simulated start must be (0, 0)')
        if not math.isfinite(spec['radius_m']) or spec['radius_m'] <= 0:
            raise ValueError(f'Invalid arrival radius for {name}')
        if not math.isfinite(spec['speed_knots']) or spec['speed_knots'] < 0:
            raise ValueError(f'Invalid speed ceiling for {name}')
    return scenarios


SCENARIOS = load_scenarios()
ROUTES = {name: [tuple(p) for p in spec['points_m']] for name, spec in SCENARIOS.items()}
