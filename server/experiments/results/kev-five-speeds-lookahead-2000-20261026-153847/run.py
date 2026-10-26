"""Reproduce the old crossing map using the current real Kev policy."""
from pathlib import Path
import json
import sys
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from server.experiments import field_tasks as ft
from server.experiments.simulate import scenario
OUTPUT=Path(__file__).resolve().parent
config=ft.TestConfig(steps=2000,dt=1.,start_offset_m=640.,initial_course_deg=90.,max_turn_rate_deg_s=45.)
# Only select the original map fixture. The policy makes real HTTP calls to Kev.
with patch.object(ft,'load_task',side_effect=lambda name:scenario('crossing_km')):
    result=ft.run_task('crossing_km',OUTPUT,config)
(OUTPUT/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({key:result[key] for key in ('passed','completed','reason','steps','reached_waypoints','total_waypoints','speed_violation_count','recovery_decisions','stop_confirmed','elapsed_s')},ensure_ascii=False),flush=True)
