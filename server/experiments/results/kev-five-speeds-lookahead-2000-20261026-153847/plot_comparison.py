"""Plot the current real run in the same display coordinates as the old map."""
import json
from pathlib import Path
import sys
from collections import Counter

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

OUTPUT = Path(__file__).resolve().parent
ROOT = OUTPUT.parents[3]
sys.path.insert(0, str(ROOT))
from server.experiments.scenarios import ROUTES

summary = json.loads((OUTPUT / 'summary.json').read_text())
rows = [json.loads(line) for line in (OUTPUT / 'crossing_km/trajectory.jsonl').open()]
old_dir = ROOT / 'server/experiments/results/kev-turn45-2000'
old = [json.loads(line) for line in (old_dir / 'crossing_km.jsonl').open()]
points = ROUTES['crossing_km']
shift = points[1]
positions = [(row['x'] + shift[0], row['y'] + shift[1]) for row in rows]


def draw(ax, positions, radius, title):
    ax.plot(*zip(*points), '--', color='black', linewidth=1.3, label='Reference order')
    ax.plot(*zip(*positions), color='#0078b8', linewidth=1.8, label='Actual model trajectory')
    ax.scatter(*points[0], color='#13965e', marker='s', s=45, label='Start', zorder=4)
    ax.scatter(*positions[-1], color='#cf483d', marker='x', s=55, label='Last position', zorder=5)
    for index, point in enumerate(points):
        label = 'S' if index == 0 else ('F' if index == len(points) - 1 else f'P{index}')
        ax.annotate(label, point, xytext=(6, 10), textcoords='offset points', fontsize=9)
        if index:
            ax.add_patch(Circle(point, radius, fill=False, color='gray', linewidth=1., alpha=.65))
    ax.set(xlim=(-700, 1200), ylim=(-700, 1200), xlabel='East (m)', ylabel='North (m)', title=title, aspect='equal')
    ax.grid(alpha=.25)
    ax.legend(loc='upper right', fontsize=9)


fig, ax = plt.subplots(figsize=(10, 9))
draw(ax, positions, 100, 'Updated Kev: five speeds + waypoint lookahead, fixed coordinates, progress supervision')
fig.text(.5, .025,
         f"{len(rows)} samples | interval 1 s | 10/20/30/40/50 knots | arrival radius 100 m | "
         f"reached {summary['reached_waypoints']}/10 | {summary['reason']}",
         ha='center', fontsize=10)
fig.tight_layout(rect=(0, .055, 1, 1))
fig.savefig(OUTPUT / 'crossing_km-route-zoom.png', dpi=180)
plt.close(fig)

fig, axes = plt.subplots(1, 2, figsize=(16, 8))
draw(axes[0], [(r['x'], r['y']) for r in old], 50, 'Previous run: 15 knots, radius 50 m, reached 4/10')
draw(axes[1], positions, 100,
     f"Updated run: five speeds + waypoint lookahead, radius 100 m, reached {summary['reached_waypoints']}/10")
fig.text(.5, .025, 'Same crossing map and display frame; internal decision coordinates are anchored at P1.', ha='center', fontsize=10)
fig.tight_layout(rect=(0, .055, 1, 1))
fig.savefig(OUTPUT / 'comparison.png', dpi=180)
plt.close(fig)

decisions = [r['decision'] for r in rows if r['decision']]
audit = dict(real_decisions=len(decisions),
             selected_speeds_knots=dict(Counter(r['command']['speed'] for r in rows)),
             candidate_counts=dict(Counter(d['candidate_count'] for d in decisions)),
             recovery_decisions=summary['recovery_decisions'],
             display_offset_from_internal_m=shift,
             max_speed_knots=summary['max_speed_knots'],
             max_actual_turn_rate_deg_s=summary['max_actual_turn_rate_deg_s'])
(OUTPUT / 'audit.json').write_text(json.dumps(audit, indent=2) + '\n')
print(json.dumps(audit, indent=2))
