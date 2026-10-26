"""Compare the user's saved baseline with the latest real Kev rerun."""
import json
import math
from collections import Counter
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

OUTPUT = Path(__file__).resolve().parent
ROOT = OUTPUT.parents[3]
sys.path.insert(0,str(ROOT))
from server.experiments.scenarios import ROUTES

points = ROUTES['crossing_km']
old_dir = ROOT/'server/experiments/results/kev-turn45-2000'
old = [json.loads(line) for line in (old_dir/'crossing_km.jsonl').open()]
rows = [json.loads(line) for line in (OUTPUT/'full_route/trajectory.jsonl').open()]
summary = json.loads((OUTPUT/'full_route/summary.json').read_text())
positions = [(r['x']+points[1][0],r['y']+points[1][1]) for r in rows]
old_positions = [(r['x'],r['y']) for r in old]


def draw(ax, positions, radius, title):
    ax.plot(*zip(*points),'--',color='black',linewidth=1.3,label='Waypoint order')
    ax.plot(*zip(*positions),color='#0078b8',linewidth=1.8,label='Actual Kev trajectory')
    ax.scatter(*positions[0],marker='s',s=45,color='#13965e',label='Start',zorder=5)
    ax.scatter(*positions[-1],marker='x',s=55,color='#cf483d',label='Last position',zorder=5)
    for i,p in enumerate(points):
        label='S' if i==0 else ('F' if i==len(points)-1 else f'P{i}')
        ax.annotate(label,p,xytext=(5,9),textcoords='offset points',fontsize=9)
        if i:ax.add_patch(Circle(p,radius,fill=False,color='gray',alpha=.6))
    ax.set(xlabel='East (m)',ylabel='North (m)',aspect='equal',title=title)
    ax.grid(alpha=.2)
    ax.legend(loc='best',fontsize=8)


fig,ax=plt.subplots(figsize=(10,9))
draw(ax,positions,100.,f"Turn input restored: reached {summary['reached_waypoints']}/10; one speed violation")
xs,ys=zip(*(points+positions))
ax.set_xlim(min(xs)-200,max(xs)+200)
ax.set_ylim(min(ys)-200,max(ys)+200)
fig.text(.5,.025,f"{len(rows)} samples | 1 s | 10/20/30/40/50 knots | arrival radius 100 m | {summary['reason']}",ha='center',fontsize=9)
fig.tight_layout(rect=(0,.055,1,1));fig.savefig(OUTPUT/'latest-route.png',dpi=180);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(16,8))
draw(axes[0],old_positions,50.,'Saved baseline: 15 knots, radius 50 m, reached 4/10')
draw(axes[1],positions,100.,f"Latest: five speed classes, radius 100 m, reached {summary['reached_waypoints']}/10")
for ax in axes:
    ax.set_xlim(min(xs)-200,max(xs)+200)
    ax.set_ylim(min(ys)-200,max(ys)+200)
fig.text(.5,.025,'Saved baseline continues outside this route-area view. Parameters differ; this is a behavior comparison, not an isolated ablation.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.055,1,1));fig.savefig(OUTPUT/'comparison.png',dpi=180);plt.close(fig)

fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
times=[r['time_s'] for r in rows]
axes[0].plot(times,[r['actual_speed_knots'] for r in rows],label='Actual speed')
axes[0].step(times,[r['command']['speed'] for r in rows],where='post',label='Command',alpha=.7)
axes[0].set(ylabel='Speed (knots)',title='Latest real rerun: speed and active-target heading error')
axes[1].plot(times,[abs(r['decision']['state']['target_relative_bearing_deg']) if r['decision'] else float('nan') for r in rows],color='#cf483d')
axes[1].set(xlabel='Simulation time (s)',ylabel='Heading error (deg)',ylim=(0,185))
for arrival in summary['arrivals']:
    for ax in axes:ax.axvline(arrival['time_s'],color='gray',linestyle=':',alpha=.6)
    axes[1].annotate(f"P{arrival['index']+1}",(arrival['time_s'],175),fontsize=8)
for ax in axes:ax.grid(alpha=.2)
axes[0].legend();fig.tight_layout();fig.savefig(OUTPUT/'speed-and-heading.png',dpi=180);plt.close(fig)


def distance_to_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.dist(p,(a[0]+t*dx,a[1]+t*dy))


def leg_stats(data,old_format=False):
    result=[]
    for target in range(1,5):
        selected=[r for r in data if (r['target_index'] if old_format else r['target_index']+1)==target and r.get('decision')]
        if not selected:continue
        xy=[(r['x'],r['y']) if old_format else (r['x']+points[1][0],r['y']+points[1][1]) for r in selected]
        maneuvers=Counter()
        for r in selected:
            signed=(r['command']['course']-r['actual_course_deg']+180)%360-180
            relative=r['decision']['state']['target_relative_bearing_deg']
            maneuvers['straight' if abs(signed)<.01 else ('toward_target' if signed*relative>0 else 'away_from_target')]+=1
        result.append(dict(target=f'P{target}',samples=len(selected),
                           max_distance_to_reference_leg_m=max(distance_to_segment(p,points[target-1],points[target]) for p in xy),
                           maneuvers=dict(maneuvers)))
    return result


audit=dict(latest_arrivals=summary['arrivals'],saved_baseline_legs=leg_stats(old,True),
           latest_legs=leg_stats(rows),selected_speeds_knots=dict(Counter(r['command']['speed'] for r in rows)),
           candidate_counts=dict(Counter(r['decision']['candidate_count'] for r in rows if r['decision'])))
(OUTPUT/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(audit,ensure_ascii=False,indent=2))
