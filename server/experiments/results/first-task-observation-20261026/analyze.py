"""Analyze recorded telemetry without contacting or controlling the vessel."""
import json
import math
import statistics
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
records = defaultdict(list)
groups = defaultdict(dict)
lengths = Counter()
sources = Counter()
headers = Counter()
errors = []
skews = []
start = end = None
with (OUT/'events.jsonl').open() as stream:
    for line in stream:
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue  # The live recorder may still be appending its final line.
        if row.get('channel') != 'multicast':
            continue
        start = start or row['received_at']
        end = row['received_at']
        lengths[row['length']] += 1
        sources['{}:{}'.format(*row['source'])] += 1
        if 'decode_error' in row:
            errors.append(row)
        if 'state' not in row:
            continue
        state = row['state']
        records[state['platformId']].append(row)
        groups[state['timestamp']][state['platformId']] = row
        header = row['header']
        headers[(header['infoUnitSeq'],header['infoUnitId'],header['secondaryInfoUnitId'])] += 1
        local = datetime.fromisoformat(row['received_at']).astimezone(timezone(timedelta(hours=8)))
        local_ms = ((local.hour*60+local.minute)*60+local.second)*1000+local.microsecond/1000
        skews.append(((state['timestamp']-local_ms+43200000)%86400000-43200000)/1000)


def distance(a,b):
    x = math.radians(a['longitude']-b['longitude'])*math.cos(math.radians((a['latitude']+b['latitude'])/2))
    y = math.radians(a['latitude']-b['latitude'])
    return 6378137*math.hypot(x,y)


def stats(values):
    return dict(min=round(min(values),4),median=round(statistics.median(values),4),
                max=round(max(values),4),mean=round(statistics.mean(values),4)) if values else None


targets = {}
fig,ax = plt.subplots(figsize=(11,6))
for pid,rows in sorted(records.items()):
    gaps = [b['elapsed_s']-a['elapsed_s'] for a,b in zip(rows,rows[1:])]
    fields = ('longitude','latitude','targetSpeed','targetCourse','type','sign','batch')
    entry = dict(count=len(rows),first_received_at=rows[0]['received_at'],last_received_at=rows[-1]['received_at'],
        first_elapsed_s=rows[0]['elapsed_s'],last_elapsed_s=rows[-1]['elapsed_s'],
        interval_s=stats(gaps),first_state=rows[0]['state'],last_state=rows[-1]['state'],
        unique_values={key:len({r['state'][key] for r in rows}) for key in fields})
    if pid >= 4000:
        aligned = [(g[pid]['elapsed_s'],distance(g[1010]['state'],g[pid]['state']))
                   for g in groups.values() if pid in g and 1010 in g]
        if aligned:
            entry['distance_to_own_m'] = stats([d for t,d in aligned])
            entry['first_aligned_distance_m'] = round(aligned[0][1],2)
            entry['last_aligned_distance_m'] = round(aligned[-1][1],2)
            entry['new_updates_beyond_5km'] = sum(d>5000 for t,d in aligned)
            ax.plot([t for t,d in aligned],[d/1000 for t,d in aligned],label=str(pid),linewidth=1.5)
            ax.scatter(aligned[0][0],aligned[0][1]/1000,s=25)
    targets[pid] = entry
ax.axhline(5,color='black',linestyle='--',label='5 km from own vessel')
ax.set(xlabel='Seconds since observation started (not mission start)',ylabel='Distance to own vessel (km)',
       title='Obstacle packets actually received, aligned by message timestamp')
ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(OUT/'target-distance.png',dpi=160);plt.close(fig)
own = records.get(1010,[])
fusion_check = {}
for pid,rows in sorted(records.items()):
    if pid < 4000:
        continue
    point = rows[-1]['state']
    seen, absent, nearest_ids = [], [], set()
    for group in groups.values():
        vessels = [(bid,row['state']) for bid,row in group.items() if 1001 <= bid <= 1010]
        if len(vessels) != 10:
            continue
        bid,dist = min(((bid,distance(st,point)) for bid,st in vessels),key=lambda item:item[1])
        (seen if pid in group else absent).append(dist)
        if pid in group:
            nearest_ids.add(bid)
    fusion_check[pid] = dict(nearest_vessel_ids_when_seen=sorted(nearest_ids),
        max_nearest_vessel_distance_when_seen_m=round(max(seen),2) if seen else None,
        min_nearest_vessel_distance_when_absent_m=round(min(absent),2) if absent else None)
(OUT/'fusion-range-check.json').write_text(json.dumps(fusion_check,ensure_ascii=False,indent=2)+'\n')
result = dict(observation_started_at_utc=start,last_packet_at_utc=end,
    received_packets=sum(lengths.values()),packet_lengths=dict(lengths),sources=dict(sources),
    header_combinations=[dict(infoUnitSeq=a,infoUnitId=b,secondaryInfoUnitId=c,count=n) for (a,b,c),n in headers.items()],
    decode_error_count=len(errors),message_clock_minus_receive_clock_s=stats(skews),targets=targets,
    latest_own_speed_knots=own[-1]['state']['targetSpeed'] if own else None,
    own_zero_speed_samples=sum(r['state']['targetSpeed']==0 for r in own),
    task_status_capture_available=False,
    limitations=['Task/status unicast could not be captured: raw socket unavailable and sudo requires a password.',
        'Mission start and completion require external log confirmation; receiver start is not mission start.',
        'Distances use the same-message-timestamp own-vessel position and a local geographic approximation.',
        'Read-only capture; navigation and collision avoidance behavior were not changed.'])
(OUT/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='targets'},ensure_ascii=False))
for pid,entry in targets.items():
    if pid>=4000:
        print(pid,json.dumps({k:entry[k] for k in ('count','interval_s','first_aligned_distance_m','last_aligned_distance_m','new_updates_beyond_5km') if k in entry},ensure_ascii=False))
