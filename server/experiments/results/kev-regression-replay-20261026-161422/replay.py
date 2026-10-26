"""Real Kev, fixed physical states, one input change per replay variant."""
import copy
import json
from pathlib import Path
from collections import defaultdict, Counter
from urllib.request import build_opener, ProxyHandler, Request

OUTPUT=Path(__file__).resolve().parent
ROOT=OUTPUT.parents[3]
old=[json.loads(l) for l in (ROOT/'server/experiments/results/kev-turn45-2000/crossing_km.jsonl').open()]
new=[json.loads(l) for l in (ROOT/'server/experiments/results/kev-planned-turn-rerun-20261026-160544/full_route/trajectory.jsonl').open()]
opener=build_opener(ProxyHandler({}))
with opener.open('http://127.0.0.1:18028/v1/models',timeout=10) as response:
    (OUTPUT/'models.json').write_bytes(response.read())
indices=(30,31,35,37,60,80,100,120,140,160,180,200)
variants=('latest','old_instructions','old_option_ids','relative_coordinate_fields',
          'no_next_direction','no_recent_history','absolute_turn_field',
          'no_redundant_option_speed','legacy_presentation')


def payload(row):
    d=row['decision']
    return dict(model='kev-latest',state=copy.deepcopy(d['state']),questions=dict(action=dict(
        type='choice',instructions=d['instructions'],criteria=copy.deepcopy(d['options']))))


def mutate(p,variant):
    state=p['state'];q=p['questions']['action'];criteria=q['criteria']
    if variant in ('old_instructions','legacy_presentation'):
        q['instructions']=old[81]['decision']['instructions']
    if variant in ('old_option_ids','legacy_presentation'):
        q['criteria']={key.split('__',1)[1]:value for key,value in criteria.items()}
        criteria=q['criteria']
    if variant in ('relative_coordinate_fields','legacy_presentation'):
        offset=state.pop('target_offset_m')
        state.pop('boat_xy_m',None);state.pop('target_xy_m',None)
        state['target_east_m'],state['target_north_m']=offset
        state['coordinate_system']='East/north; target offsets in metres from the current boat. Course clockwise from north: 0=N, 90=E, 180=S, 270=W. Negative relative angles=left, positive=right.'
    if variant=='no_next_direction':
        for key in ('next_waypoint_xy_m','next_leg_bearing_deg'):
            state['speed_profile'].pop(key,None)
    if variant=='no_recent_history':
        state.pop('recent_decisions',None)
    if variant in ('absolute_turn_field','legacy_presentation'):
        for value in criteria.values():value['turn_deg']=abs(value['signed_turn_deg'])
    if variant in ('no_redundant_option_speed','legacy_presentation'):
        for value in criteria.values():value.pop('speed_knots',None)
    if variant=='legacy_presentation':
        state.pop('speed_profile',None);state.pop('progress',None)
        speed=state.pop('speeds_knots')[0]
        state['command_speed_knots']=speed
        state['task']=f'Control an USV to visit waypoint circles in order at {speed:g} knots while keeping sail stable and short.'
        state['forecast_note']='Approximate forecasts obey the configured turn-rate limit.'
        for previous in state.get('recent_decisions',[]):previous.pop('chosen_speed_knots',None)
    return p


def call(p):
    request=Request('http://127.0.0.1:18028/v1/systemone',data=json.dumps(p,allow_nan=False).encode(),headers={'Content-Type':'application/json'})
    with opener.open(request,timeout=120) as response:return json.load(response)


def evaluate(p,response):
    answer=response['answers']['action'];choice=answer['choice'];option=p['questions']['action']['criteria'][choice]
    turn=option['signed_turn_deg'];before=abs(p['state']['target_relative_bearing_deg'])
    after=option['target_heading_error_deg']
    return dict(choice=choice,course_deg=option['absolute_course_deg'],turn_deg=turn,
                heading_error_before_deg=before,heading_error_after_deg=after,
                kind='straight' if abs(turn)<.01 else ('improves_heading' if after<before-.01 else 'worsens_heading'),
                confidence=answer.get('confidence'))


records=[]
with (OUTPUT/'replays.jsonl').open('w') as log:
    for i in (81,82,83,84,85,86):
        p=payload(old[i]);response=call(p)
        record=dict(source='saved_old',step=i,variant='old_exact',payload=p,response=response,
                    observed_old_choice=old[i]['decision']['choice'],evaluation=evaluate(p,response))
        records.append(record);log.write(json.dumps(record)+'\n');log.flush()
        print('old_exact',i,record['evaluation'],flush=True)
    for variant in variants:
        for i in indices:
            p=mutate(payload(new[i]),variant);response=call(p)
            record=dict(source='latest',step=i,variant=variant,payload=p,response=response,
                        observed_new_choice=new[i]['decision']['choice'],evaluation=evaluate(p,response))
            records.append(record);log.write(json.dumps(record)+'\n');log.flush()
        subset=[r for r in records if r['variant']==variant]
        print(variant,dict(Counter(r['evaluation']['kind'] for r in subset)),flush=True)

summary={}
for variant in ('old_exact',*variants):
    selected=[r for r in records if r['variant']==variant]
    summary[variant]=dict(n=len(selected),choices=dict(Counter(r['evaluation']['kind'] for r in selected)),
                          mean_heading_error_after_deg=sum(r['evaluation']['heading_error_after_deg'] for r in selected)/len(selected))
summary['old_replay_exact_matches']=sum(r['evaluation']['choice']==r['observed_old_choice'] for r in records if r['variant']=='old_exact')
summary['latest_replay_exact_matches']=sum(r['evaluation']['choice']==r['observed_new_choice'] for r in records if r['variant']=='latest')
(OUTPUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
