"""Validate the absolute-turn feature on fixed and held-out turn states."""
import ast
import copy
import json
from collections import Counter
from pathlib import Path
import sys
from urllib.request import build_opener, ProxyHandler, Request

OUTPUT=Path(__file__).resolve().parent
ROOT=OUTPUT.parents[3]
sys.path.insert(0,str(ROOT))
from server.experiments.simulate import scenario,telemetry
from server.policies.kev_policy import KevPolicy

old=[json.loads(l) for l in (ROOT/'server/experiments/results/kev-turn45-2000/crossing_km.jsonl').open()]
new=[json.loads(l) for l in (ROOT/'server/experiments/results/kev-planned-turn-rerun-20261026-160544/full_route/trajectory.jsonl').open()]
opener=build_opener(ProxyHandler({}))
tree=ast.parse((OUTPUT/'replay.py').read_text())
defs=ast.Module(body=[node for node in tree.body if isinstance(node,ast.FunctionDef)],type_ignores=[])
namespace=globals()
exec(compile(defs,'replay_helpers','exec'),namespace)

samples=[]
for i in (30,31,35,37,60,80,100,120,140,160,180,200):
    samples.append(('failed_states',i,payload(new[i])))
for i in (81,82,83,84,226,227,228,229,366,367,368,369):
    row=old[i]
    packet=scenario('crossing_km')
    packet['params']['currentRoute']=row['target_index']-1
    packet['situation']=[telemetry(row['x'],row['y'],row['speed_mps'],row['actual_course_deg'])]
    policy=KevPolicy(initial_course_deg=90.)
    policy.local.history=copy.deepcopy(row['decision']['state']['recent_decisions'])
    p=policy._request(policy.observation(packet))
    # Heading-only comparison at 10 knots keeps the physical action set fixed.
    p['questions']['action']['criteria']={k:v for k,v in p['questions']['action']['criteria'].items() if k.startswith('speed_10__')}
    p['state']['speeds_knots']=[10.]
    p['state']['speed_profile']['speed_cap_knots']=10.
    samples.append(('held_out_states',i,p))

variants=('latest','absolute_turn_field','absolute_turn_original_order',
          'absolute_turn_old_instructions','absolute_turn_relative_fields','legacy_presentation')
records=[]
with (OUTPUT/'validation.jsonl').open('w') as log:
    for variant in variants:
        for group,i,source in samples:
            p=copy.deepcopy(source)
            if variant=='absolute_turn_original_order':
                opts=p['questions']['action']['criteria']
                for key,value in list(opts.items()):
                    option={}
                    for field,item in value.items():
                        if field=='signed_turn_deg':option['turn_deg']=abs(item)
                        option[field]=item
                    opts[key]=option
            elif variant=='absolute_turn_old_instructions':
                p=mutate(mutate(p,'absolute_turn_field'),'old_instructions')
            elif variant=='absolute_turn_relative_fields':
                p=mutate(mutate(p,'absolute_turn_field'),'relative_coordinate_fields')
            else:
                p=mutate(p,variant)
            result=call(p)
            record=dict(group=group,step=i,variant=variant,payload=p,response=result,evaluation=evaluate(p,result))
            records.append(record);log.write(json.dumps(record)+'\n');log.flush()
        print(variant,{g:dict(Counter(r['evaluation']['kind'] for r in records if r['variant']==variant and r['group']==g)) for g in ('failed_states','held_out_states')},flush=True)
summary={}
for group in ('failed_states','held_out_states'):
    summary[group]={variant:dict(Counter(r['evaluation']['kind'] for r in records if r['group']==group and r['variant']==variant)) for variant in variants}
(OUTPUT/'validation-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
