#!/usr/bin/env python3
"""Reproduce v12 against the retained v10 cases using the production policy."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from simulate_lowering import Policy,Model,simulate
OUT=Path(__file__).parent
BASE=OUT.parent/'fast-return-v10'
summary={'limits':['Unvalidated contact/plant model; no physical speed or reliability guarantee.',
                  'Recorded-input replay establishes command scope, not future physical motion.']}
for fast,file in ((False,'baseline-simulation.json'),(True,'fast-simulation.json')):
    old=json.loads((BASE/file).read_text())['results']
    policy=Policy(fast=fast);results=[];traces={}
    for before in old:
        after,trace=simulate(policy,Model(**before['model']),before['name']);results.append(after)
        assert (before['outcome']=='lower_complete')==(after['outcome']=='lower_complete'),(before['name'],before['outcome'],after['outcome'])
        assert after['peak_target_lead_rad']<=.2401
        if not fast: assert before==after,before['name']
        # All motion through support confirmation must be identical.
        early=lambda r:[x for x in r['transitions'] if x['phase'] in ('stopping','reaching','committing','catching','descending')][:5]
        assert early(before)==early(after),before['name']
        if fast and before['name'] in ('nominal','trial_rebound_stress','trial_v3_delayed_impact'):
            with (OUT/(before['name']+'.csv')).open('w') as f:
                w=csv.writer(f);w.writerow(['time_s','tilt_deg','rate_dps','arm_l','arm_r','target_l','target_r','torque_l','torque_r','phase','committed','wheel_rad_s','setpoint','wheel_request','supported']);w.writerows(trace)
    good=[(a,b) for a,b in zip(old,results) if a['outcome']==b['outcome']=='lower_complete']
    ratios=[(a['first_flat_s']-a['catch_s'])/(b['first_flat_s']-b['catch_s']) for a,b in good]
    summary['fast' if fast else 'normal']=dict(cases=len(results),completions=len(good),completion_classifications_identical=True,
        changed_fault_reasons=[dict(name=a['name'],before=a['outcome'],after=b['outcome']) for a,b in zip(old,results) if a['outcome']!=b['outcome']],
        normal_results_identical=not fast,precontact_transitions_identical=True,
        median_support_speed_ratio=statistics.median(ratios),support_speed_ratio_range=[min(ratios),max(ratios)],
        baseline_sha256=hashlib.sha256((BASE/file).read_bytes()).hexdigest(),
        nominal_before=old[0],nominal_after=results[0])
    if fast:(OUT/'fast-simulation.json').write_text(json.dumps(dict(results=results),indent=2)+'\n')
    print('fast' if fast else 'normal',len(results),len(good),statistics.median(ratios),flush=True)
# Replay prior physical normal/fast inputs; do not copy their raw files.
spec=importlib.util.spec_from_file_location('lower_replay',BASE/'replay.py');replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
folder=ROOT/'output/lower-v12-replay-baseline';folder.mkdir(exist_ok=True)
(folder/'balance_lower.h').write_bytes(subprocess.check_output(['git','show','45c1a94:src/balance_lower.h'],cwd=ROOT))
oldnormal,newnormal,oldfast,newfast=Policy(folder),Policy(),Policy(folder,fast=True),Policy(fast=True)
summary['replay']=[]
for stem in ('bal_20260920_forward_catch_v7_success_wifi','bal_20260920_forward_catch_v8_stop_wifi','bal_20260920_forward_catch_v9_success_wifi','bal_20260921T011755Z_fast_tip_success_drift_wifi','bal_20260921T013959Z_fast_lower_speed_wifi'):
    path=ROOT/'telemetry_logs'/(stem+'.csv')
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(x for x in path.read_text().splitlines() if not x.startswith('#'))]
    a,ar=replay.replay(oldnormal,rows);b,br=replay.replay(newnormal,rows);assert a==b and ar==br
    a,ar=replay.replay(oldfast,rows);b,br=replay.replay(newfast,rows)
    support=next(x[0] for x in a if int(x[3])==3)
    divergence=next((x[0] for x,y in zip(a,b) if x!=y),None)
    assert divergence is not None and divergence>support
    summary['replay'].append(dict(source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),normal_identical=True,support_ms=support,fast_divergence_ms=divergence))
summary['header_sha256']=hashlib.sha256((ROOT/'src/balance_lower.h').read_bytes()).hexdigest()
summary['simulator_sha256']=hashlib.sha256((ROOT/'scripts/simulate_lowering.py').read_bytes()).hexdigest()
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('Five normal replays identical; fast divergence after support only',flush=True)
