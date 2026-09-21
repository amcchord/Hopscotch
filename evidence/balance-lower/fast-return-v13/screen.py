#!/usr/bin/env python3
"""Paired policy screen plus post-catch low reported torque regression stress."""
import csv,hashlib,importlib.util,json,statistics,subprocess,sys,types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'scripts'))
from simulate_lowering import Policy,Model,simulate
base_dir=ROOT/'output/lower-v13-baseline';base_dir.mkdir(exist_ok=True)
(base_dir/'balance_lower.h').write_bytes(subprocess.check_output(['git','show','09d2e01:src/balance_lower.h'],cwd=ROOT))
reference=json.loads((OUT.parent/'fast-return-v12/fast-simulation.json').read_text())['results']
normal_reference=json.loads((OUT.parent/'fast-return-v10/baseline-simulation.json').read_text())['results']
new=Policy(fast=True);old=Policy(base_dir,fast=True);normal=Policy()
results=[]
for before in reference:
    after,trace=simulate(new,Model(**before['model']),before['name']);results.append(after)
    assert (before['outcome']=='lower_complete')==(after['outcome']=='lower_complete'),(before['name'],before['outcome'],after['outcome'])
    if before['name']=='nominal':
        with (OUT/'nominal.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['time_s','tilt_deg','rate_dps','arm_l','arm_r','target_l','target_r','torque_l','torque_r','phase','committed','wheel_rad_s','setpoint','wheel_request','supported']);w.writerows(trace)
for before in normal_reference:
    after,_=simulate(normal,Model(**before['model']),before['name']);assert before==after,before['name']
print('329 fast classifications and 329 exact normal model results preserved',flush=True)
# Keep physical floor/contact forces intact. After support confirmation, scale
# only reported torque supplied to the controller, not plant support forces.
source_path=ROOT/'scripts/simulate_lowering.py';source=source_path.read_text()
assert source.count('*velocities,*torques)')==1
source=source.replace('*velocities,*torques)', '*velocities,*(x*reported_scale if phase==3 and contact_time is not None and t-contact_time>=low_torque_after else x for x in torques))')
module=types.ModuleType('low_reported_torque_model');module.__file__=str(source_path)
sys.modules[module.__name__]=module;exec(compile(source,str(source_path),'exec'),module.__dict__)
low=[]
for delay,scale in ((d,s) for d in (0.,.8) for s in (0.,.1,.25,.5)):
    module.reported_scale=scale;module.low_torque_after=delay
    for name in ('nominal','trial_rebound_stress','trial_uneven_floor','trial_asymmetric_servo','trial_v3_delayed_impact'):
        params=next(r['model'] for r in reference if r['name']==name)
        a,_=module.simulate(old,module.Model(**params),name);b,trace=module.simulate(new,module.Model(**params),name)
        low.append(dict(low_torque_after_support_s=delay,reported_scale=scale,baseline=a,candidate=b))
        assert b['outcome']=='lower_complete',(scale,name,b['outcome'])
print('40 low-reported-torque contact cases complete',flush=True)
# The failure's exact recorded inputs must reproduce the old parked targets.
spec=importlib.util.spec_from_file_location('recorded_replay',OUT.parent/'fast-return-v10/replay.py');rep=importlib.util.module_from_spec(spec);spec.loader.exec_module(rep)
archive=ROOT/'telemetry_logs/bal_20260921T020520Z_v12_contact_stop_wifi.csv'
rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(x for x in archive.read_text().splitlines() if not x.startswith('#'))]
# One extra stationary frame crosses the strict >3000 ms timeout, whose
# internal tick is slightly earlier than the saved 50 Hz snapshot timestamp.
rows.append(dict(rows[-1],t_ms=rows[-1]['t_ms']+20,sample_dt_ms=20))
a,ar=rep.replay(old,rows);b,br=rep.replay(new,rows)
support=next(x[0] for x in a if int(x[3])==3)
a3=[x for x in a if int(x[3])==3];b3=[x for x in b if int(x[3])==3]
assert ar=='lower_descent_timeout';assert len(set(x[1] for x in a3))==1
first_motion=next(x[0] for x in b3 if abs(x[1]-b3[0][1])>.0001)
assert first_motion>support and first_motion-support<=100
# Frozen sensors cannot follow a changed command, so do not claim completion.
replay=dict(source=str(archive.relative_to(ROOT)),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),support_ms=support,old_reason=ar,new_fixed_sensor_reason=br,old_target_constant=True,new_first_return_ms=first_motion,new_max_target_lead=max(abs(x[j+1]-(next(r for r in rows if r['t_ms']==x[0])['arm_'+side]-rows[0]['arm_'+side+'_tgt'])) for x in b3 for j,side in enumerate(('l','r'))),extra_stationary_frame_ms=20,limits='Recorded sensors stay parked; this verifies released command, not future physical completion.')
# Target updates obey .06 rad; a held target can gain 1 mrad of error when
# the measured arm moves during a backward-rate pause. Record that separately.
by_time={r['t_ms']:r for r in rows}
for previous,current in zip(b3,b3[1:]):
    for j,side in enumerate(('l','r')):
        if abs(current[j+1]-previous[j+1])>1e-6:
            measured=by_time[current[0]]['arm_'+side]-rows[0]['arm_'+side+'_tgt']
            assert abs(current[j+1]-measured)<=.06001
replay['target_updates_bounded_rad']=.06
replay['paused_measured_arm_motion_note']='Held target error reaches 0.061 rad after 0.001-rad measured motion during a backward-rate pause.'
oldnormal=Policy(base_dir)
normal_replays=[]
for stem in ('bal_20260920_forward_catch_v7_success_wifi','bal_20260920_forward_catch_v8_stop_wifi','bal_20260920_forward_catch_v9_success_wifi','bal_20260921T011755Z_fast_tip_success_drift_wifi','bal_20260921T013959Z_fast_lower_speed_wifi','bal_20260921T020520Z_v12_contact_stop_wifi'):
    path=ROOT/'telemetry_logs'/(stem+'.csv')
    original=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(x for x in path.read_text().splitlines() if not x.startswith('#'))]
    assert rep.replay(oldnormal,original)==rep.replay(normal,original)
    normal_replays.append(stem)
print('Six normal command replays identical; failure target resumes after support',flush=True)
summary=dict(baseline_source='09d2e01',header_sha256=hashlib.sha256((ROOT/'src/balance_lower.h').read_bytes()).hexdigest(),simulator_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),normal_exact_cases=len(normal_reference),fast_cases=len(results),fast_completions=sum(r['outcome']=='lower_complete' for r in results),changed_fault_reasons=[dict(name=a['name'],before=a['outcome'],after=b['outcome']) for a,b in zip(reference,results) if a['outcome']!=b['outcome']],low_torque_cases=len(low),low_torque_old_completions=sum(r['baseline']['outcome']=='lower_complete' for r in low),low_torque_new_completions=sum(r['candidate']['outcome']=='lower_complete' for r in low),failure_replay=replay,normal_command_replays=normal_replays,limits=['Approximate contact model; no physical acceptance claimed.','Torque stress scales feedback only after support; it is not an identified sensor model.'])
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(OUT/'fast-simulation.json').write_text(json.dumps(dict(results=results),indent=2)+'\n');(OUT/'low-torque.json').write_text(json.dumps(low,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
