#!/usr/bin/env python3
"""Recorded v5 preparation abort; replay only until candidate actions diverge."""
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
LOG = ROOT/'telemetry_logs/bal_20260920_forward_catch_v5_wifi.csv'
sys.path.insert(0,str(ROOT/'scripts'))
from simulate_lowering import Policy


def phase(row):
    return (int(row['pilot_flags']) >> 8) & 15


def replay(policy, rows, forward):
    handle = policy.lib.lower_new()
    result = (C.c_float*7)()
    transitions=[]
    previous=None
    try:
        for n,r in enumerate(rows):
            values=(C.c_float*11)(r['roll'],r['roll_rate'],r['setpoint']-r['roll'],
                r['bl_vel'],r['br_vel'],r['arm_l']-forward[0],r['arm_r']-forward[1],
                r['arm_l_vel'],r['arm_r_vel'],r['arm_l_torque'],r['arm_r_torque'])
            if n==0:
                assert policy.lib.lower_request(handle,int(r['t_ms']),values,True,1.768,-1.767)
            policy.lib.lower_step(handle,int(r['t_ms']),r['sample_dt_ms']/1000,values,True,result)
            if int(result[2])!=previous:
                transitions.append(dict(t_ms=r['t_ms'],phase=int(result[2]),tilt=r['roll'],
                    rate=r['roll_rate'],wheels=[r['bl_vel'],r['br_vel']],
                    arms=[r['arm_l']-forward[0],r['arm_r']-forward[1]],wheel_command=result[5]))
                previous=int(result[2])
            # Later recorded sensor values would no longer follow the new
            # commands. Do not use them as proof of a counterfactual catch.
            if result[6] or int(result[2])==8:
                break
        return dict(transitions=transitions,reason=policy.lib.lower_reason(handle).decode(),
                    stops_at_first_commit_or_fault=True)
    finally:
        policy.lib.lower_delete(handle)


def main():
    lines=LOG.read_text().splitlines()
    metadata=dict(l[2:].split('=',1) for l in lines if l.startswith('# ') and '=' in l)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(l for l in lines if not l.startswith('#'))]
    active=[r for r in rows if phase(r)]
    forward=[active[0]['arm_l_tgt'],active[0]['arm_r_tgt']]
    preparing=next(r for r in active if phase(r)==2)
    final=active[-1]
    baseline_dir=ROOT/'output/lowering-v6-recorded-baseline'
    baseline_dir.mkdir(parents=True,exist_ok=True)
    (baseline_dir/'balance_lower.h').write_bytes(subprocess.check_output(['git','show','17c499c:src/balance_lower.h'],cwd=ROOT))
    before=replay(Policy(baseline_dir),active,forward)
    after=replay(Policy(),active,forward)
    assert before['reason']==metadata['end_reason']=='lower_prepare_disturbed'
    assert before['transitions'][-1]['t_ms']==final['t_ms']
    assert after['transitions'][-1]['phase']==9
    fields=['t_ms','roll','roll_rate','setpoint','base_sp','bl_vel','br_vel','cmd_left','cmd_right',
            'arm_l','arm_r','arm_l_tgt','arm_r_tgt','arm_l_vel','arm_r_vel','arm_l_torque','arm_r_torque']
    summary=dict(samples=len(rows),duration_s=int(metadata['run_duration_ms'])/1000,
        csv_sha256=hashlib.sha256(LOG.read_bytes()).hexdigest(),
        wire_sha256=hashlib.sha256(LOG.with_suffix('.wire').read_bytes()).hexdigest(),
        installed_source='17c499c06b57d8e6d82ffaee3fbf289b722aa8b6',
        candidate_policy_sha256=hashlib.sha256((ROOT/'src/balance_lower.h').read_bytes()).hexdigest(),
        fast_selected=any(int(r['pilot_flags'])&128 for r in rows),
        end_reason=metadata['end_reason'],inferred_forward_rad=forward,
        preparation_duration_s=(final['t_ms']-preparing['t_ms'])/1000,
        preparation_body_change_deg=final['roll']-preparing['roll'],
        maximum_preparation_setpoint=max(r['setpoint'] for r in active if phase(r)==2),
        final_sample={k:final[k] for k in fields},
        recorded_committed_or_supported_samples=sum(phase(r) in (3,4,5,9,10) for r in rows),
        installed_replay=before,candidate_replay=after,
        limitation='Capture stops before contact/return; operator observed supported arms afterward. Replay validates the earlier handoff only, not subsequent physical success.')
    (OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
