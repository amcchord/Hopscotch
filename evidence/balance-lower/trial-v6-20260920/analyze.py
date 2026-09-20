#!/usr/bin/env python3
"""Measured v6 wait/contact and production-policy sensor-sequence regression."""
import csv,ctypes as C,hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent
LOG=ROOT/'telemetry_logs/bal_20260920_forward_catch_v6_wifi.csv'
sys.path.insert(0,str(ROOT/'scripts'))
from simulate_lowering import Policy

def phase(r):return (int(r['pilot_flags'])>>8)&15

def replay(policy,rows,forward):
    h=policy.lib.lower_new();o=(C.c_float*7)();events=[];previous=None;first_wheel_stop=None
    try:
        for n,r in enumerate(rows):
            v=(C.c_float*11)(r['roll'],r['roll_rate'],r['angle_err'],r['bl_vel'],r['br_vel'],r['arm_l']-forward[0],r['arm_r']-forward[1],r['arm_l_vel'],r['arm_r_vel'],r['arm_l_torque'],r['arm_r_torque'])
            if n==0:assert policy.lib.lower_request(h,int(r['t_ms']),v,True,1.768,-1.767)
            policy.lib.lower_step(h,int(r['t_ms']),r['sample_dt_ms']/1000,v,True,o)
            if int(o[2])!=previous:
                events.append(dict(t_ms=r['t_ms'],phase=int(o[2]),tilt=r['roll'],rate=r['roll_rate'],wheel_command=o[5]));previous=int(o[2])
            if int(o[2])==10 and o[5]<3.10 and first_wheel_stop is None:first_wheel_stop=r['t_ms']
            if int(o[2])==8:break
        return dict(events=events,reason=policy.lib.lower_reason(h).decode(),first_wheel_stop_ms=first_wheel_stop,
            scope='Replays fixed recorded sensor inputs. Candidate commands and earlier drive handoff change physics; later replay states are not predicted physical outcomes.')
    finally:policy.lib.lower_delete(h)

def main():
    lines=LOG.read_text().splitlines();meta=dict(l[2:].split('=',1) for l in lines if l.startswith('# ') and '=' in l)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(l for l in lines if not l.startswith('#'))]
    active=[r for r in rows if phase(r)];waiting=[r for r in active if phase(r)==1]
    fields=['t_ms','roll','roll_rate','bl_vel','br_vel','arm_l','arm_r','arm_l_tgt','arm_r_tgt','arm_l_torque','arm_r_torque','arm_l_vel','arm_r_vel']
    transitions=[];previous=None
    for r in active:
        if phase(r)!=previous:transitions.append(dict(phase=phase(r),**{k:r[k] for k in fields}));previous=phase(r)
    streak=longest=0
    for r in waiting:
        if r['t_ms']>=27500:break
        calm=abs(r['roll_rate'])<=4 and abs(r['angle_err'])<=2 and max(abs(r['bl_vel']),abs(r['br_vel']))<=.65
        streak=streak+r['sample_dt_ms'] if calm else 0;longest=max(longest,streak)
    first_contact=next(r for r in active if phase(r)==10 and abs(r['arm_l_torque'])>=.4 and abs(r['arm_r_torque'])>=.4)
    folder=ROOT/'output/lowering-v7-recorded-baseline';folder.mkdir(parents=True,exist_ok=True)
    (folder/'balance_lower.h').write_bytes(subprocess.check_output(['git','show','a772ecc:src/balance_lower.h'],cwd=ROOT))
    forward=[active[0]['arm_l_tgt'],active[0]['arm_r_tgt']]
    baseline=replay(Policy(folder),active,forward);candidate=replay(Policy(),active,forward)
    assert baseline['reason']==meta['end_reason']=='lower_wrong_direction'
    assert any(e['phase']==3 for e in candidate['events'])
    summary=dict(samples=len(rows),duration_s=int(meta['run_duration_ms'])/1000,installed_source='a772ecc',fast_selected=True,
        csv_sha256=hashlib.sha256(LOG.read_bytes()).hexdigest(),wire_sha256=hashlib.sha256(LOG.with_suffix('.wire').read_bytes()).hexdigest(),
        end_reason=meta['end_reason'],forward_rad=forward,transitions=transitions,
        stop_wait_s=(transitions[1]['t_ms']-transitions[0]['t_ms'])/1000,
        longest_calm_ms_before_27500=longest,
        waiting_driving_samples=sum(bool(int(r['pilot_flags'])&16) for r in waiting),
        first_two_arm_load={k:first_contact[k] for k in fields},final={k:rows[-1][k] for k in fields},
        installed_replay=baseline,candidate_sensor_sequence=candidate,
        candidate_header_sha256=hashlib.sha256((ROOT/'src/balance_lower.h').read_bytes()).hexdigest())
    (OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
