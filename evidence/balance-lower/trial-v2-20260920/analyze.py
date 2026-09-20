#!/usr/bin/env python3
"""Recorded v2 impact/rebound; pinned policy replay, not a counterfactual trial."""
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
LOG = ROOT / 'telemetry_logs/bal_20260920_forward_catch_v2_wifi.csv'
PHASES = ['idle','stopping','preparing','descending','ground_hold','retracting',
          'reserved','complete','fault','committing','catching']


def phase(row):
    return (int(row['pilot_flags']) >> 8) & 15


def analyze():
    lines = LOG.read_text().splitlines()
    meta = dict(line[2:].split('=', 1) for line in lines if line.startswith('# ') and '=' in line)
    rows = [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader(line for line in lines if not line.startswith('#'))]
    active = [r for r in rows if phase(r)]
    trigger = active[0]
    forward = [trigger['arm_l_tgt'], trigger['arm_r_tgt']]
    for row in rows:
        row['reach_l'] = forward[0] - row['arm_l']
        row['reach_r'] = row['arm_r'] - forward[1]
    fields = ('t_ms','roll','roll_rate','arm_l_torque','arm_r_torque',
              'reach_l','reach_r','arm_l_vel','arm_r_vel','motor_vel')
    transitions, previous = [], 0
    for row in active:
        if phase(row) != previous:
            transitions.append(dict(phase=PHASES[phase(row)], **{k: row[k] for k in fields}))
            previous = phase(row)
    commit = next(r for r in active if phase(r) == 9)
    sweep = next(r for r in active if r['t_ms'] > commit['t_ms']
                 and forward[0] - r['arm_l_tgt'] > 1.31)
    first_load = next(r for r in active if phase(r) in (9, 10)
                      and max(abs(r['arm_l_torque']), abs(r['arm_r_torque'])) >= .4)
    summary = dict(log=str(LOG.relative_to(ROOT)), samples=len(rows),
                   csv_sha256=hashlib.sha256(LOG.read_bytes()).hexdigest(),
                   wire_sha256=hashlib.sha256(LOG.with_suffix('.wire').read_bytes()).hexdigest(),
                   features=int(meta['telemetry_features']), duration_s=int(meta['run_duration_ms'])/1000,
                   end_reason=meta['end_reason'], inferred_forward_rad=forward,
                   operator='Arms seemed too far forward before lean; almost bounced off the arms and fell backward.',
                   transitions=transitions,
                   sweep_start={k: sweep[k] for k in fields},
                   sweep_start_drop_deg=commit['roll']-sweep['roll'],
                   first_load={k: first_load[k] for k in fields},
                   first_load_drop_deg=commit['roll']-first_load['roll'],
                   minimum_forward_rate_dps=min(r['roll_rate'] for r in active),
                   supported_samples=sum(phase(r)==3 for r in rows),
                   final_sample={k: rows[-1][k] for k in fields},
                   timing_max={k:max(r[k] for r in active) for k in
                               ('sample_dt_ms','inner_dt_max_us','update_age_ms','imu_age_ms',
                                'feedback_age_l_ms','feedback_age_r_ms')})
    return rows, active, summary


def replay(rows, active, summary):
    out = ROOT / 'output/lowering-v2-trial-replay'
    out.mkdir(parents=True, exist_ok=True)
    for target, source in (('balance_lower.h','src/balance_lower.h'), ('bridge.cpp','scripts/lowering_bridge.cpp')):
        (out / target).write_bytes(subprocess.check_output(['git','show','13d4ee2:'+source], cwd=ROOT))
    subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror','-shared','-fPIC',
                    '-I'+str(out),str(out/'bridge.cpp'),'-o',str(out/'policy.so')],check=True)
    lib = C.CDLL(str(out/'policy.so'))
    floats = C.POINTER(C.c_float)
    lib.lower_new.restype=C.c_void_p
    lib.lower_delete.argtypes=[C.c_void_p]
    lib.lower_request.argtypes=[C.c_void_p,C.c_uint32,floats,C.c_bool,C.c_float,C.c_float]
    lib.lower_request.restype=C.c_bool
    lib.lower_step.argtypes=[C.c_void_p,C.c_uint32,C.c_float,floats,C.c_bool,floats]
    lib.lower_reason.argtypes=[C.c_void_p];lib.lower_reason.restype=C.c_char_p
    handle=lib.lower_new(); result=(C.c_float*7)()
    start=rows.index(active[0]);forward=summary['inferred_forward_rad']
    transitions,mismatches=[],[];previous=0
    try:
        for index in range(start,len(rows)):
            r=rows[index]
            v=(C.c_float*11)(r['roll'],r['roll_rate'],rows[index-1]['setpoint']-r['roll'],
                            r['bl_vel'],r['br_vel'],r['arm_l']-forward[0],r['arm_r']-forward[1],
                            r['arm_l_vel'],r['arm_r_vel'],r['arm_l_torque'],r['arm_r_torque'])
            now=int(r['t_ms'])
            if index==start:
                assert lib.lower_request(handle,now,v,True,1.768,-1.767)
            lib.lower_step(handle,now,r['sample_dt_ms']/1000,v,True,result)
            current=int(result[2])
            if current!=previous:
                transitions.append(dict(phase=PHASES[current],t_ms=now,reason=lib.lower_reason(handle).decode()))
                previous=current
            if current!=phase(r):mismatches.append(now)
        summary['replay']=dict(source='13d4ee2 (same policy as installed dd74154)',
                               scope='Measured inputs through pinned C++; not physical simulation',
                               caveat='Forward inferred from settled target; center sign from historical calibration. Export/control sampling can differ within a tick.',
                               transitions=transitions,phase_mismatch_t_ms=mismatches,
                               final_reason=lib.lower_reason(handle).decode())
    finally:lib.lower_delete(handle)
    # The exported preparation confirmation does not reproduce exactly. The
    # helper's input is read before logging, while live arm/gyro fields can be
    # updated in between. Do not claim a full exact replay. Separately initialize
    # at the observed committed posture to isolate the impact decisions.
    handle=lib.lower_new()
    commit=next(r for r in active if phase(r)==9)
    now=int(commit['t_ms'])
    v=(C.c_float*11)(commit['roll'],0,0,commit['bl_vel'],commit['br_vel'],
                    commit['arm_l']-forward[0],commit['arm_r']-forward[1],0,0,0,0)
    assert lib.lower_request(handle,now-680,v,True,1.768,-1.767)
    for tick in range(35):lib.lower_step(handle,now-680+tick*20,.02,v,True,result)
    assert int(result[2])==9
    transitions=[dict(phase='committing',t_ms=now,initialization='Observed committed posture')]
    previous=9;mismatches=[]
    try:
        for r in active:
            if r['t_ms']<=now:continue
            v=(C.c_float*11)(r['roll'],r['roll_rate'],0,r['bl_vel'],r['br_vel'],
                            r['arm_l']-forward[0],r['arm_r']-forward[1],r['arm_l_vel'],r['arm_r_vel'],
                            r['arm_l_torque'],r['arm_r_torque'])
            lib.lower_step(handle,int(r['t_ms']),r['sample_dt_ms']/1000,v,True,result)
            current=int(result[2])
            if current!=previous:
                transitions.append(dict(phase=PHASES[current],t_ms=int(r['t_ms']),reason=lib.lower_reason(handle).decode()))
                previous=current
            if current!=phase(r):mismatches.append(int(r['t_ms']))
        summary['committed_replay']=dict(scope='Seeded at observed commitment; only subsequent measured-input decisions',
                                        transitions=transitions,phase_mismatch_t_ms=mismatches,
                                        final_reason=lib.lower_reason(handle).decode())
        assert lib.lower_reason(handle)==b'lower_wrong_direction'
        assert not result[3]
    finally:lib.lower_delete(handle)


def plot(rows, active, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    start=next(r['t_ms'] for r in active if phase(r)==9)
    data=[r for r in rows if r['t_ms']>=start-700]
    t=[(r['t_ms']-start)/1000 for r in data]
    fig,ax=plt.subplots(4,1,figsize=(10,10),sharex=True,constrained_layout=True)
    fig.suptitle('Physical v2 trial: early moving-arm contact and backward rebound\nRecorded telemetry · body still near upright at impact')
    ax[0].plot(t,[r['roll'] for r in data],label='Body tilt (higher = backward)')
    ax[0].set_ylabel('degrees');ax[0].legend()
    ax[1].plot(t,[r['roll_rate'] for r in data],label='Fast body rate')
    ax[1].axhline(0,color='gray',lw=.7);ax[1].set_ylabel('degrees/s');ax[1].legend()
    f=summary['inferred_forward_rad']
    for key,values in [('Left measured',[r['reach_l'] for r in data]),
                       ('Right measured',[r['reach_r'] for r in data]),
                       ('Left target',[f[0]-r['arm_l_tgt'] for r in data]),
                       ('Right target',[r['arm_r_tgt']-f[1] for r in data])]:
        ax[2].plot(t,values,label=key,ls='--' if 'target' in key else '-')
    ax[2].set_ylabel('forward reach rad');ax[2].legend(ncol=2)
    for key,label in [('arm_l_torque','Left load magnitude'),('arm_r_torque','Right load magnitude')]:
        ax[3].plot(t,[abs(r[key]) for r in data],label=label)
    ax[3].axhline(.4,color='gray',ls=':',label='First-load threshold')
    ax[3].set_ylabel('Nm');ax[3].legend();ax[3].set_xlabel('Seconds from deliberate departure')
    for a in ax:
        a.grid(alpha=.2)
        a.axvline(0,color='gray',ls=':')
        a.axvline((summary['first_load']['t_ms']-start)/1000,color='#c45420',ls=':')
    fig.savefig(OUT/'impact-rebound.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    rows,active,summary=analyze()
    replay(rows,active,summary)
    plot(rows,active,summary)
    (OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
