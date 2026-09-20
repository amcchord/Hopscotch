#!/usr/bin/env python3
"""Recorded v4 preparation lean and impact/rebound; measured telemetry only."""
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
LOG = ROOT / 'telemetry_logs/bal_20260920_forward_catch_v4_wifi.csv'
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
                 and forward[0] - r['arm_l_tgt'] > 1.26)
    first_load = next(r for r in active if phase(r) in (9, 10)
                      and max(abs(r['arm_l_torque']), abs(r['arm_r_torque'])) >= .4)
    summary = dict(log=str(LOG.relative_to(ROOT)), samples=len(rows),
                   csv_sha256=hashlib.sha256(LOG.read_bytes()).hexdigest(),
                   wire_sha256=hashlib.sha256(LOG.with_suffix('.wire').read_bytes()).hexdigest(),
                   features=int(meta['telemetry_features']), duration_s=int(meta['run_duration_ms'])/1000,
                   end_reason=meta['end_reason'], inferred_forward_rad=forward,
                   operator='Arms contact flipped robot backward; user requests much earlier forward departure and removal of backward lean during arm lowering.',
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
    preparing = next(r for r in active if phase(r)==2)
    last_balanced = next(r for r in reversed(active) if phase(r)==2)
    summary['preparation'] = dict(duration_s=(commit['t_ms']-preparing['t_ms'])/1000, body_backward_deg=commit['roll']-preparing['roll'], base_backward_deg=last_balanced['base_sp']-preparing['base_sp'], initial_tilt=preparing['roll'], launch_tilt=commit['roll'])
    summary['fast_tip_selected'] = any(int(r['pilot_flags']) & 128 for r in rows)
    return rows, active, summary


def plot(rows, active, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    start=next(r['t_ms'] for r in active if phase(r)==9)
    data=[r for r in rows if r['t_ms']>=start-7000]
    t=[(r['t_ms']-start)/1000 for r in data]
    fig,ax=plt.subplots(4,1,figsize=(10,10),sharex=True,constrained_layout=True)
    fig.suptitle('Physical v4 trial: 4.46 degree scheduled backward lean, then contact rebound\nRecorded telemetry · body still near upright at impact')
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
    plot(rows,active,summary)
    (OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
