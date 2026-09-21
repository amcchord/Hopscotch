#!/usr/bin/env python3
"""Measure fast stand-up travel before intentional driving or lowering."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(path):
    lines = path.read_text().splitlines()
    rows = [{k: float(v) for k,v in r.items()} for r in
            csv.DictReader(l for l in lines if not l.startswith('#'))]
    assert rows and all(int(r['pilot_flags']) & 128 for r in rows)
    capture = next(r for r in rows if r['state']==2)
    returning = next(r for r in rows if r['state']==2 and int(r['flags']) & 32)
    cutoff = next((r['t_ms'] for r in rows if int(r['pilot_flags']) & (2|64)), rows[-1]['t_ms']+1)
    balance = [r for r in rows if r['state']==2 and r['t_ms']<cutoff]
    quiet = [r for r in balance if int(r['flags']) & 64 and r['arm_tip_frac']<.02
             and abs(r['bl_vel'])<.7 and abs(r['br_vel'])<.7 and abs(r['roll_rate'])<4]
    event = lambda bit: next((r['t_ms']/1000 for r in balance if int(r['flags']) & bit),None)
    common = lambda r: .5*(r['bl_vel']+r['br_vel'])
    result = dict(source=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        rows=len(rows), duration_s=rows[-1]['t_ms']/1000,
        end_reason=next(l.split('=',1)[1] for l in lines if l.startswith('# end_reason=')),
        capture_s=capture['t_ms']/1000, capture_angle_deg=capture['roll'],
        arm_return_s=returning['t_ms']/1000, arm_return_angle_deg=returning['roll'],
        ramp_complete_s=event(64), recovery_trigger_s=event(128),
        recovery_settled_s=next((r['t_ms']/1000 for r in balance if int(r['diag_flags']) & 32768),None),
        cutoff_s=cutoff/1000, quiet_samples=len(quiet),
        quiet_median_angle_deg=statistics.median(r['roll'] for r in quiet) if quiet else None,
        peak_integral_deg=max(r['vel_integral'] for r in balance),
        minimum_position_rad=min(r['meas_drift'] for r in balance),
        maximum_position_rad=max(r['meas_drift'] for r in balance),
        minimum_common_speed_rad_s=min(common(r) for r in balance),
        maximum_common_speed_rad_s=max(common(r) for r in balance),
        maximum_wheel_speed_difference_rad_s=max(abs(r['bl_vel']-r['br_vel']) for r in balance))
    return result,balance


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('latest',type=Path)
    parser.add_argument('--references',type=Path,nargs='*',default=[])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    latest,rows=analyze(args.latest)
    result=dict(latest=latest,references=[analyze(p)[0] for p in args.references],
        limits=['Quiet samples are selected observations, not a calibrated measurement of true equilibrium.',
                'Capture and Forward use different arm poses; angle changes cannot establish sensor drift alone.',
                'Wheel radians do not establish ground distance or tire slip.'])
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'observed.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    t=[r['t_ms']/1000 for r in rows]
    for key,label in [('roll','Body angle'),('base_sp','Base target'),('setpoint','Target including recovery correction')]:
        axes[0].plot(t,[r[key] if key!='base_sp' or r[key]!=0 else float('nan') for r in rows],label=label)
    axes[0].set_ylabel('Degrees')
    axes[1].plot(t,[r['meas_drift'] for r in rows],label='Rear-wheel position from upright capture')
    axes[1].set_ylabel('Wheel radians')
    axes[2].plot(t,[r['vel_integral'] for r in rows],label='Learned angle correction')
    axes[2].set_ylabel('Degrees')
    for ax in axes:
        ax.axvline(latest['arm_return_s'],color='#777',ls=':',label='Arm return starts')
        if ax is not axes[0]: ax.axhline(0,color='#777',lw=.5)
        ax.grid(alpha=.2);ax.legend(fontsize=8);ax.spines[['top','right']].set_visible(False)
    axes[0].set_title('Successful fast stand-up: forward surge followed by larger backward recoil')
    axes[-1].set_xlabel('Seconds from run logging start; intentional driving/lowering excluded')
    fig.tight_layout();fig.savefig(args.output/'observed.png',dpi=150);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
