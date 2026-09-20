#!/usr/bin/env python3
"""Read a fast run in place and retain derived handoff evidence, not raw logs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(path, output):
    lines = path.read_text().splitlines()
    rows = [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader(line for line in lines if not line.startswith('#'))]
    assert rows and all(int(r['pilot_flags']) & 128 for r in rows), 'Need a distinct fast run'
    engage = next(r for r in rows if r['state'] == 2)
    returning = next(r for r in rows if r['state'] == 2 and int(r['flags']) & 32)
    unloaded = next(r for r in rows if r['t_ms'] > returning['t_ms']
                    and abs(r['arm_l_torque']) < .4 and abs(r['arm_r_torque']) < .4)
    recovery = next(r for r in rows if r['state'] == 2 and int(r['flags']) & 128)
    selected = ['t_ms', 'roll', 'roll_rate', 'gyro_raw', 'setpoint', 'base_sp',
                'capture_shift', 'run_curve_shift', 'bl_vel', 'br_vel',
                'arm_tip_frac', 'arm_l_torque', 'arm_r_torque', 'flags']
    events = {name: {k: row[k] for k in selected} for name, row in
              [('capture', engage), ('arm_return', returning),
               ('both_arm_loads_below_0.4_nm', unloaded), ('startup_recovery', recovery),
               ('last_sample', rows[-1])]}
    result = dict(source=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  rows=len(rows), all_samples_fast=True,
                  end_reason=next(line.split('=', 1)[1] for line in lines if line.startswith('# end_reason=')),
                  tip_to_balance_s=(engage['t_ms']-rows[0]['t_ms'])/1000,
                  events=events,
                  limits=['Low measured arm torque suggests support release; contact force is not directly measured.',
                          'The stored trim is a prior estimate, not a measured free equilibrium for this trial.',
                          'This log does not prove that the proposed correction will recover physical balance.'])
    output.mkdir(parents=True, exist_ok=True)
    (output/'fast-trial.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    time = [r['t_ms']/1000 for r in rows]
    for key, label in [('roll', 'Measured tilt'), ('setpoint', 'Balance target')]:
        axes[0].plot(time, [r[key] if key == 'roll' or r['state'] == 2 else float('nan') for r in rows], label=label)
    axes[0].set_ylabel('Degrees')
    for key, label in [('bl_vel', 'Left rear'), ('br_vel', 'Right rear')]:
        axes[1].plot(time, [r[key] for r in rows], label=label)
    axes[1].set_ylabel('Wheel speed (rad/s)')
    axes[2].plot(time, [r['arm_tip_frac'] for r in rows], label='Measured fraction toward tip pose')
    axes[2].set_ylabel('Arm fraction')
    for key, label in [('arm_l_torque', 'Left arm'), ('arm_r_torque', 'Right arm')]:
        axes[3].plot(time, [r[key] for r in rows], label=label)
    axes[3].set_ylabel('Arm torque (Nm)')
    for ax in axes:
        for event, color in [(engage, '#239269'), (returning, '#c18a22'), (unloaded, '#ab4747')]:
            ax.axvline(event['t_ms']/1000, color=color, ls='--', alpha=.65)
        ax.grid(alpha=.2)
        ax.legend(loc='upper left', fontsize=8)
        ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_title('Fast tip-up: quiet capture, then forward roll during arm return\n'
                      f"Capture {engage['t_ms']/1000:.3f} s · return {returning['t_ms']/1000:.3f} s · "
                      f"low arm loads {unloaded['t_ms']/1000:.3f} s")
    axes[-1].set_xlabel('Seconds after run logging starts')
    fig.tight_layout()
    fig.savefig(output/'fast-trial.png', dpi=150)
    plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    analyze(args.csv, args.output)
