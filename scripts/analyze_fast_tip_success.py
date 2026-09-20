#!/usr/bin/env python3
"""Analyze a completed fast stand-up, excluding subsequent intentional driving."""
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
    rows = [{k: float(v) for k, v in row.items()} for row in
            csv.DictReader(line for line in lines if not line.startswith('#'))]
    assert rows and all(int(r['pilot_flags']) & 128 for r in rows)
    engage = next(r for r in rows if r['state'] == 2)
    returning = next(r for r in rows if r['state'] == 2 and int(r['flags']) & 32)
    ramp = next(r for r in rows if r['state'] == 2 and int(r['flags']) & 64)
    settled = next(r for r in rows if r['state'] == 2 and int(r['diag_flags']) & 32768)
    ready = next(r for r in rows if r['state'] == 2 and int(r['pilot_flags']) & 1)
    driving = next(r for r in rows if r['state'] == 2 and int(r['pilot_flags']) & 2)
    lower = next(r for r in rows if int(r['pilot_flags']) & 64)
    standby = [r for r in rows if engage['t_ms'] <= r['t_ms'] < driving['t_ms']]
    steady = [r for r in standby if r['t_ms'] >= settled['t_ms']+500]
    events = {name: {k: r[k] for k in ['t_ms', 'roll', 'roll_rate', 'meas_drift',
              'arm_tip_frac', 'vel_integral', 'run_curve_shift', 'capture_shift']}
              for name, r in [('capture', engage), ('arm_return', returning),
              ('arm_base_ramp_complete', ramp), ('recovery_settled', settled),
              ('pilot_ready', ready), ('intentional_drive', driving), ('lower_request', lower)]}
    common = lambda r: (r['bl_vel']+r['br_vel'])*.5
    maximum = lambda key: max(r[key] for r in standby)
    minimum = lambda key: min(r[key] for r in standby)
    result = dict(source=str(path.resolve()), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        installed_source='a772ecc833c6100f76cdb67173bdaf4f4d99cd78',
        rows=len(rows), duration_s=rows[-1]['t_ms']/1000,
        end_reason=next(x.split('=', 1)[1] for x in lines if x.startswith('# end_reason=')),
        operator_report='Fast tip worked with substantial catch travel; later manually settled oscillations before lowering.',
        events=events, fast_metrics=dict(
            tip_to_capture_s=(engage['t_ms']-rows[0]['t_ms'])/1000,
            capture_to_ramp_complete_s=(ramp['t_ms']-engage['t_ms'])/1000,
            capture_to_recovery_settled_s=(settled['t_ms']-engage['t_ms'])/1000,
            maximum_forward_wheel_position_rad=maximum('meas_drift'),
            minimum_reverse_wheel_position_rad=minimum('meas_drift'),
            maximum_forward_common_speed_rad_s=max(common(r) for r in standby),
            minimum_reverse_common_speed_rad_s=min(common(r) for r in standby),
            maximum_recovery_integral_deg=maximum('vel_integral'),
            steady_window_ms=[steady[0]['t_ms'], steady[-1]['t_ms']],
            steady_wheel_position_range_rad=max(r['meas_drift'] for r in steady)-min(r['meas_drift'] for r in steady),
            steady_tilt_range_deg=[min(r['roll'] for r in steady), max(r['roll'] for r in steady)]),
        decision='Preserve successful fast v2 production policy for the lowering/drive-handoff correction.',
        limits=['Wheel radians are not ground displacement; effective tire radius and slip are unmeasured.',
                'One operator-confirmed successful stand-up does not establish a success rate.',
                'Later intentional driving and manual settling are excluded from fast-catch metrics.'])
    output.mkdir(parents=True, exist_ok=True)
    (output/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    first = [r for r in rows if r['t_ms'] < driving['t_ms']]
    time = [r['t_ms']/1000 for r in first]
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(time, [r['roll'] for r in first], label='Body angle')
    axes[0].plot(time, [r['setpoint'] if r['state'] == 2 else float('nan') for r in first], label='Balance target')
    axes[0].set_ylabel('Degrees')
    axes[1].plot(time, [common(r) for r in first], label='Measured common rear-wheel speed')
    axes[1].set_ylabel('rad/s')
    axes[2].plot(time, [r['meas_drift'] if r['state'] == 2 else float('nan') for r in first], label='Wheel position from capture')
    axes[2].set_ylabel('Wheel radians')
    axes[3].plot(time, [r['vel_integral'] for r in first], label='Recovery / equilibrium integral')
    axes[3].plot(time, [r['capture_shift'] for r in first], label='Temporary supported-capture shift')
    axes[3].set_ylabel('Degrees')
    for ax in axes:
        for event, color in [(engage, '#289368'), (ramp, '#bf8d25'), (settled, '#6c67a2')]:
            ax.axvline(event['t_ms']/1000, color=color, ls='--', alpha=.7)
        ax.axhline(0, color='#777', lw=.6)
        ax.legend(fontsize=8)
        ax.grid(alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_title('Fast v2: successful lift and arm return, with remaining recoil travel\n'
        f"Capture {engage['t_ms']/1000:.3f} s · arm/base ramp complete {ramp['t_ms']/1000:.3f} s · "
        f"recovery settled {settled['t_ms']/1000:.3f} s")
    axes[-1].set_xlabel('Seconds after run logging starts; ends before intentional driving')
    fig.tight_layout()
    fig.savefig(output/'fast-success.png', dpi=150)
    plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    analyze(args.csv, args.output)
