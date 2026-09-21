#!/usr/bin/env python3
"""Compare archived fast-run wheel tracking, reading raw telemetry in place."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_run(path):
    lines = path.read_text().splitlines()
    rows = [{k: float(v) for k, v in r.items()} for r in
            csv.DictReader(line for line in lines if not line.startswith('#'))]
    assert rows and all(int(r['pilot_flags']) & 128 for r in rows), 'Expected fast stand-up'
    capture = next(r for r in rows if r['state'] == 2)
    returning = next(r for r in rows if r['state'] == 2 and int(r['flags']) & 32)
    # Limit comparison to initial recovery; exclude later operator driving/lowering.
    recovery = [r for r in rows if r['state'] == 2
                and r['t_ms'] <= capture['t_ms'] + 3000
                and not int(r['pilot_flags']) & (2 | 64)]
    keys = ['t_ms', 'roll', 'roll_rate', 'setpoint', 'arm_tip_frac',
            'cmd_left', 'cmd_right', 'bl_vel', 'br_vel', 'yaw_diff',
            'vel_integral', 'capture_shift', 'run_curve_shift']
    selected = lambda r: {k: r[k] for k in keys}
    info = dict(source=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                rows=len(rows), end_reason=next(l.split('=', 1)[1] for l in lines
                                               if l.startswith('# end_reason=')),
                capture=selected(capture), arm_return=selected(returning),
                final_sample=selected(rows[-1]), comparison_window_ms=[recovery[0]['t_ms'], recovery[-1]['t_ms']],
                max_wheel_speed_difference_rad_s=max(abs(r['bl_vel']-r['br_vel']) for r in recovery),
                peak_recovery_integral_deg=max(r['vel_integral'] for r in recovery),
                timing_maxima={k: max(r[k] for r in recovery) for k in
                    ['inner_dt_max_us', 'update_age_ms', 'imu_age_ms',
                     'feedback_age_l_ms', 'feedback_age_r_ms']})
    return rows, recovery, info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('failed', type=Path)
    parser.add_argument('successful', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--installed-source', required=True)
    parser.add_argument('--fit-window', nargs=2, type=float, required=True,
                        help='Explicit failed-run seconds used for measured left-wheel linear fit')
    args = parser.parse_args()
    failed, fail_recovery, fail_info = read_run(args.failed)
    _, success_recovery, success_info = read_run(args.successful)
    fit = [r for r in fail_recovery if args.fit_window[0] <= r['t_ms']/1000 <= args.fit_window[1]]
    assert len(fit) >= 3
    t = np.array([r['t_ms']/1000 for r in fit])
    velocity = np.array([r['bl_vel'] for r in fit])
    coeff = np.polyfit(t, velocity, 1)
    residual = velocity - np.polyval(coeff, t)
    point = min(failed, key=lambda r: abs(r['t_ms'] - 5506))
    result = dict(installed_source=args.installed_source, failed=fail_info, successful_reference=success_info,
        measured_left_wheel_fit=dict(requested_window_s=args.fit_window,
            actual_window_s=[float(t[0]), float(t[-1])], samples=len(fit),
            acceleration_rad_s2=float(coeff[0]),
            r_squared=float(1-np.sum(residual**2)/np.sum((velocity-velocity.mean())**2)),
            rmse_rad_s=float(np.sqrt(np.mean(residual**2)))),
        representative_tracking_sample={k: point[k] for k in
            ['t_ms', 'roll', 'cmd_left', 'cmd_right', 'bl_vel', 'br_vel', 'bl_torque', 'br_torque',
             'feedback_age_l_ms', 'feedback_age_r_ms', 'yaw_diff', 'yaw_corr']},
        limits=['A fitted wheel acceleration is not a motor-register readback.',
                'The log does not measure tire slip, ground contact, obstruction or applied ACC_RAD.',
                'The successful run is a physical reference, not a controlled experiment.',
                'No production change or physical intervention is tested by this comparison.'])
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True, sharey='row')
    for col, (rows, info, label) in enumerate([(fail_recovery, fail_info, 'Latest failed run'),
                                             (success_recovery, success_info, 'Previous successful v9 run')]):
        time = [(r['t_ms']-info['capture']['t_ms'])/1000 for r in rows]
        for row, series in enumerate([
                [('roll', 'Measured body angle'), ('setpoint', 'Balance target')],
                [('bl_vel', 'Measured left'), ('cmd_left', 'Commanded left')],
                [('br_vel', 'Measured right'), ('cmd_right', 'Commanded right')]]):
            ax = axes[row, col]
            for index, (key, name) in enumerate(series):
                ax.plot(time, [r[key] for r in rows], label=name, ls='-' if index == 0 else '--')
            ax.axvline((info['arm_return']['t_ms']-info['capture']['t_ms'])/1000,
                       color='#777777', lw=.8, ls=':', label='Arm return starts')
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
            ax.spines[['top', 'right']].set_visible(False)
        axes[0, col].set_title(label)
        axes[-1, col].set_xlabel('Seconds after upright capture')
    axes[0, 0].set_ylabel('Degrees')
    axes[1, 0].set_ylabel('Left rear speed (rad/s)')
    axes[2, 0].set_ylabel('Right rear speed (rad/s)')
    fig.suptitle('Fast tip-up: rear-left wheel falls behind during the backward catch', fontsize=14)
    fig.tight_layout()
    fig.savefig(args.output/'wheel-tracking.png', dpi=150)
    plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
