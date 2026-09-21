#!/usr/bin/env python3
"""Archive-derived timing/tracking evidence for a completed CH11 lowering run."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics


def phase(row):
    return (int(row['pilot_flags']) >> 8) & 15


def analyze(path, output, installed_source):
    lines = path.read_text().splitlines()
    meta = dict(line[2:].split('=', 1) for line in lines if line.startswith('# ') and '=' in line)
    rows = [{key: float(value) for key, value in row.items()} for row in
            csv.DictReader(line for line in lines if not line.startswith('#'))]
    assert meta['end_reason'] == 'lower_complete' and phase(rows[-1]) == 7
    active = [row for row in rows if phase(row)]
    # The tip-up starts at the calibrated Forward reference, before arm motion.
    forward = [rows[0]['arm_l_tgt'], rows[0]['arm_r_tgt']]
    events = []
    previous = None
    for row in active:
        if phase(row) != previous:
            events.append(dict(phase=phase(row), **{key: row[key] for key in
                ['t_ms', 'roll', 'roll_rate', 'arm_l', 'arm_r', 'arm_l_tgt', 'arm_r_tgt', 'bl_vel', 'br_vel']}))
            previous = phase(row)
    first = {p: next(row for row in active if phase(row) == p) for p in (1, 2, 3, 4, 5, 7, 9, 10)}
    summary = {}
    for p in (1, 2, 9, 10, 3, 4, 5):
        selected = [row for row in active if phase(row) == p]
        summary[str(p)] = dict(samples=len(selected),
            sampled_duration_s=sum(row['sample_dt_ms'] for row in selected)/1000,
            tilt_range_deg=[min(row['roll'] for row in selected), max(row['roll'] for row in selected)],
            rate_range_dps=[min(row['roll_rate'] for row in selected), max(row['roll_rate'] for row in selected)],
            median_rate_dps=statistics.median(row['roll_rate'] for row in selected),
            max_target_error_rad=[max(abs(row['arm_'+side+'_tgt']-row['arm_'+side]) for row in selected) for side in ('l', 'r')],
            max_abs_torque_nm=[max(abs(row['arm_'+side+'_torque']) for row in selected) for side in ('l', 'r')],
            outside_normal_rate_bounds_samples=sum(row['roll_rate'] < -12 or row['roll_rate'] > 4 for row in selected))
    timing = dict(request_to_prepare_s=(first[2]['t_ms']-first[1]['t_ms'])/1000,
        prepare_to_commit_s=(first[9]['t_ms']-first[2]['t_ms'])/1000,
        commit_to_support_s=(first[3]['t_ms']-first[9]['t_ms'])/1000,
        support_to_first_flat_s=(first[4]['t_ms']-first[3]['t_ms'])/1000,
        first_flat_to_retract_s=(first[5]['t_ms']-first[4]['t_ms'])/1000,
        final_retract_s=(first[7]['t_ms']-first[5]['t_ms'])/1000,
        total_lower_s=(first[7]['t_ms']-first[1]['t_ms'])/1000)
    fast = bool(int(rows[0]['pilot_flags']) & 128)
    fast_events = {}
    if fast:
        for name, predicate in [('capture', lambda r: r['state'] == 2),
                ('arm_return', lambda r: r['state'] == 2 and int(r['flags']) & 32),
                ('ramp_complete', lambda r: r['state'] == 2 and int(r['flags']) & 64),
                ('recovery_settled', lambda r: r['state'] == 2 and int(r['diag_flags']) & 32768)]:
            row = next((r for r in rows if predicate(r)), None)
            # A clean stand-up can finish without triggering early recovery.
            fast_events[name] = ({key: row[key] for key in ('t_ms', 'roll', 'roll_rate', 'capture_shift', 'run_curve_shift')}
                                 if row is not None else None)
    final = rows[-1]
    result = dict(source=str(path.resolve()), installed_source=installed_source,
        csv_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        wire_sha256=hashlib.sha256(path.with_suffix('.wire').read_bytes()).hexdigest(),
        samples=len(rows), schema=int(meta['telemetry_schema']), end_reason=meta['end_reason'],
        duration_s=int(meta['run_duration_ms'])/1000, forward_rad=forward, events=events,
        timing=timing, phase_metrics=summary, fast_selected=fast, fast_events=fast_events,
        fast_lower_selected=all(bool(int(row['pilot_flags']) & 4096) for row in active),
        final=dict(tilt_deg=final['roll'], rate_dps=final['roll_rate'],
            measured_forward_error_rad=[final['arm_l']-forward[0], final['arm_r']-forward[1]],
            wheel_rad_s=[final['bl_vel'], final['br_vel']]),
        limits=['One successful physical trial does not establish a reliability rate.',
                'Measured torque and target tracking do not establish mechanical load ratings.',
                'A faster return changes body/contact dynamics; recorded-input replay cannot predict it.'])
    output.mkdir(parents=True, exist_ok=True)
    (output/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = [(r['t_ms']-first[1]['t_ms'])/1000 for r in active]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(t, [r['roll'] for r in active], label='Body tilt')
    axes[0].set_ylabel('Degrees')
    axes[1].plot(t, [r['roll_rate'] for r in active], label='Measured body rate')
    axes[1].axhline(-12, color='#b66b18', ls=':', label='Normal return limit (fast limit varies)')
    axes[1].set_ylabel('Degrees/s')
    for side, origin, sign, color in [('l', forward[0], 1, '#287ba6'), ('r', forward[1], -1, '#ad5b85')]:
        axes[2].plot(t, [sign*(r['arm_'+side]-origin) for r in active], color=color, label=side.upper()+' arm offset')
        axes[2].plot(t, [sign*(r['arm_'+side+'_tgt']-origin) for r in active], color=color, ls=':', alpha=.7)
        axes[3].plot(t, [abs(r['arm_'+side+'_torque']) for r in active], color=color, label=side.upper()+' |torque|')
    axes[2].set_ylabel('Radians from Forward')
    axes[3].set_ylabel('Nm (telemetry)')
    for ax in axes:
        for p in (3, 4, 5):
            ax.axvline((first[p]['t_ms']-first[1]['t_ms'])/1000, color='#777', ls='--', alpha=.6)
        ax.legend(loc='best', fontsize=8); ax.grid(alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_title('Successful CH11 lowering: most time is supported arm return\n'
        f"Total {timing['total_lower_s']:.2f} s · support to flat {timing['support_to_first_flat_s']:.2f} s · "
        f"final retraction {timing['final_retract_s']:.2f} s")
    axes[-1].set_xlabel('Seconds since CH11 lowering request')
    fig.tight_layout(); fig.savefig(output/'lowering-success.png', dpi=150); plt.close(fig)
    print(json.dumps(dict(samples=len(rows), timing=timing, final=result['final'], fast_events=fast_events), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--installed-source', required=True)
    args = parser.parse_args()
    analyze(args.csv, args.output, args.installed_source)
