#!/usr/bin/env python3
"""Reproduce the v8 catch stop and bound v9's first command divergence."""
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'scripts'))
from simulate_lowering import Policy


def read(name):
    path = ROOT/'telemetry_logs'/name
    lines = path.read_text().splitlines()
    meta = dict(line[2:].split('=', 1) for line in lines if line.startswith('# ') and '=' in line)
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(line for line in lines if not line.startswith('#'))]
    return path, meta, rows


def phase(row):
    return (int(row['pilot_flags']) >> 8) & 15


def replay(policy, rows):
    forward = [rows[0]['arm_l_tgt'], rows[0]['arm_r_tgt']]
    active = [r for r in rows if phase(r)]
    handle = policy.lib.lower_new(); output = (C.c_float*7)()
    commands = []; events = []; previous = None
    try:
        for n, r in enumerate(active):
            values = (C.c_float*11)(r['roll'], r['roll_rate'], r['angle_err'], r['bl_vel'], r['br_vel'],
                r['arm_l']-forward[0], r['arm_r']-forward[1], r['arm_l_vel'], r['arm_r_vel'], r['arm_l_torque'], r['arm_r_torque'])
            if n == 0:
                assert policy.lib.lower_request(handle, int(r['t_ms']), values, True, 1.768, -1.767)
            policy.lib.lower_step(handle, int(r['t_ms']), r['sample_dt_ms']/1000, values, True, output)
            commands.append([r['t_ms'], *output, policy.lib.lower_arm_speed(handle)])
            if int(output[2]) != previous:
                events.append(dict(t_ms=r['t_ms'], phase=int(output[2]), tilt=r['roll'], rate=r['roll_rate']))
                previous = int(output[2])
            if int(output[2]) in (7, 8):
                break
        return dict(events=events, reason=policy.lib.lower_reason(handle).decode()), commands
    finally:
        policy.lib.lower_delete(handle)


def main():
    path, meta, rows = read('bal_20260920_forward_catch_v8_stop_wifi.csv')
    folder = ROOT/'output/lowering-v9-recorded-baseline'; folder.mkdir(parents=True, exist_ok=True)
    (folder/'balance_lower.h').write_bytes(subprocess.check_output(['git', 'show', '47eb19a:src/balance_lower.h'], cwd=ROOT))
    old, new = Policy(folder), Policy()
    before, commands_before = replay(old, rows)
    after, commands_after = replay(new, rows)
    assert before['reason'] == meta['end_reason'] == 'lower_wrong_direction'
    divergence = next(a[0] for a, b in zip(commands_before, commands_after) if a != b)
    assert any(e['phase'] == 3 for e in after['events'])
    _, _, successful = read('bal_20260920_forward_catch_v7_success_wifi.csv')
    success_old, commands_old = replay(old, successful)
    success_new, commands_new = replay(new, successful)
    assert commands_old == commands_new and success_old == success_new
    active = [r for r in rows if phase(r)]
    impact = next(r for r in active if phase(r) == 10 and min(abs(r['arm_l_torque']), abs(r['arm_r_torque'])) >= .4)
    result = dict(source=str(path.relative_to(ROOT)), installed_source='47eb19a2ce57f51eeea368c4c6469edd1cc51a96',
        samples=len(rows), duration_s=int(meta['run_duration_ms'])/1000, end_reason=meta['end_reason'],
        csv_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        wire_sha256=hashlib.sha256(path.with_suffix('.wire').read_bytes()).hexdigest(),
        faster_return_reached=any(phase(r) == 3 for r in rows),
        first_two_arm_impact={k: impact[k] for k in ['t_ms', 'roll', 'roll_rate', 'arm_l_torque', 'arm_r_torque']},
        installed_replay=before, candidate_fixed_sensor_replay=after, first_command_divergence_ms=divergence,
        prior_success_v8_v9_command_replays_identical=True,
        candidate_header_sha256=hashlib.sha256((ROOT/'src/balance_lower.h').read_bytes()).hexdigest(),
        limits=['Fixed recorded inputs reproduce the installed fault and locate command divergence.',
                'After divergence the candidate would change physics. Its later lower_support_lost on old inputs is not a physical prediction.',
                'The earlier successful trace produces identical v8/v9 command replay, not a prediction of duration.'])
    (OUT/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    catch = [r for r in rows if r['t_ms'] >= impact['t_ms']-80]
    t = [(r['t_ms']-impact['t_ms'])/1000 for r in catch]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, [r['roll'] for r in catch], label='Measured body tilt')
    axes[0].set_ylabel('Degrees')
    axes[1].plot(t, [r['roll_rate'] for r in catch], label='Measured body rate')
    axes[1].axhline(12, ls=':', color='#9b6527', label='Ordinary confirmation maximum')
    axes[1].axhline(20, ls=':', color='#2a9175', label='V9 bounded impact maximum')
    axes[1].set_ylabel('Degrees/s')
    for side in ('l', 'r'):
        axes[2].plot(t, [abs(r['arm_'+side+'_torque']) for r in catch], label=side.upper()+' |torque|')
    axes[2].axhline(.2, color='#888', ls=':', label='Recent-support threshold')
    axes[2].set_ylabel('Nm (telemetry)')
    for ax in axes:
        ax.axvline(0, color='#888', ls='--', alpha=.6)
        ax.axvline((divergence-impact['t_ms'])/1000, color='#2a9175', ls='--', label='Candidate divergence')
        ax.grid(alpha=.2); ax.legend(fontsize=8); ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_title('V8 stopped during catch, before faster supported return\n'
        'V9 confirms a small loaded rebound; subsequent recorded motion is no longer predictive')
    axes[-1].set_xlabel('Seconds since first two-arm impact')
    fig.tight_layout(); fig.savefig(OUT/'catch-stop.png', dpi=150); plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
