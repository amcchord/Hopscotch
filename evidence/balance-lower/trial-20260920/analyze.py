#!/usr/bin/env python3
"""Reproduce the failed physical CH11 trial analysis; does not simulate a fix.

Replays measured inputs through the production C++ lowering policy. Replay
tests state/command decisions, not the counterfactual motion of another policy.
The arm Forward references are inferred from the settled pre-trigger targets;
the center magnitudes are historical calibration values, not exported by this
log. Neither uncertainty affects the observed absence of resisted contact.
"""
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
LOG = ROOT / 'telemetry_logs/bal_20260920_lowering_trial_wifi.csv'
PHASES = ['idle', 'stopping', 'reaching', 'descending', 'ground_hold',
          'retracting', 'canceling', 'complete', 'fault', 'loading']


def phase(row):
    return (int(row['pilot_flags']) >> 8) & 15


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extent(rows, key):
    return [min(r[key] for r in rows), max(r[key] for r in rows)]


def analyze():
    lines = LOG.read_text().splitlines()
    metadata = dict(line[2:].split('=', 1) for line in lines
                    if line.startswith('# ') and '=' in line)
    rows = [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader(line for line in lines if not line.startswith('#'))]
    lowering = [r for r in rows if phase(r)]
    reaching = [r for r in lowering if phase(r) == 2]
    trigger = lowering[0]
    forward = [trigger['arm_l_tgt'], trigger['arm_r_tgt']]
    transitions = []
    previous = 0
    for row in lowering:
        if phase(row) != previous:
            transitions.append({
                'phase': PHASES[phase(row)], 't_s': row['t_ms']/1000,
                'since_ch11_s': (row['t_ms']-trigger['t_ms'])/1000,
                **{k: row[k] for k in ('roll', 'roll_rate', 'setpoint', 'base_sp',
                                      'sp_offset', 'bl_vel', 'br_vel', 'motor_vel')}})
            previous = phase(row)
    summary = {
        'log': str(LOG.relative_to(ROOT)),
        'csv_sha256': sha(LOG), 'wire_sha256': sha(LOG.with_suffix('.wire')),
        'samples': len(rows), 'duration_s': int(metadata['run_duration_ms'])/1000,
        'build': metadata['build_date']+' '+metadata['build_time'],
        'features': int(metadata['telemetry_features']), 'end_reason': metadata['end_reason'],
        'operator': {'ground_drive': 'worked really well', 'standing_drive': 'worked really well',
                     'lowering': 'tipped backward on its own; no catch/push reported'},
        'transitions': transitions,
        'supported_lowering_samples': sum(r['state'] == 4 for r in rows),
        'loading_samples': sum(phase(r) == 9 for r in rows),
        'inferred_forward_rad': forward,
        'reach': {
            **{k: extent(reaching, k) for k in ('roll', 'setpoint', 'base_sp', 'meas_vel')},
            'max_resisted_target_error_left_rad': max(r['arm_l']-r['arm_l_tgt'] for r in reaching),
            'max_resisted_target_error_right_rad': max(r['arm_r_tgt']-r['arm_r'] for r in reaching),
            'max_abs_torque_left_nm': max(abs(r['arm_l_torque']) for r in reaching),
            'max_abs_torque_right_nm': max(abs(r['arm_r_torque']) for r in reaching),
            'max_extra_lean_beyond_base_and_pi_deg': max(abs(r['setpoint']-r['base_sp']-r['sp_offset']) for r in reaching),
        },
        'lowering_timing_max': {k: max(r[k] for r in lowering) for k in (
            'sample_dt_ms', 'inner_dt_max_us', 'update_age_ms', 'imu_age_ms',
            'feedback_age_l_ms', 'feedback_age_r_ms')},
        'pre_lowering_pilot_drive_samples': sum(bool(int(r['pilot_flags']) & 16)
                                              for r in rows if not phase(r)),
        'lowering_pilot_inputs': {k: extent(lowering, k) for k in ('pilot_forward', 'pilot_steering')},
    }
    return rows, lowering, summary


def replay(rows, lowering, summary):
    build = ROOT/'output/lowering-trial-replay'
    build.mkdir(parents=True, exist_ok=True)
    binary = build/'policy.so'
    # Pin the deployed policy so this historical diagnosis stays reproducible
    # when the next candidate changes the live source.
    deployed_header = subprocess.check_output(
        ['git', 'show', '43b1967:src/balance_lower.h'], cwd=ROOT)
    (build/'balance_lower.h').write_bytes(deployed_header)
    subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-fPIC', '-I'+str(build),
                    str(ROOT/'scripts/lowering_bridge.cpp'), '-o', str(binary)], check=True)
    lib = C.CDLL(str(binary))
    floats = C.POINTER(C.c_float)
    lib.lower_new.restype = C.c_void_p
    lib.lower_delete.argtypes = [C.c_void_p]
    lib.lower_request.argtypes = [C.c_void_p, C.c_uint32, floats, C.c_bool, C.c_float, C.c_float]
    lib.lower_request.restype = C.c_bool
    lib.lower_step.argtypes = [C.c_void_p, C.c_uint32, C.c_float, floats, C.c_bool, floats]
    lib.lower_reason.argtypes = [C.c_void_p]
    lib.lower_reason.restype = C.c_char_p
    forward = summary['inferred_forward_rad']
    start = rows.index(lowering[0])
    handle = lib.lower_new()
    output = (C.c_float*6)()
    transitions, mismatches = [], []
    previous = 0
    try:
        for index in range(start, len(rows)):
            r = rows[index]
            # The helper runs before the outer loop publishes this row's new
            # setpoint; use the preceding row's effective setpoint.
            inputs = (C.c_float*11)(r['roll'], r['roll_rate'], rows[index-1]['setpoint']-r['roll'],
                r['bl_vel'], r['br_vel'], r['arm_l']-forward[0], r['arm_r']-forward[1],
                r['arm_l_vel'], r['arm_r_vel'], r['arm_l_torque'], r['arm_r_torque'])
            now = int(r['t_ms'])
            if index == start:
                assert lib.lower_request(handle, now, inputs, True, 1.768, -1.767)
            lib.lower_step(handle, now, r['sample_dt_ms']/1000, inputs, True, output)
            current = int(output[2])
            if current != previous:
                transitions.append({'phase': PHASES[current], 't_s': now/1000,
                                    'reason': lib.lower_reason(handle).decode()})
                previous = current
            if current != phase(r):
                mismatches.append(now)
            assert not output[3], 'Measured run must never qualify supported lowering'
            assert output[5] == 0, 'Measured run never reaches its forward-lean phase'
        summary['replay'] = {
            'scope': 'Recorded inputs through deployed 43b1967 helper, not a physical simulation',
            'calibration': 'Forward inferred from settled target; historical centers +1.768/-1.767 rad',
            'transitions': transitions, 'phase_mismatch_t_ms': mismatches,
        }
        assert [t['phase'] for t in transitions] == ['stopping', 'reaching', 'canceling', 'fault']
        assert transitions[-2]['reason'] == 'lower_reach_disturbed'
        assert transitions[-1]['reason'] == 'lower_motion_limit'
        # Export rounds values and the fast/outer tasks sample asynchronously.
        assert len(mismatches) <= 1, mismatches
    finally:
        lib.lower_delete(handle)


def plot(rows, lowering, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    start = lowering[0]['t_ms']
    selected = [r for r in rows if r['t_ms'] >= start-1000]
    t = [(r['t_ms']-start)/1000 for r in selected]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    fig.suptitle('Physical CH11 trial: backward lean before supported lowering\n'
                 'September 20, 2026 · recorded telemetry, not simulation', fontsize=14)
    for key, label, color in [('roll','Measured tilt','#1b4965'), ('setpoint','Effective target','#cc5500'),
                              ('base_sp','Arm-scheduled base','#888888')]:
        axes[0].plot(t, [r[key] for r in selected], label=label, color=color)
    axes[0].axhline(90, color='#999999', lw=.6)
    axes[0].set_ylabel('Tilt (deg)\n↑ backward')
    axes[0].legend(loc='upper left', ncol=3, fontsize=9)
    for key, label in [('meas_vel','Measured wheel mean'), ('motor_vel','Wheel command')]:
        axes[1].plot(t, [r[key] for r in selected], label=label)
    axes[1].axhline(-4, color='#b00020', ls=':', label='Reach cancel limit')
    axes[1].set_ylabel('Wheel rad/s')
    axes[1].legend(loc='lower left', ncol=3, fontsize=9)
    fwd = summary['inferred_forward_rad']
    for key, offset, sign, label in [('arm_l',fwd[0],-1,'Left measured'),
                                   ('arm_r',fwd[1],1,'Right measured'),
                                   ('arm_l_tgt',fwd[0],-1,'Left target'),
                                   ('arm_r_tgt',fwd[1],1,'Right target')]:
        axes[2].plot(t, [(r[key]-offset)*sign for r in selected],
                     label=label, ls='--' if key.endswith('tgt') else '-')
    axes[2].set_ylabel('Forward reach (rad)')
    axes[2].legend(loc='upper left', ncol=4, fontsize=9)
    axes[3].plot(t, [r['arm_l']-r['arm_l_tgt'] for r in selected], label='Left resisted target error')
    axes[3].plot(t, [r['arm_r_tgt']-r['arm_r'] for r in selected], label='Right resisted target error')
    axes[3].axhline(.06, color='#b00020', ls=':', label='Contact requires ≥0.06 rad')
    axes[3].set_ylabel('Target resistance (rad)')
    axes[3].legend(loc='lower left', ncol=3, fontsize=9)
    axes[3].set_xlabel('Seconds after CH11 request (request at run t = 37.020 s)')
    for ax in axes:
        ax.grid(alpha=.2)
        for event in summary['transitions']:
            ax.axvline(event['since_ch11_s'], color='#666666', ls=':', lw=.8)
    for event in summary['transitions']:
        axes[0].annotate(event['phase'], (event['since_ch11_s'], 107), rotation=90,
                         xytext=(3, 0), textcoords='offset points', fontsize=8, va='top')
    fig.savefig(OUT/'lowering-trial.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    rows, lowering, summary = analyze()
    replay(rows, lowering, summary)
    plot(rows, lowering, summary)
    (OUT/'analysis.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
