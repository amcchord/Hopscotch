"""Reproduce post-feedback-fix trial comparison; do not infer hand timing."""
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from analyze_balance_logs import load_rows, parse_config
FILES = [
    ('Feedback correction: hand-assisted', 'bal_20260914_222023_feedback-fix-first.csv'),
    ('Capture correction: hand-assisted', 'bal_20260914_223915_capture-trim-assisted.csv'),
]
fig, ax = plt.subplots(4, 2, figsize=(12, 10), sharex=True, constrained_layout=True)
metrics = []
for col, (title, filename) in enumerate(FILES):
    path = ROOT / 'telemetry_logs' / filename
    cfg = parse_config(path)
    rows = [r for r in load_rows(path) if r['state'] == '2']
    values = lambda key: np.array([float(r[key]) for r in rows])
    t = (values('t_ms') - float(rows[0]['t_ms'])) / 1000
    scale = float(cfg.get('wheel_feedback_velocity_range_rad_s', '33'))
    velocity_factor = 50 / scale
    velocity = values('meas_vel') * velocity_factor
    age = np.maximum(values('feedback_age_l_ms'), values('feedback_age_r_ms'))
    first4 = t < 4
    calm = ((t >= 4) & (values('arm_tip_frac') < .02)
            & (np.abs(values('arm_assist_frac')) < .02)
            & (np.abs(values('roll_rate')) < 2) & (np.abs(velocity) < .5)
            & (np.abs(values('motor_vel')) < .5))
    result = dict(file=filename, samples=len(load_rows(path)), end_reason=cfg['end_reason'],
                  file_checksum=cfg['checksum'], usb_checksum=cfg['transport_fnv1a'],
                  profile_scope=cfg.get('profile_scope'), engage_s=float(rows[0]['t_ms'])/1000,
                  engage_roll_deg=values('roll')[0], balance_span_s=t[-1],
                  arm_return_s=next(float(ti) for ti,r in zip(t,rows) if int(r['flags']) & 0x20),
                  ramp_complete_s=next(float(ti) for ti,r in zip(t,rows) if int(r['flags']) & 0x40),
                  first4_peak_command_rad_s=float(np.max(np.abs(values('motor_vel')[first4]))),
                  first4_peak_average_wheel_velocity_rad_s=float(np.max(np.abs(velocity[first4]))),
                  first4_peak_wheel_displacement_rad=float(np.max(np.abs(values('meas_drift')[first4]))),
                  calm_forward_median_roll_deg=float(np.median(values('roll')[calm])),
                  calm_forward_minus_engage_deg=float(np.median(values('roll')[calm])-values('roll')[0]),
                  calm_row_count=int(calm.sum()),
                  after_first50ms_feedback_max_ms=float(np.max(age[t >= .05])),
                  inner_max_us=float(np.max(values('inner_dt_max_us'))),
                  imu_max_age_ms=float(np.max(values('imu_age_ms'))),
                  operator='Hand intervention; exact contact time unmarked')
    metrics.append(result)
    ax[0,col].plot(t, values('roll'), label='Tilt')
    ax[0,col].plot(t, values('setpoint'), label='Target')
    ax[0,col].plot(t[t>.02], values('base_sp')[t>.02], label='Base target', ls='--')
    ax[1,col].plot(t, values('motor_vel'), label='Wheel command')
    ax[1,col].plot(t, velocity, label='Wheel feedback', alpha=.7)
    ax[2,col].plot(t, values('meas_drift'), label='Wheel displacement')
    ax[3,col].plot(t, values('arm_tip_frac'), label='Measured arm tip fraction')
    for axis in ax[:,col]:
        axis.axvline(result['arm_return_s'], color='#888888', ls=':')
        axis.axvline(result['ramp_complete_s'], color='#888888', ls='--')
        axis.grid(alpha=.18)
        axis.legend(fontsize=8)
    ax[0,col].set_title(title)
    ax[3,col].set_xlabel('Seconds after BALANCE begins')
for axis,label in zip(ax[:,0], ['Degrees', 'Physical rad/s', 'Wheel radians from origin', 'Arm fraction']):
    axis.set_ylabel(label)
fig.suptitle('Initial roll-away remains; both attempts recovered with hand assistance\nDotted: arm return begins. Dashed: ramp completes. Contact timing unknown.', fontsize=13)
fig.savefig(ROOT / 'evidence/balance-handoff-followup/capture-trial-comparison.png', dpi=160)
(ROOT / 'evidence/balance-handoff-followup/capture-trial-metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
print(json.dumps(metrics, indent=2))
