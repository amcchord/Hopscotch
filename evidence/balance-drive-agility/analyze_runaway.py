"""Compare the failed v3 stand-up with the preceding successful v2 capture."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
names={'prior v2':'bal_20260919_181555_drive-response-v2-delayed.csv',
       'failed v3':'bal_20260919_185202_drive-agility-standup-runaway.csv'}
fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
results={}
for label,name in names.items():
    path=ROOT/'telemetry_logs'/name
    lines=path.read_text().splitlines()
    meta=dict(l[2:].split('=',1) for l in lines if l.startswith('# ') and '=' in l)
    rows=list(csv.DictReader(l for l in lines if not l.startswith('#')))
    a={k:np.array([float(r[k]) if r[k] else np.nan for r in rows]) for k in rows[0]}
    b=a['state']==2;t=(a['t_ms']-a['t_ms'][b][0])/1000
    flags=a['flags'].astype(int);diag=a['diag_flags'].astype(int);pilot=a['pilot_flags'].astype(int)
    def first(mask):
        where=np.flatnonzero(mask)
        return float(t[where[0]]) if len(where) else None
    q=b&(t<=2.38)
    result=dict(file=str(path.relative_to(ROOT)),samples=len(rows),end_reason=meta['end_reason'],
        binary_checksum=meta['checksum'],transport_checksum=meta['transport_fnv1a'],
        initial_tilt=float(a['roll'][0]),engage_tilt=float(a['roll'][b][0]),
        initial_accel_angle=float(a['accel_angle'][0]),
        return_start_s=first((flags&0x20)>0),recovery_trigger_s=first((diag&0x1000)>0),
        crisis_pause_s=first((diag&0x20)>0),ramp_complete_s=first((flags&0x40)>0),
        startup_2p38s_peak_filtered_speed=float(max(abs(a['filtered_vel'][q]))),
        startup_2p38s_peak_wheel_travel=float(max(abs(a['meas_drift'][q]))),
        pilot_ready_rows=int(np.sum((pilot&1)>0)),pilot_moving_rows=int(np.sum((pilot&2)>0)),
        min_bus_voltage=float(min(a['bus_voltage'])),
        inner_max_us=int(max(a['inner_dt_max_us'][b])),imu_max_ms=int(max(a['imu_age_ms'][b])),
        imu_fault_rows=int(np.sum((diag&0x400)>0)),can_tx_fault_rows=int(np.sum((diag&0x800)>0)))
    if label=='failed v3':
        result.update(acceleration_active_rows=int(np.sum((pilot&16)>0)),
            planned_arm_peak=float(max(abs(a['pilot_arm']))),
            max_recorded_stick=float(max(max(abs(a['pilot_forward'])),max(abs(a['pilot_steering'])))),
            pd_residual_max=float(max(abs(a['motor_vel_raw'][b]-(2*a['angle_err'][b]-.08*a['roll_rate'][b])))),
            final_arm_tip_fraction=float(a['arm_tip_frac'][-1]),
            buffer_operation='561 checksummed schema4 samples saved successfully')
    results[label]=result
    color='#ea580c' if label=='failed v3' else '#2563eb'
    axes[0].plot(t[q],a['roll'][q],color=color,label=label+' tilt')
    axes[0].plot(t[q],a['setpoint'][q],color=color,ls='--',alpha=.65,label=label+' target')
    axes[1].plot(t[q],a['filtered_vel'][q],color=color,label=label)
    axes[2].plot(t[q],a['arm_tip_frac'][q],color=color,label=label)
    axes[3].plot(t[q],a['vel_integral'][q],color=color,label=label)
for ax,y in zip(axes,('body degrees','wheel rad/s','arm tip fraction','learned correction °')):
    ax.set_ylabel(y);ax.legend(loc='best',ncol=2);ax.grid(alpha=.2);ax.set_xlim(0,2.4)
axes[-1].set_xlabel('Seconds after balance engagement')
fig.suptitle('Failed stand-up: new drive controller and planned arms never activated\nActual runs; contact and sensor-mount changes are not instrumented')
fig.savefig(OUT/'runaway-comparison.png',dpi=150)
(OUT/'runaway-metrics.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
