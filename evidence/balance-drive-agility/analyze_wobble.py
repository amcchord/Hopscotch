"""Physical v3 driving analysis; terminal contact interval is not used for tuning."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
p=ROOT/'telemetry_logs/bal_20260919_185559_drive-agility-wobble-wall.csv'
lines=p.read_text().splitlines();meta=dict(l[2:].split('=',1) for l in lines if l.startswith('# ') and '=' in l)
r=list(csv.DictReader(l for l in lines if not l.startswith('#')))
a={k:np.array([float(row[k]) for row in r]) for k in r[0]}
b=a['state']==2;t=(a['t_ms']-a['t_ms'][b][0])/1000
f=a['pilot_flags'].astype(int);d=a['diag_flags'].astype(int)
def first(mask):
    ids=np.flatnonzero(mask)
    return float(t[ids[0]]) if len(ids) else None
def transitions(mask):
    ids=np.flatnonzero(mask[1:]!=mask[:-1])+1
    return [[float(t[i]),bool(mask[i])] for i in ids]
early=b&(t>=4)&(t<31.5)
metrics=dict(file=str(p.relative_to(ROOT)),samples=len(r),duration_s=float((a['t_ms'][-1]-a['t_ms'][0])/1000),
    balance_s=float(t[-1]),binary_checksum=meta['checksum'],transport_checksum=meta['transport_fnv1a'],
    end_reason=meta['end_reason'],user_report='stand-up worked; front-to-back rocking while driving; wall contact at end, exact time unmarked',
    first_ready_s=first((f&1)>0),first_driving_s=first((f&2)>0),
    ready_transitions=transitions((f&1)>0),controller_transitions=transitions((f&16)>0),
    inner_max_us=int(max(a['inner_dt_max_us'][b])),imu_max_ms=int(max(a['imu_age_ms'][b])),
    imu_fault_rows=int(np.sum((d&0x400)>0)),can_tx_fault_rows=int(np.sum((d&0x800)>0)),
    post_engage_missing_input_rows=int(np.sum(b&(t>.05)&((f&8)==0))),
    first_positive_request_s=first((t>4)&(a['pilot_forward']>.001)),
    first_actual_forward_half_rad_s=first((t>6.74)&(a['filtered_vel']>.5)),
    planned_arm_peak=float(max(abs(a['pilot_arm'][early]))),
    actual_total_assist_min=float(min(a['arm_assist_frac'][early])),actual_total_assist_max=float(max(a['arm_assist_frac'][early])),
    early_peak_filtered_speed=float(max(abs(a['filtered_vel'][early]))),
    early_peak_body_rate=float(max(abs(a['roll_rate'][early]))),
    early_peak_angle_error=float(max(abs(a['angle_err'][early]))),
    first_recovery_arm_s=first(b&np.isin(a['arm_stage'],[1,2])),
    first_emergency_arm_s=first((d&64)>0),windows={})
for label,lo,hi in [('quiet_before_drive',4.2,6.5),('first_stop',9.2,13.5),('second_stop',17.5,20.5),('late_centered_wobble',20.5,23.0)]:
    q=b&(t>=lo)&(t<hi);v=a['filtered_vel'][q];rate=a['roll_rate'][q]
    # A descriptive frequency for sufficiently long windows, not plant fitting.
    signal=rate-np.mean(rate);freq=np.fft.rfftfreq(len(signal),.02);power=abs(np.fft.rfft(signal))**2
    band=(freq>.4)&(freq<6);peak=float(freq[band][np.argmax(power[band])])
    metrics['windows'][label]=dict(start_s=lo,end_s=hi,speed_min=float(min(v)),speed_max=float(max(v)),
        speed_rms=float(np.sqrt(np.mean(v*v))),tilt_span_deg=float(np.ptp(a['roll'][q])),
        max_abs_requested_velocity=float(max(abs(a['target_vel'][q]))),
        max_abs_steering=float(max(abs(a['pilot_steering'][q]))),
        body_rate_dominant_hz=peak,max_abs_rate=float(max(abs(rate))))
fig,axes=plt.subplots(5,1,figsize=(12,12),sharex=True,layout='constrained')
axes[0].plot(t,a['target_vel'],label='requested speed',c='#2563eb');axes[0].plot(t,a['filtered_vel'],label='measured speed',c='#ea580c')
axes[1].plot(t,a['angle_err'],label='balance angle error',c='#9a3412')
axes[2].plot(t,a['pilot_arm'],label='small planned arm fraction',c='#2563eb');axes[2].plot(t,a['arm_assist_frac'],label='total arm fraction incl. recovery',c='#ea580c')
axes[3].plot(t,(a['bl_vel']-a['br_vel'])/2,label='actual differential',c='#ea580c');axes[3].plot(t,a['pilot_turn'],label='requested turn',c='#2563eb')
axes[4].plot(t,(f&1)>0,label='accepting stick inputs',c='#2563eb');axes[4].plot(t,(f&16)>0,label='acceleration control / braking',c='#ea580c',ls='--')
for ax,y in zip(axes,('wheel rad/s','degrees','center fraction','turn rad/s','active')):
    ax.set_ylabel(y);ax.legend(loc='upper left',ncol=2);ax.grid(alpha=.2);ax.set_xlim(4,38.8)
    ax.axvspan(32,38.8,color='#64748b',alpha=.12)
axes[1].set_ylim(-9,9)
axes[-1].set_xlabel('Seconds after balance engagement; shaded terminal bout excluded from tuning conclusions')
fig.suptitle('Actual v3 drive: faster travel, repeated fore/aft oscillation and large recovery-arm swings\nWall-contact time is unmarked; terminal tilt exceeds plotted scale')
fig.savefig(OUT/'wobble-overview.png',dpi=150)
(OUT/'wobble-metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps(metrics,indent=2))
