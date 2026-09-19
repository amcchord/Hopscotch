"""Measured response-v2 trial, separate from v3 simulations."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
p=ROOT/'telemetry_logs/bal_20260919_181555_drive-response-v2-delayed.csv'
s=p.read_text().splitlines();meta=dict(x[2:].split('=',1) for x in s if x.startswith('# ') and '=' in x)
r=list(csv.DictReader(x for x in s if not x.startswith('#')));a={k:np.array([float(x[k]) for x in r]) for k in r[0]}
b=a['state']==2;t=(a['t_ms']-a['t_ms'][b][0])/1000;f=a['pilot_flags'].astype(int);d=a['diag_flags'].astype(int)
def first(mask):
 i=np.flatnonzero(mask);return float(t[i[0]]) if len(i) else None
metrics=dict(file=str(p.relative_to(ROOT)),metadata=meta,samples=len(r),balance_seconds=float(t[-1]),
  first_ready_s=first((f&1)>0),max_abs_error_deg=float(abs(a['angle_err'][b]).max()),
  saturated_rows=int(np.sum((d&1)>0)),sp_clamped_rows=int(np.sum((d&0x80)>0)),imu_fault_rows=int(np.sum((d&0x400)>0)),can_tx_fault_rows=int(np.sum((d&0x800)>0)),
  first_full_reverse_s=first((t>4)&(a['pilot_forward']<-.9)),
  reverse_reaches_half_speed_s=first((t>4)&(a['filtered_vel']<-1)),
  first_full_forward_s=first((t>8)&(a['pilot_forward']>.9)),
  forward_reaches_half_speed_s=first((t>8)&(a['filtered_vel']>1)),
  forward_reaches_90percent_s=first((t>8)&(a['filtered_vel']>1.8)),
  inner_max_us=int(a['inner_dt_max_us'][b].max()),imu_age_max_ms=int(a['imu_age_ms'][b].max()),
  rear_feedback_max_after_first50ms=int(max(a['feedback_age_l_ms'][b&(t>.05)].max(),a['feedback_age_r_ms'][b&(t>.05)].max())),
  min_bus_voltage=float(a['bus_voltage'][b].min()),final_2s_velocity_rms=float(np.sqrt(np.mean(a['filtered_vel'][(t>t[-1]-2)&(t<t[-1]-.02)]**2))))
(OUT/'v2-trial-metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
fig,ax=plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
ax[0].plot(t,a['pilot_forward']*2,label='CH2 demand × 2 rad/s',c='#6b7280');ax[0].plot(t,a['target_vel'],label='shaped velocity target',c='#2563eb');ax[0].plot(t,a['filtered_vel'],label='measured velocity',c='#ea580c');ax[0].set_ylabel('wheel rad/s');ax[0].legend(ncol=3)
ax[1].plot(t,a['pilot_turn'],label='turn target');ax[1].plot(t,(a['bl_vel']-a['br_vel'])/2,label='actual differential',alpha=.8);ax[1].set_ylabel('turn rad/s');ax[1].legend(ncol=2)
ax[2].plot(t,a['roll'],label='body tilt');ax[2].plot(t,a['setpoint'],label='setpoint');ax[2].set_ylim(83,89);ax[2].set_ylabel('degrees');ax[2].legend(ncol=2)
ax[3].plot(t,a['vel_integral'],label='learned correction');ax[3].plot(t,a['vel_p_term'],label='velocity P correction');ax[3].set_ylabel('degrees');ax[3].legend(ncol=2)
for axis in ax:axis.set_xlim(0,t[-1]);axis.grid(alpha=.2)
ax[-1].set_xlabel('Seconds after balance engagement')
fig.suptitle('Response v2: commands arrive; speed takes seconds to follow\nActual robot · source d1ae97d · no recorded saturation or fault')
fig.savefig(OUT/'v2-delayed-response.png',dpi=150)
print(json.dumps({k:v for k,v in metrics.items() if k!='metadata'},indent=2))
