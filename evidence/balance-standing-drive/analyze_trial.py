"""First physical standing-drive run: identify delay, preserve measured evidence."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
PATH=ROOT/'telemetry_logs/bal_20260919_155240_standing-drive-first-test.csv'
lines=PATH.read_text().splitlines();meta=dict(x[2:].split('=',1) for x in lines if x.startswith('# ') and '=' in x)
r=list(csv.DictReader(x for x in lines if not x.startswith('#')))
a={k:np.array([float(x[k]) for x in r]) for k in r[0]};b=a['state']==2
origin=a['t_ms'][b][0];t=(a['t_ms']-origin)/1000
f=a['pilot_flags'].astype(int);d=a['diag_flags'].astype(int)
def first(mask):
 ids=np.flatnonzero(mask)
 return float(t[ids[0]]) if len(ids) else None
m=dict(file=str(PATH.relative_to(ROOT)),metadata=meta,samples=len(r),balance_seconds=float(t[-1]),
 first_ready_s=first((f&1)>0),ready_dropouts_before_disarm=int(np.sum(np.diff(((f[:-1]&1)>0).astype(int))==-1)),
 recovery_start_s=first((d&0x1000)>0),recovery_settled_s=first((d&0x8000)>0),
 moving_start_s=first((f&2)>0),pilot_max_velocity=float(np.max(np.abs(a['target_vel'][(f&2)>0]))),
 pilot_max_turn=float(np.max(np.abs(a['pilot_turn']))),min_bus_voltage=float(a['bus_voltage'].min()),
 max_tracking_error_deg=float(np.max(abs(a['angle_err'][b]))),
 saturation_rows=int(np.sum((d&1)>0)),imu_fault_rows=int(np.sum((d&0x400)>0)),
 can_tx_fault_rows=int(np.sum((d&0x800)>0)),inner_max_us=int(a['inner_dt_max_us'][b].max()),
 balance_sample_max_ms=int(a['sample_dt_ms'][b].max()),
 imu_age_max_ms=int(a['imu_age_ms'][b].max()),
 rear_feedback_max_after_first_50ms=int(max(a['feedback_age_l_ms'][b&(t>.05)].max(),a['feedback_age_r_ms'][b&(t>.05)].max())),
 max_arm_assist_fraction=float(abs(a['arm_assist_frac'][b]).max()),episodes=[])
for name,lo,hi,sgn in [('forward',20.8,31.5,1),('reverse',32.92,45.58,-1)]:
 mask=(t>=lo-1e-6)&(t<hi)
 cross=first(mask&(sgn*a['filtered_vel']>.25));target=first(mask&(sgn*a['target_vel']>.9))
 m['episodes'].append(dict(name=name,input_start_s=lo,input_end_s=hi,
   target_point9_delay_s=target-lo,measured_point25_delay_s=cross-lo,
   peak_velocity_rad_s=float(np.max(sgn*a['filtered_vel'][mask])),
   integral_start_deg=float(a['vel_integral'][mask][0]),integral_end_deg=float(a['vel_integral'][mask][-1])))
# Least-squares measured turn slope: diagnostic only, not a yaw-angle measurement.
turn=b&(abs(a['pilot_turn'])>.2)
dv=(a['bl_vel']-a['br_vel'])/2
m['turn_same_sign_fraction']=float(np.mean(dv[turn]*a['pilot_turn'][turn]>0))
m['turn_differential_error_rms']=float(np.sqrt(np.mean((dv[turn]-a['pilot_turn'][turn])**2)))
m['last_2s_velocity_rms']=float(np.sqrt(np.mean(a['filtered_vel'][(t>t[-1]-2)&(t<t[-1]-.02)]**2)))
(OUT/'trial-metrics.json').write_text(json.dumps(m,indent=2)+'\n')
fig,ax=plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
ax[0].plot(t,a['pilot_forward'],label='CH2 after deadband',c='#6b7280');ax[0].plot(t,a['target_vel'],label='requested velocity',c='#1d4ed8');ax[0].plot(t,a['filtered_vel'],label='measured velocity',c='#ea580c');ax[0].set_ylabel('wheel rad/s / stick');ax[0].legend(ncol=3)
ax[1].plot(t,a['pilot_turn'],label='requested per-wheel turn');ax[1].plot(t,dv,label='measured differential',alpha=.8);ax[1].set_ylabel('turn rad/s');ax[1].legend(ncol=2)
ax[2].plot(t,a['roll'],label='body tilt');ax[2].plot(t,a['setpoint'],label='balance setpoint');ax[2].set_ylabel('degrees');ax[2].set_ylim(82,90);ax[2].legend(ncol=2)
ax[3].plot(t,a['vel_integral'],label='equilibrium integral');ax[3].plot(t,a['vel_p_term'],label='velocity P correction');ax[3].set_ylabel('degrees');ax[3].legend(ncol=2)
for axis in ax:
 axis.grid(alpha=.2);axis.set_xlim(0,t[-1]);axis.axvline(m['first_ready_s'],c='#15803d',ls=':',alpha=.7)
 ax[0].axvspan(20.8,27,alpha=.07,color='red') if axis is ax[0] else None
ax[-1].set_xlabel('Seconds after balance engagement')
fig.suptitle('First standing-drive test: inputs arrive promptly; forward/reverse motion lags\nSource 90f07e8 · actual robot telemetry · 2,881 samples')
fig.savefig(OUT/'first-drive-trial.png',dpi=150)
print(json.dumps({k:v for k,v in m.items() if k!='metadata'},indent=2))
