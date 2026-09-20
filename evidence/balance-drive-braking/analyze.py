"""Analyze saved physical v4 drive run downloaded over Wi-Fi."""
import csv,json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
FILE=ROOT/'telemetry_logs/bal_20260919_wifi_v4_slow_stop.csv'
lines=FILE.read_text().splitlines();meta=dict(l[2:].split('=',1) for l in lines if l.startswith('# ') and '=' in l)
rows=list(csv.DictReader(l for l in lines if not l.startswith('#')));a={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
t=(a['t_ms']-a['t_ms'][a['state']==2][0])/1000;b=a['state']==2;flags=a['pilot_flags'].astype(int);diag=a['diag_flags'].astype(int)
neutral=np.abs(a['pilot_forward'])<.001
stops=np.flatnonzero(neutral[1:]&~neutral[:-1])+1
records=[]
for i in stops:
 end=np.flatnonzero(~neutral[i+1:]);j=i+1+end[0] if len(end) else len(t)-1
 ids=np.arange(i,j);v=a['filtered_vel'][ids];sign=np.sign(v[0]);peak=max(abs(v));start=t[i]
 def first(mask):
  at=np.flatnonzero(mask);return float(t[ids[at[0]]]-start) if len(at) else None
 settled=None
 for k in range(len(ids)-20):
  if np.max(abs(v[k:k+21]))<.3:settled=float(t[ids[k]]-start);break
 rec=dict(center_s=float(start),neutral_s=float(t[j]-start),speed_at_center=float(v[0]),request_at_center=float(a['target_vel'][i]),request_zero_s=first(abs(a['target_vel'][ids])<.001),half_speed_s=first(sign*v<abs(v[0])*.5),first_below_point3_s=first(abs(v)<.3),calm_point3_400ms_s=settled,peak_reverse=float(max(0,max(-sign*v))),travel_radians=float(a['meas_drift'][j-1]-a['meas_drift'][i]),peak_body_rate=float(max(abs(a['roll_rate'][ids]))),peak_angle_error=float(max(abs(a['angle_err'][ids]))),arm_min=float(min(a['arm_assist_frac'][ids])),arm_max=float(max(a['arm_assist_frac'][ids])),max_steering=float(max(abs(a['pilot_steering'][ids]))),integral_start=float(a['vel_integral'][i]),integral_end=float(a['vel_integral'][j-1]))
 records.append(rec)
metrics=dict(file=str(FILE.relative_to(ROOT)),csv_sha256=hashlib.sha256(FILE.read_bytes()).hexdigest(),samples=len(rows),balance_duration_s=float(t[b][-1]-t[b][0]),binary_checksum=meta['checksum'],transport_checksum=meta['transport_fnv1a'],features=meta['telemetry_features'],end_reason=meta['end_reason'],inner_max_us=int(max(a['inner_dt_max_us'][b])),sample_max_ms=int(max(a['sample_dt_ms'][b])),imu_max_age_ms=int(max(a['imu_age_ms'][b])),imu_faults=int(sum((diag&1024)!=0)),can_tx_faults=int(sum((diag&2048)!=0)),emergency_arms=int(sum((diag&64)!=0)),saturated_ticks=int(sum(a['inner_sat_ticks'][b])),stops=records)
fig,axes=plt.subplots(5,1,figsize=(13,12),sharex=True,layout='constrained')
axes[0].plot(t,a['target_vel'],label='ramped speed request',c='#2563eb');axes[0].plot(t,a['filtered_vel'],label='measured speed',c='#ea580c')
axes[0].plot(t,20*a['pilot_forward'],label='stick × 20',c='#64748b',alpha=.5,ls='--')
axes[1].plot(t,a['angle_err'],label='angle error');axes[1].plot(t,a['roll_rate']/10,label='filtered rate / 10',alpha=.7)
axes[2].plot(t,a['arm_assist_frac'],label='total arms');axes[2].plot(t,a['pilot_arm'],label='planned arms',ls='--')
axes[3].plot(t,a['vel_integral'],label='equilibrium correction');axes[3].plot(t,a['base_sp']-np.median(a['base_sp'][b]),label='base setpoint relative to run median')
axes[4].plot(t,a['meas_drift'],label='wheel travel');axes[4].plot(t,a['pilot_turn'],label='turn request')
for ax,y in zip(axes,('rad/s','degrees, deg/s ÷ 10','center fraction','degrees','wheel radians')):
 ax.set_ylabel(y);ax.legend(loc='upper left',ncol=3);ax.grid(alpha=.2);ax.set_xlim(3,50.8)
 for r in records:ax.axvline(r['center_s'],alpha=.15,color='k')
axes[-1].set_xlabel('Seconds after balance engagement; vertical lines mark CH2 returning to center')
fig.suptitle('Physical driving v4 over Wi-Fi: smooth travel, long braking and recoil\nUser contact/intervention times are unmarked')
fig.savefig(OUT/'physical-v4-stops.png',dpi=140)
(OUT/'physical-v4-metrics.json').write_text(json.dumps(metrics,indent=2)+'\n');print(json.dumps(metrics,indent=2))
