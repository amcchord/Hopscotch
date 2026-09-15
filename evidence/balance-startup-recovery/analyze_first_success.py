"""Compare the first reported successful start with the preceding assisted trial."""
import sys,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_balance_logs import load_rows,parse_config
files=[('Previous: hand assistance','bal_20260914_223915_capture-trim-assisted.csv'),('Early recovery: reported success','bal_20260914_233101_early-recovery-first-success.csv')]
metrics=[];fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True,layout='constrained')
for j,(label,name) in enumerate(files):
 p=ROOT/'telemetry_logs'/name;all_rows=load_rows(p);cfg=parse_config(p);r=[x for x in all_rows if x['state']=='2']
 v=lambda key:np.array([float(x[key]) for x in r]);t=(v('t_ms')-v('t_ms')[0])/1000;first4=t<4
 flags=v('flags').astype(int);diag=v('diag_flags').astype(int)
 def first(mask):
  idx=np.flatnonzero(mask)
  return float(t[idx[0]]) if len(idx) else None
 events={key:first(mask) for key,mask in [('arm_return',flags&32!=0),('ramp_complete',flags&64!=0),('recovery',diag&0x1000!=0),('boost',diag&0x2000!=0),('settled',diag&0x8000!=0)]}
 end_boost=np.flatnonzero(diag&0x2000!=0);events['boost_last']=float(t[end_boost[-1]]) if len(end_boost) else None
 settled=np.flatnonzero(diag&0x8000!=0)
 result=dict(file=name,samples=len(all_rows),duration_s=(float(all_rows[-1]['t_ms'])-float(all_rows[0]['t_ms']))/1000,balance_s=float(t[-1]),engage_s=float(v('t_ms')[0]/1000),engage_tilt_deg=float(v('roll')[0]),features=int(cfg['telemetry_features']),file_checksum=cfg['checksum'],usb_checksum=cfg['transport_fnv1a'],end_reason=cfg['end_reason'],events_after_engage_s=events,first4_peak_speed_rad_s=float(max(abs(v('meas_vel')[first4]))),first4_peak_command_rad_s=float(max(abs(v('motor_vel')[first4]))),first4_peak_travel_rad=float(max(abs(v('meas_drift')[first4]))),whole_balance_peak_travel_rad=float(max(abs(v('meas_drift')))),final_travel_rad=float(v('meas_drift')[-1]),max_angle_error_deg=float(max(abs(v('angle_err')))),peak_integral_deg=float(max(abs(v('vel_integral')))),final_integral_deg=float(v('vel_integral')[-1]),settled_hold_travel_rad=float(v('meas_drift')[settled[0]]) if len(settled) else None,inner_max_us=float(max(v('inner_dt_max_us'))),imu_max_ms=float(max(v('imu_age_ms'))),feedback_after50ms_max_ms=float(max(np.maximum(v('feedback_age_l_ms'),v('feedback_age_r_ms'))[t>=.05])),sample_max_ms=float(max(v('sample_dt_ms'))),sample_p99_ms=float(np.percentile(v('sample_dt_ms'),99)),receiver_run_max_us=int(cfg['prof_crsf_max_us']),control_gap_max_us=int(cfg['prof_ctlgap_max_us']),stall_events=int(cfg['stall_events']),saturated_rows=int(sum((diag&1!=0)|(v('inner_sat_ticks')>0))),recovery_limit_rows=int(sum(diag&0x4000!=0)),imu_fault_rows=int(sum(diag&0x400!=0)),can_tx_fault_rows=int(sum(diag&0x800!=0)),arm_active_rows=int(sum(v('arm_stage')==1)))
 if len(settled):
  after=t>t[settled[0]]+.5
  result['post_settle_window_s']=float(t[-1]-t[settled[0]]-.5)
  result['post_settle_speed_rms']=float(np.sqrt(np.mean(v('meas_vel')[after]**2)))
  result['post_settle_travel_range']=float(np.ptp(v('meas_drift')[after]))
 metrics.append(result)
 axes[0,j].plot(t,v('meas_vel'),label='Measured wheel speed');axes[0,j].plot(t,v('motor_vel'),label='Command',alpha=.7)
 axes[1,j].plot(t,v('meas_drift'),label='Travel from engagement')
 axes[2,j].plot(t,v('vel_integral'),label='Learned correction')
 axes[0,j].set_title(label)
 for ax in axes[:,j]:
  for key,color,style in [('recovery','#0a8a60','--'),('ramp_complete','#b45309',':'),('settled','#7555a3','-.')]:
   if events[key] is not None:ax.axvline(events[key],color=color,ls=style,lw=1.4,label=key.replace('_',' '))
  ax.set_xlim(0,12);ax.grid(alpha=.2);ax.legend(fontsize=7,loc='best')
 axes[-1,j].set_xlabel('Seconds after balance engagement')
for ax,label in zip(axes[:,0],('Wheel speed (rad/s)','Wheel travel (rad)','Learned correction (degrees)')):ax.set_ylabel(label)
for i in range(3):
 lo=min(ax.get_ylim()[0] for ax in axes[i]);hi=max(ax.get_ylim()[1] for ax in axes[i])
 for ax in axes[i]:ax.set_ylim(lo,hi)
fig.suptitle('First successful early-recovery trial: slower initial roll, then settling\nOperator reports success; previous trial required a hand stop. Equal vertical scales.',fontsize=13)
fig.savefig(ROOT/'evidence/balance-startup-recovery/first-success-comparison.png',dpi=160)
(ROOT/'evidence/balance-startup-recovery/first-success-metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps(metrics,indent=2))
