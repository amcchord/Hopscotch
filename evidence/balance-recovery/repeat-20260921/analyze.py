#!/usr/bin/env python3
"""Compare repeated physical stand-up failures with successful fast captures."""
import csv,hashlib,json,statistics,sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]; OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
def analyze(path):
 lines=path.read_text().splitlines()
 rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(l for l in lines if not l.startswith('#'))]
 capture=next(r for r in rows if r['state']==2)
 returning=next(r for r in rows if r['state']==2 and int(r['flags'])&32)
 cutoff=next((r['t_ms'] for r in rows if int(r['pilot_flags'])&(2|64)),rows[-1]['t_ms']+1)
 b=[r for r in rows if r['state']==2 and r['t_ms']<cutoff]
 q=[r for r in b if int(r['flags'])&64 and r['arm_tip_frac']<.02 and abs(r['arm_assist_frac'])<.02
    and max(abs(r['bl_vel']),abs(r['br_vel']))<.7 and abs(r['roll_rate'])<4
    and max(abs(r['arm_l_vel']),abs(r['arm_r_vel']))<.3]
 event=lambda bit:next((r['t_ms']/1000 for r in b if int(r['flags'])&bit),None)
 common=lambda r:.5*(r['bl_vel']+r['br_vel'])
 result=dict(source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),rows=len(rows),
  duration_s=rows[-1]['t_ms']/1000,end_reason=next(l.split('=',1)[1] for l in lines if l.startswith('# end_reason=')),
  fast=bool(int(capture['pilot_flags'])&128),capture_s=capture['t_ms']/1000,capture_angle_deg=capture['roll'],
  arm_return_s=returning['t_ms']/1000,arm_return_angle_deg=returning['roll'],
  ramp_complete_s=event(64),recovery_trigger_s=event(128),
  recovery_settled_s=next((r['t_ms']/1000 for r in b if int(r['diag_flags'])&32768),None),
  cutoff_s=cutoff/1000,quiet_samples=len(q),
  quiet_medians={k:statistics.median(r[k] for r in q) if q else None for k in
   ['roll','accel_angle','gyro_raw','accel_norm','setpoint','base_sp','vel_integral','arm_l','arm_r']},
  peak_integral_deg=max(r['vel_integral'] for r in b),minimum_position_rad=min(r['meas_drift'] for r in b),
  maximum_position_rad=max(r['meas_drift'] for r in b),minimum_common_speed_rad_s=min(common(r) for r in b),
  maximum_common_speed_rad_s=max(common(r) for r in b),maximum_wheel_speed_difference_rad_s=max(abs(r['bl_vel']-r['br_vel']) for r in b))
 return result,b
SOURCES=[
 ('Slow success, no reboot','bal_20260921T030552Z_slow_comparison_wifi.csv','#9253a8'),
 ('Repeat failure','bal_20260921T030048Z_standup_repeat_wifi.csv','#bc3737'),
 ('Previous failure','bal_20260921T023411Z_standup_rollaway_wifi.csv','#e5913a'),
 ('Successful with drift','bal_20260921T011755Z_fast_tip_success_drift_wifi.csv','#306ca5'),
 ('Successful quiet start','bal_20260921T013959Z_fast_lower_speed_wifi.csv','#388d66')]
fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
results=[]
for label,name,color in SOURCES:
 path=ROOT/'telemetry_logs'/name;stats,rows=analyze(path)
 allrows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(l for l in path.read_text().splitlines() if not l.startswith('#'))]
 header=dict(l[2:].split('=',1) for l in path.read_text().splitlines() if l.startswith('# ') and '=' in l)
 stats.update(label=label,source='telemetry_logs/'+name,stored_trim_deg=float(header['stored_trim']),run_start_uptime_ms=int(header['run_start_uptime_ms']),initial_flat_angle_deg=allrows[0]['roll'],initial_accel_angle_deg=allrows[0]['accel_angle'],initial_voltage=allrows[0]['bus_voltage'],initial_arms_rad=[allrows[0]['arm_l'],allrows[0]['arm_r']])
 event=lambda mask: next((r for r in rows if int(r['diag_flags'])&mask),None)
 emergency=event(64)
 stats['arm_emergency_s']=emergency['t_ms']/1000 if emergency else None
 if stats['ramp_complete_s'] is not None:
  end=stats['arm_emergency_s'] or stats['cutoff_s']
  blocked=[r for r in rows if stats['ramp_complete_s']<=r['t_ms']/1000<end and r['arm_stage']==3 and abs(r['arm_demand'])>.29]
  stats['blocked_full_arm_demand_samples']=len(blocked)
  stats['ramp_to_emergency_s']=end-stats['ramp_complete_s'] if emergency else None
 stats['peak_sp_offset_deg']=max(r['sp_offset'] for r in rows)
 stats['maximum_timing']={k:max(r[k] for r in rows) for k in ['sample_dt_ms','inner_dt_max_us','update_age_ms','imu_age_ms','feedback_age_l_ms','feedback_age_r_ms']}
 stats['fault_samples']={k:sum(bool(int(r['diag_flags'])&mask) for r in rows) for k,mask in [('deadman_soft',2),('deadman_hard',4),('imu_stale',1024),('CAN_tx',2048)]}
 prefall=[r for r in rows if abs(r['roll_rate'])<30 and abs(r['roll']-r['base_sp'])<15]
 stats['tracking_before_large_fall']={'selection':'state2, before pilot/lower, abs(rate)<30 and abs(body-base)<15','samples':len(prefall),'left_RMS_rad_s':statistics.mean((r['cmd_left']-r['bl_vel'])**2 for r in prefall)**.5,'right_RMS_rad_s':statistics.mean((r['cmd_right']-r['br_vel'])**2 for r in prefall)**.5,'max_wheel_difference_rad_s':max(abs(r['bl_vel']-r['br_vel']) for r in prefall)}
 results.append(stats)
 shown=[r for r in rows if stats['arm_return_s']<=r['t_ms']/1000<=stats['arm_return_s']+5]
 t=[r['t_ms']/1000-stats['arm_return_s'] for r in shown]
 for ax,key in zip(axes,['roll','sp_offset','filtered_vel','vel_integral']):ax.plot(t,[r[key] for r in shown],color=color,label=label)
 if emergency:
  x=emergency['t_ms']/1000-stats['arm_return_s']
  axes[2].scatter([x],[emergency['filtered_vel']],s=60,marker='x',color=color,zorder=4)
axes[0].set_ylabel('Body angle (degrees)');axes[0].set_ylim(78,110)
axes[1].set_ylabel('Target offset (degrees)')
axes[2].set_ylabel('Filtered wheel speed (rad/s)')
axes[3].set_ylabel('Learned correction (degrees)')
for a in axes:a.grid(alpha=.2);a.spines[['top','right']].set_visible(False)
axes[0].legend(fontsize=8);axes[0].set_title('Repeated fast failures and slow-mode recovery')
axes[2].text(.02,.08,'× Emergency arm assistance begins',transform=axes[2].transAxes,fontsize=9)
axes[-1].set_xlabel('Seconds after arms begin returning; final body impact above110 degrees clipped')
fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=160);plt.close(fig)
result=dict(runs=results,limits=['Stored trim is the same across these runs; true equilibrium may differ.',
 'Similar supported capture angle does not imply similar free-balancing dynamics.',
 'A slow-mode comparison changes trajectory and supported-capture calibration policy, not only speed.',
 'Operator intervention and physical environment changes cannot be inferred from this telemetry alone.',
 'The slow run starts after a substantial flat accelerometer-angle change; its steady angle cannot be assigned retrospectively to earlier fast failures.'])
(OUT/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(results[0],indent=2))
