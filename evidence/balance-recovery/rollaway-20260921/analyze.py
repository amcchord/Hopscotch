#!/usr/bin/env python3
"""Observed stand-up/arm behavior; no counterfactual physical-success claim."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_fast_tip_drift import analyze
NAMES=['bal_20260921T023411Z_standup_rollaway_wifi.csv',
       'bal_20260921T013959Z_fast_lower_speed_wifi.csv',
       'bal_20260921T011755Z_fast_tip_success_drift_wifi.csv']

def load(name):
 return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(
  l for l in (ROOT/'telemetry_logs'/name).read_text().splitlines() if not l.startswith('#'))]

def event(rows,predicate):
 r=next((r for r in rows if predicate(r)),None)
 if r is None:return None
 keys=['t_ms','roll','roll_rate','setpoint','base_sp','raw_base_sp','motor_vel',
       'filtered_vel','vel_integral','arm_stage','arm_calm_ms','arm_assist_frac',
       'arm_demand','arm_l','arm_r','arm_l_vel','arm_r_vel','meas_drift']
 return {k:r[k] for k in keys}

reports=[]
for name in NAMES:
 stats,rows=analyze(ROOT/'telemetry_logs'/name)
 stats['source']='telemetry_logs/'+name
 stats['ramp_event']=event(rows,lambda r:int(r['flags'])&64)
 stats['arm_emergency_event']=event(rows,lambda r:int(r['diag_flags'])&64)
 stats['first_saturation_event']=event(rows,lambda r:r['inner_sat_ticks']>0)
 stats['timing_maxima']={k:max(r[k] for r in rows) for k in
  ['sample_dt_ms','inner_dt_max_us','update_age_ms','imu_age_ms','feedback_age_l_ms','feedback_age_r_ms']}
 preimpact=[r for r in rows if abs(r['roll_rate'])<30 and abs(r['roll']-r['base_sp'])<15]
 stats['pre_large_fall_tracking']={
  'selection':'state2 before pilot/lower; abs(body rate)<30dps and abs(body-base)<15deg',
  'samples':len(preimpact),
  'wheel_difference_max_rad_s':max(abs(r['bl_vel']-r['br_vel']) for r in preimpact),
  'left_command_tracking_rms_rad_s':statistics.mean((r['cmd_left']-r['bl_vel'])**2 for r in preimpact)**.5,
  'right_command_tracking_rms_rad_s':statistics.mean((r['cmd_right']-r['br_vel'])**2 for r in preimpact)**.5}
 stats['fault_samples']={name:sum(bool(int(r['diag_flags'])&mask) for r in rows)
                        for name,mask in [('deadman_soft',2),('deadman_hard',4),('imu_stale',1024),('can_tx',2048)]}
 reports.append(stats)
rows=analyze(ROOT/'telemetry_logs'/NAMES[0])[1]
ramp=reports[0]['ramp_event']['t_ms']/1000
arm=reports[0]['arm_emergency_event']['t_ms']/1000
start=reports[0]['capture_s']
cool=[r for r in rows if ramp<=r['t_ms']/1000<arm]
reports[0]['post_return_cooldown']={'duration_s':arm-ramp,'samples':len(cool),
 'all_stage3':all(r['arm_stage']==3 for r in cool),
 'maximum_calm_ms':max(r['arm_calm_ms'] for r in cool),
 'full_negative_demand_samples':sum(r['arm_demand']<=-.299 for r in cool)}
fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True)
t=[r['t_ms']/1000 for r in rows]
for key,label in [('roll','Body'),('base_sp','Arm-scheduled base'),('setpoint','Commanded target')]:
 ax[0].plot(t,[r[key] if key!='base_sp' or r[key]!=0 else float('nan') for r in rows],label=label)
ax[0].set_ylabel('Angle (degrees)');ax[0].set_ylim(80,110)
for key,label,style in [('bl_vel','Left wheel','-'),('br_vel','Right wheel','--'),('motor_vel','Common command',':')]:
 ax[1].plot(t,[r[key] for r in rows],style,label=label)
ax[1].set_ylabel('Speed (rad/s)')
for key,label in [('vel_integral','Learned angle correction'),('sp_offset','Total target correction')]:
 ax[2].plot(t,[r[key] for r in rows],label=label)
ax[2].set_ylabel('Offset (degrees)')
for key,label in [('arm_demand','Requested assist'),('arm_assist_frac','Filtered assist target')]:
 ax[3].plot(t,[r[key] for r in rows],label=label)
ax[3].set_ylabel('Center-axis fraction')
for a in ax:
 a.axvspan(ramp,arm,color='#edc967',alpha=.23,label='Arms held in cooldown')
 a.axvline(arm,color='#bd4b41',ls=':',label='Emergency arm command')
 a.grid(alpha=.2);a.legend(fontsize=8,loc='best');a.spines[['top','right']].set_visible(False)
ax[0].set_title('Recorded stand-up failure: recovery overshoot and delayed arm assistance')
ax[-1].set_xlabel('Seconds from run start; plot clips final body impact above 110 degrees')
fig.tight_layout();fig.savefig(OUT/'observed.png',dpi=160);plt.close(fig)
# Lowering is a different contact/control regime: plot phases explicitly.
allrows=load(NAMES[1]);lower=[r for r in allrows if int(r['pilot_flags'])&64]
first=lower[0]['t_ms']/1000
shown=[r for r in allrows if first-.4<=r['t_ms']/1000<=first+2.2]
fwdL=allrows[0]['arm_l'];fwdR=allrows[0]['arm_r']
commit=next(r['t_ms']/1000 for r in lower if r['state']==4)
support=next(r['t_ms']/1000 for r in lower if (int(r['pilot_flags'])>>8)&15==3)
fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True)
t=[r['t_ms']/1000 for r in shown]
ax[0].plot(t,[r['roll'] for r in shown],label='Measured body angle')
ax[0].plot(t,[r['setpoint'] if r['state']==2 else float('nan') for r in shown],label='Upright target while balancing')
ax[0].set_ylabel('Angle (degrees)')
ax[1].plot(t,[.5*((r['arm_l']-fwdL)-(r['arm_r']-fwdR)) for r in shown],label='Measured mirrored-arm displacement')
ax[1].set_ylabel('Encoder displacement (rad)')
ax[2].plot(t,[.5*(r['arm_l_vel']-r['arm_r_vel']) for r in shown],label='Mirrored-arm velocity')
ax[2].plot(t,[.5*(r['bl_vel']+r['br_vel']) for r in shown],label='Common wheel velocity')
ax[2].set_ylabel('Rate (rad/s)')
for a in ax:
 a.axvline(commit,color='#bd4b41',ls=':',label='Upright PD hands off')
 a.axvline(support,color='#557c45',ls=':',label='Both-arm support confirmed')
 a.grid(alpha=.2);a.legend(fontsize=8);a.spines[['top','right']].set_visible(False)
ax[0].set_title('Successful lowering: arm motion, body response and contact regimes')
ax[-1].set_xlabel('Seconds from run start; contact forces prevent treating the entire trace as free balance')
fig.tight_layout();fig.savefig(OUT/'lowering-arms.png',dpi=160);plt.close(fig)
result={'runs':reports,'lowering_reference':{'source':'telemetry_logs/'+NAMES[1],
 'sha256':hashlib.sha256((ROOT/'telemetry_logs'/NAMES[1]).read_bytes()).hexdigest(),
 'request_s':first,'upright_handoff_s':commit,'both_support_s':support},
 'limits':['Arm demand/assist fraction are controller signals, not a calibrated center-of-mass measurement.',
 'Recorded arm/body motion does not identify arm inertia separately from wheel acceleration, gravity and contact.',
 'No quiet Forward equilibrium exists in the failure; an incorrect stored balance angle is not proven.',
 'Earlier intervention time does not establish that the fall would have been recovered.']}
(OUT/'observed.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'latest':reports[0],'lowering':result['lowering_reference']},indent=2))
