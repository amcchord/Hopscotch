"""Acceleration-state feedback experiment, independent of live firmware/device."""
import importlib.util,sys,types,itertools,json
from dataclasses import replace
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
# Frozen exploratory model, not the final production-helper screen. Keep the
# string substitutions tied to the exact model they were developed against.
source=(OUT/'rejected-reference-v3-source/balance_sim.py').read_text()
assert '                cmd += outer.pilot.velocity' in source
source=source.replace('* (vel_err - self.arm_assist_vel)', '* ((self.filtered_wheel_vel - clampf(self.filtered_wheel_vel, min(0,target_vel),max(0,target_vel)) if pilot_moving else vel_err) - self.arm_assist_vel)')
source=source.replace('self.arm_assist_frac += alpha', 'target=clampf(target+(self.pilot.arm if pilot_moving else 0),-c.arm_assist_range_neg,c.arm_assist_range_pos)\n            self.arm_assist_frac += alpha')
source=source.replace('    hard_stop_latched = False\n','    hard_stop_latched = False\n    accel_drive_active=False\n    accel_drive_cmd=0.\n')
source=source.replace('                cmd += outer.pilot.velocity','''                if not accel_drive_active: accel_drive_cmd=last_cmd
                accel_drive_active=True
                pa=outer.pilot
                a=pa.ka*angle_err-pa.kd*gyro_filt+pa.kv*clampf(outer.filtered_wheel_vel-pa.velocity,-8,8)
                accel_drive_cmd=clampf(accel_drive_cmd+clampf(a,-pa.limit,pa.limit)*INNER_DT,-cmd_max,cmd_max)
                cmd=accel_drive_cmd
            else:
                accel_drive_active=False''')
m=types.ModuleType('accel_experiment_model');m.__file__=str(ROOT/'scripts/balance_sim.py');sys.modules[m.__name__]=m;exec(compile(source,m.__file__,'exec'),m.__dict__)
cfg=m.current_firmware_config();plant=m.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json')
class Pilot:
 def __init__(self,params,profile):
  self.ka,self.kd,self.kv,self.limit=params;self.profile=profile;self.velocity=0.;self.moving=False;self.ready=False;self.neutral_ms=0;self.stop_ms=0;self.governed_motion=True;self.arm=0.;self.rows=[]
 def update(self,now,phase,speed,rate,err,dt):
  f,fresh=self.profile(now/1000);neutral=abs(f)<.06
  allowed=phase and fresh and err<5 and abs(rate)<30
  calm=phase and abs(speed)<.3 and abs(rate)<4 and err<1
  if not allowed:self.ready=False;self.neutral_ms=0
  elif not self.ready:
   self.neutral_ms=self.neutral_ms+dt*1000 if neutral and calm else 0
   self.ready=self.neutral_ms+.001>=400
  target=(0 if neutral else np.sign(f)*(abs(f)-.06)/.94*20) if self.ready else 0
  if target:self.moving=True
  step=(16 if abs(target)<abs(self.velocity) or target*self.velocity<0 else 12)*dt
  change=np.clip(target-self.velocity,-step,step)
  self.velocity+=change
  arm_target=np.clip(-.0083333*change/dt,-.1,.1)
  self.arm+=(dt/(.08+dt))*(arm_target-self.arm)
  self.stop_ms=self.stop_ms+dt*1000 if calm and self.velocity==0 and target==0 else 0
  if self.stop_ms+.001>=400:self.moving=False
  self.rows.append([now/1000,f,self.velocity,float(self.ready),float(self.moving)])
 def velocity_correction(self,*args):return 0
 def learning_allowed(self,error):return abs(error)<1
last=None
def factory(params,profile):
 def make():
  global last
  last=Pilot(params,profile);return last
 return make
profiles={'short':lambda t:(.15 if 16<=t<18 else -.15 if 24<=t<26 else 0,True),
 'full':lambda t:(1 if 16<=t<26 else -1 if 36<=t<46 else 0,True),
 'loss':lambda t:(1 if 16<=t<26 else 0,not(20<=t<21))}
variants=[(8.8,3.1,3,100)]
allrows=[];nominal={}
for params in variants:
 for label,profile in profiles.items():
  rows=[]
  for af,bf,tau,offset in itertools.product((.7,1,1.4),(.7,1,1.4),(.01,.03,.06),(-.4,.4)):
   p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=-1.9,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1)
   b=m.simulate(cfg,p,engage_offset_deg=offset,duration_s=56,seed=1,pilot_factory=factory(params,profile))
   pr=np.array(last.rows);t=np.array(b.t);v=np.array(b.wheel_vel);late=t>=52
   rows.append(dict(variant=params,profile=label,af=af,bf=bf,tau=tau,offset=offset,fell=b.fell,commanded=bool(np.any(abs(pr[:,2])>.1)),peak_speed=float(max(abs(v))),final_rms=float(np.sqrt(np.mean(v[late]**2))) if np.any(late) else None))
   if af==bf==1 and tau==.03 and offset==.4:nominal[str(params)+'_'+label]=dict(t=b.t[::5],v=b.wheel_vel[::5],target=b.target_vel[::5],roll=b.roll[::5],pilot=last.rows[::5])
  allrows+=rows
  print(params,label,'falls',sum(r['fell'] for r in rows),'commanded',sum(r['commanded'] for r in rows),flush=True)
(OUT/'acceleration-experiment.json').write_text(json.dumps(dict(results=allrows,nominal=nominal),indent=2)+'\n')
