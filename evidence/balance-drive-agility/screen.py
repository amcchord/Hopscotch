"""Offline production-pilot response screening; not a physical safety certification."""
import ctypes as ct
import hashlib,importlib.util,itertools,json,subprocess,sys
from dataclasses import replace
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
BASE='d1ae97d5ef31b96a9f3f39a2765998f06b8d965b'
TMP=ROOT/'output/drive-agility';TMP.mkdir(exist_ok=True)
frozen=TMP/'baseline'
for name in ('src/config.h','src/balance_pilot.h','src/balance_math.h','scripts/balance_sim.py','evidence/balance-drive-response/pilot_bridge.cpp'):
 p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(subprocess.check_output(['git','show',f'{BASE}:{name}'],cwd=ROOT))
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
old=load('agility_old',frozen/'scripts/balance_sim.py');new=load('agility_new',ROOT/'scripts/balance_sim.py')
libs={}
for label,inc,bridge in [('installed',frozen/'src',frozen/'evidence/balance-drive-response/pilot_bridge.cpp'),('candidate',ROOT/'src',OUT/'pilot_bridge.cpp')]:
 binary=TMP/f'{label}.dylib'
 subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2',f'-I{inc}','-Itests/stubs',str(bridge),'-o',str(binary)],cwd=ROOT,check=True)
 lib=ct.CDLL(str(binary));lib.pilot_create.restype=ct.c_void_p;lib.pilot_destroy.argtypes=[ct.c_void_p]
 lib.pilot_tick.argtypes=[ct.c_void_p,ct.c_int,ct.c_int]+[ct.c_float]*6+[ct.POINTER(ct.c_float)]
 lib.pilot_correction.argtypes=[ct.c_void_p]+[ct.c_float]*4+[ct.c_int];lib.pilot_correction.restype=ct.c_float
 if label=='candidate':
  lib.pilot_learning.argtypes=[ct.c_void_p,ct.c_float]
  lib.pilot_drive.argtypes=[ct.c_void_p]+[ct.c_float]*7+[ct.POINTER(ct.c_float)]
  lib.pilot_arm_error.argtypes=[ct.c_float]*2;lib.pilot_arm_error.restype=ct.c_float
 libs[label]=lib
class Pilot:
 def __init__(self,label,profile):
  self.governed_motion=label=='candidate';self.label=label;self.lib=libs[label];self.ptr=self.lib.pilot_create();self.profile=profile;self.moving=False;self.velocity=0;self.arm=0.;self.drive_active=False;self.rows=[]
 def __del__(self):self.lib.pilot_destroy(self.ptr)
 def update(self,now,phase,speed,rate,err,dt):
  f,y,fresh=self.profile(now/1000);out=(ct.c_float*6)()
  self.lib.pilot_tick(self.ptr,phase,fresh,speed,rate,err,f,y,dt,out)
  self.moving=bool(out[1]);self.velocity=out[2];self.arm=out[4];self.rows.append([now/1000,f,*out])
 def velocity_correction(self,error,low,high,knee,ramp):return self.lib.pilot_correction(self.ptr,error,low,high,knee,ramp)
 def drive_step(self,error,rate,speed,pd,previous,dt,limit):
  if self.label=='installed':return pd+self.velocity if self.moving else pd
  if not self.moving and not self.drive_active:return pd
  out=(ct.c_float*2)();self.lib.pilot_drive(self.ptr,error,rate,speed,pd,previous,dt,limit,out)
  self.drive_active=bool(out[1]);return out[0]
 def arm_velocity_error(self,speed,target):return self.lib.pilot_arm_error(speed,target) if self.label=='candidate' else speed-target
 def arm_command(self):return self.arm
 def learning_allowed(self,error):return self.label=='installed' or bool(self.lib.pilot_learning(self.ptr,error))
last=None
def factory(label,profile):
 def make():
  global last
  last=Pilot(label,profile);return last
 return make
plant=new.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json');cfg=new.current_firmware_config();oldcfg=old.current_firmware_config()
neutral=lambda t:(0,0,True)
neutral_cases=0
for delta,af,bf,tau,offset,seed in itertools.product((-1.9,-3.5,-4.5),(.7,1,1.4),(.7,1,1.4),(.01,.03,.06),(-.8,.8),(1,2)):
 p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1)
 a=old.simulate(oldcfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
 b=new.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed,pilot_factory=factory('candidate',neutral))
 assert a.cmd==b.cmd and a.drift==b.drift and a.setpoint==b.setpoint
 neutral_cases+=1
print('Neutral identical:',neutral_cases,flush=True)
profiles={
 'short_input':lambda t:(.15 if 16<=t<18 else -.15 if 24<=t<26 else 0,0,True),
 'full_input':lambda t:(1 if 16<=t<26 else -1 if 36<=t<46 else 0,0,True),
 'input_loss':lambda t:(1 if 16<=t<26 else 0,0,not(20<=t<21)),
}
rows=[];nominal={}
for af,bf,tau,offset in itertools.product((.7,1,1.4),(.7,1,1.4),(.01,.03,.06),(-.4,.4)):
 p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=-1.9,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1)
 for name,profile in profiles.items():
  pair={}
  for label,model,c in [('installed',old,oldcfg),('candidate',new,cfg)]:
   b=model.simulate(c,p,engage_offset_deg=offset,duration_s=56,seed=1,pilot_factory=factory(label,profile))
   pr=np.array(last.rows);v=np.array(b.wheel_vel);tt=np.array(b.t);late=tt>=52
   pair[label]=dict(fell=b.fell,commanded=bool(np.any(abs(pr[:,4])>.1)),peak_velocity=float(max(abs(v))),
     max_requested_velocity=float(max(abs(pr[:,4]))),peak_tilt_from90=float(max(abs(np.array(b.roll)-90))),
     final_velocity_rms=float(np.sqrt(np.mean(v[late]**2))) if np.any(late) else None,
     mean_forward_velocity=float(np.mean(v[(tt>=20)&(tt<26)])) if np.any((tt>=20)&(tt<26)) else None)
   if af==bf==1 and tau==.03 and offset==.4:
    nominal[name+'_'+label]=dict(t=b.t[::5],v=b.wheel_vel[::5],target=b.target_vel[::5],roll=b.roll[::5],pilot=last.rows[::5])
  rows.append(dict(profile=name,af=af,bf=bf,tau=tau,offset=offset,**pair))
for name in profiles:
 rr=[r for r in rows if r['profile']==name]
 print(json.dumps(dict(profile=name,cases=len(rr),old_falls=sum(r['installed']['fell'] for r in rr),new_falls=sum(r['candidate']['fell'] for r in rr),new_failures=sum(not r['installed']['fell'] and r['candidate']['fell'] for r in rr),old_commanded=sum(r['installed']['commanded'] for r in rr),new_commanded=sum(r['candidate']['commanded'] for r in rr))),flush=True)
hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in (
 'src/balance_pilot.h','src/balance_drive.h','src/balance_controller.cpp','src/config.h',
 'scripts/balance_sim.py','evidence/balance-drive-agility/pilot_bridge.cpp')}
(OUT/'screen.json').write_text(json.dumps(dict(baseline=BASE,source_sha256=hashes,neutral_identical_cases=neutral_cases,results=rows,nominal=nominal),indent=2)+'\n')
