"""Offline response experiments against frozen v1; no hardware access."""
import ctypes as ct
import itertools, json, subprocess, sys, types
from dataclasses import replace
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]; OUT=Path(__file__).parent
base='90f07e83e3472cebbc5ca2646e585bfbce20f2d7'
frozen=ROOT/'output/drive-response-baseline'
for name in ('scripts/balance_sim.py','src/config.h','src/balance_pilot.h','src/balance_math.h'):
 p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True)
 p.write_bytes(subprocess.check_output(['git','show',f'{base}:{name}'],cwd=ROOT))
bridge='''#include "balance_pilot.h"
struct Trial { balance_math::BalancePilot p; balance_math::PilotConfig c; };
extern "C" {
void* create(float speed,float accel,float decel) { return new Trial{{},{.06f,speed,1.5f,accel,decel,3.f,400,400}}; }
void destroy(void* x) { delete static_cast<Trial*>(x); }
void tick(void* x,int phase,int fresh,float v,float rate,float err,float f,float dt,float* out) {
 auto& t=*static_cast<Trial*>(x);
 t.p.update(phase&&fresh&&err<5&&std::fabs(rate)<30,phase&&std::fabs(v)<.3f&&std::fabs(rate)<4&&err<1,f,0,dt,t.c);
 out[0]=t.p.ready();out[1]=t.p.moving();out[2]=t.p.velocity(); }
}'''
(frozen/'bridge.cpp').write_text(bridge)
subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2',f'-I{frozen}/src','-Itests/stubs',str(frozen/'bridge.cpp'),'-o',str(frozen/'bridge.dylib')],cwd=ROOT,check=True)
lib=ct.CDLL(str(frozen/'bridge.dylib'));lib.create.argtypes=[ct.c_float]*3;lib.create.restype=ct.c_void_p
lib.destroy.argtypes=[ct.c_void_p]
lib.tick.argtypes=[ct.c_void_p,ct.c_int,ct.c_int]+[ct.c_float]*5+[ct.POINTER(ct.c_float)]
source=(frozen/'scripts/balance_sim.py').read_text()
# Experimental low-error gain changes only while production pilot is moving.
start=source.index('        abs_err = abs(vel_err)')
end=source.index('        arm_dev =',start)
piece=source[start:end].replace('c.vel_sp_kp_low','low_kp')
source=source[:start]+'        low_kp = self.pilot.kp if pilot_moving else c.vel_sp_kp_low\n'+piece+source[end:]
m=types.ModuleType('response_model');m.__file__=str(frozen/'scripts/balance_sim.py');sys.modules[m.__name__]=m
exec(compile(source,m.__file__,'exec'),m.__dict__)
cfg=m.current_firmware_config(); plant=m.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json')
class Pilot:
 def __init__(self,variant,profile):
  self.kp=variant['kp'];self.ptr=lib.create(variant['speed'],variant['accel'],variant['decel']);self.profile=profile
  self.moving=False;self.velocity=0.;self.rows=[]
 def __del__(self):lib.destroy(self.ptr)
 def update(self,now,phase,v,rate,err,dt):
  f,fresh=self.profile(now/1000);o=(ct.c_float*3)();lib.tick(self.ptr,phase,fresh,v,rate,err,f,dt,o)
  self.moving=bool(o[1]);self.velocity=o[2];self.rows.append([now/1000,f,*o])
last=None
def factory(var,profile):
 def make():
  global last
  last=Pilot(var,profile);return last
 return make
profiles={'forward_stop_reverse':lambda t:(1 if 16<=t<26 else -1 if 34<=t<46 else 0,True),
          'input_loss':lambda t:(1 if 16<=t<25 else 0,not(20<=t<21))}
variants=[dict(name='installed',kp=cfg.vel_sp_kp_low,speed=1,accel=.5,decel=.75)]
for kp in (.7,1.,1.25):
 variants.append(dict(name=f'gain{kp}_speed3',kp=kp,speed=3,accel=2,decel=3))
variants.append(dict(name='gain1_speed2',kp=1.,speed=2,accel=1.5,decel=2))
allrows=[];nominal={}
for var in variants:
 rows=[]
 for af,bf,tau,offset in itertools.product((.7,1.,1.4),(.7,1.,1.4),(.01,.03,.06),(-.4,.4)):
  p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=-1.9,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1)
  for label,profile in profiles.items():
   b=m.simulate(cfg,p,engage_offset_deg=offset,duration_s=54,seed=1,pilot_factory=factory(var,profile))
   pr=np.array(last.rows);tt=np.array(b.t);v=np.array(b.wheel_vel);target=np.array(b.target_vel);moving=(tt>=18)&(tt<26);final=tt>=50
   rows.append(dict(variant=var['name'],profile=label,af=af,bf=bf,tau=tau,offset=offset,fell=b.fell,
     commanded=bool(np.any(abs(pr[:,4])>.1)),peak_speed=float(np.max(abs(v))),
     forward_mean=float(v[moving].mean()) if moving.any() else None,
     final_rms=float(np.sqrt(np.mean(v[final]**2))) if final.any() else None))
   if af==bf==1 and tau==.03 and offset==.4:
    nominal[var['name']+'_'+label]=dict(t=b.t[::5],v=b.wheel_vel[::5],target=b.target_vel[::5],roll=b.roll[::5],pilot=last.rows[::5])
 for label in profiles:
  rr=[x for x in rows if x['profile']==label];baseline=[x for x in allrows if x['variant']=='installed' and x['profile']==label]
  print(var['name'],label,'fails',sum(x['fell'] for x in rr),'commanded',sum(x['commanded'] for x in rr),'new_vs_v1',sum(not a['fell'] and b['fell'] for a,b in zip(baseline,rr)),flush=True)
 allrows+=rows
(OUT/'experiments.json').write_text(json.dumps(dict(baseline=base,variants=variants,results=allrows,nominal=nominal),indent=2)+'\n')
