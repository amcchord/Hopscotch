"""Production C++ controller bridge and uncertain planar drive plant.

This is an offline screen, not a validation of physical reliability.
"""
import ctypes as ct
import importlib.util,sys,subprocess
from pathlib import Path
from dataclasses import replace
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
TMP=ROOT/'output/drive-braking-experiments';TMP.mkdir(exist_ok=True)
spec=importlib.util.spec_from_file_location('damping_sim',ROOT/'scripts/balance_sim.py')
sim=importlib.util.module_from_spec(spec);sys.modules[spec.name]=sim;spec.loader.exec_module(sim)
subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2','-Isrc','-Itests/stubs',str(OUT/'experiment_bridge.cpp'),'-o',str(TMP/'pilot.dylib')],cwd=ROOT,check=True)
lib=ct.CDLL(str(TMP/'pilot.dylib'))
lib.pilot_create.restype=ct.c_void_p;lib.pilot_destroy.argtypes=[ct.c_void_p]
lib.pilot_arm_cap.argtypes=[ct.c_void_p,ct.c_float]
lib.pilot_brake_params.argtypes=[ct.c_void_p]+[ct.c_float]*5
lib.pilot_stop_params.argtypes=[ct.c_void_p,ct.c_int]+[ct.c_float]*4
lib.pilot_params.argtypes=[ct.c_void_p,ct.POINTER(ct.c_float)]
lib.pilot_tick.argtypes=[ct.c_void_p,ct.c_int,ct.c_int]+[ct.c_float]*6+[ct.POINTER(ct.c_float)]
lib.pilot_correction.argtypes=[ct.c_void_p]+[ct.c_float]*4+[ct.c_int];lib.pilot_correction.restype=ct.c_float
lib.pilot_learning.argtypes=[ct.c_void_p,ct.c_float]
lib.pilot_drive.argtypes=[ct.c_void_p]+[ct.c_float]*8+[ct.POINTER(ct.c_float)]
lib.pilot_arm_error.argtypes=[ct.c_float]*2;lib.pilot_arm_error.restype=ct.c_float
lib.pilot_arm_demand.argtypes=[ct.c_void_p,ct.c_float];lib.pilot_arm_demand.restype=ct.c_float
lib.pilot_emergency.argtypes=[ct.c_void_p,ct.c_int]+[ct.c_float]*6+[ct.c_int]
OLD=[8.8,3.1,3,.06,12,16,1,.45,0]
class Pilot:
 def __init__(self,params,profile,capture=(False,1,2.5,8,60),brake=None,api=lib,arm_cap=None):
  self.api=api
  self.governed_motion=True;self.ptr=self.api.pilot_create();self.profile=profile
  if arm_cap is not None:self.api.pilot_arm_cap(self.ptr,arm_cap)
  if params is not None:self.api.pilot_params(self.ptr,(ct.c_float*9)(*params))
  if hasattr(self.api,"pilot_stop_params"):self.api.pilot_stop_params(self.ptr,*capture)
  if brake is not None:self.api.pilot_brake_params(self.ptr,*brake)
  self.drive_active=False;self.moving=False;self.velocity=0.;self.arm=0.;self.rows=[];self.emergencies=0
 def __del__(self):self.api.pilot_destroy(self.ptr)
 def update(self,now,phase,speed,rate,err,dt):
  f,y,fresh=self.profile(now/1000);out=(ct.c_float*6)()
  self.api.pilot_tick(self.ptr,phase,fresh,speed,rate,err,f,y,dt,out)
  self.moving=bool(out[1]);self.velocity=out[2];self.arm=out[4];self.rows.append([now/1000,f,y,*out])
 def velocity_correction(self,error,low,high,knee,ramp):return self.api.pilot_correction(self.ptr,error,low,high,knee,ramp)
 def drive_step_raw(self,error,raw,old,speed,pd,previous,dt,limit):
  was_active=self.drive_active
  out=(ct.c_float*2)();self.api.pilot_drive(self.ptr,error,raw,old,speed,pd,previous,dt,limit,out)
  self.drive_active=bool(out[1])
  # Avoid a Python-double to C++-float roundtrip on unchanged PD passthrough.
  return out[0] if self.moving or was_active else pd
 def arm_velocity_error(self,speed,target):return self.api.pilot_arm_error(speed,target)
 def arm_demand(self,demand):return self.api.pilot_arm_demand(self.ptr,demand)
 def arm_emergency(self,moving,cmd,velerr,err,rate,dt,limit,old):
  result=bool(self.api.pilot_emergency(self.ptr,moving,cmd,velerr,err,rate,dt,limit,old));self.emergencies+=int(moving and result);return result
 def arm_command(self):return self.arm
 def learning_allowed(self,error):return bool(self.api.pilot_learning(self.ptr,error))
cfg=sim.current_firmware_config()
base=sim.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json')
plant=replace(base,A=25,B=13,motor_omega=50,motor_zeta=.5,motor_delay_s=0,sensor_delay_s=0,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1,eq_tip_delta=-1.9)
profiles={
 'small':lambda t:(.15 if 2<=t<5 else -.15 if 10<=t<13 else 0,0,True),
 'full':lambda t:(1 if 2<=t<9 else -1 if 18<=t<25 else 0,0,True),
 'reverse':lambda t:(.5 if 2<=t<7 else -.5 if 7<=t<12 else 0,0,True),
 'loss':lambda t:(1 if 2<=t<12 else 0,0,not(6<=t<7)),
 'turn':lambda t:(0,.5 if 2<=t<5 else -.5 if 8<=t<11 else 0,True),
}
def run(params=None,profile='small',p=plant,duration=36,pushes=None,seed=1,capture=(False,1,2.5,8,60),brake=None,arm_cap=None):
 pilot=Pilot(params,profiles[profile] if isinstance(profile,str) else profile,capture,brake,arm_cap=arm_cap)
 r=sim.simulate(cfg,p,engage_offset_deg=0,duration_s=duration,seed=seed,pilot_factory=lambda:pilot,settled_start=True,pushes=pushes)
 return r,pilot

def metrics(r,pilot):
 t=np.array(r.t);v=np.array(r.wheel_vel);roll=np.array(r.roll);rows=np.array(pilot.rows)
 final=t>=t[-1]-2
 return dict(fell=r.fell,end=float(t[-1]),peak_velocity=float(max(abs(v))),tilt_span=float(np.ptp(roll)),final_rms=float(np.sqrt(np.mean(v[final]**2))),requested=float(max(abs(rows[:,5]))),ready_samples=int(sum(rows[:,3])),emergency_samples=pilot.emergencies)
if __name__=='__main__':
 for x in [OLD,[10,1,1,.006,6,8,.3,.12,1],[12,1,1,.015,6,8,.3,.12,1],[10,1,1.5,.01,6,8,.3,.12,1]]:
  for prof in ('small','full','loss','turn'):
   r,p=run(x,prof);print(x[:4],prof,metrics(r,p),flush=True)
