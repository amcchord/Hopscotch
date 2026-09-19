"""Production-C++ pilot in the planar plant; no yaw, slip or contact model."""
import ctypes as ct
import importlib.util
import itertools
import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2','-Isrc','-Itests/stubs',
                str(OUT/'pilot_bridge.cpp'),'-o','output/pilot_bridge.dylib'],cwd=ROOT,check=True)
lib=ct.CDLL(str(ROOT/'output/pilot_bridge.dylib'))
lib.pilot_create.restype=ct.c_void_p
lib.pilot_destroy.argtypes=[ct.c_void_p]
lib.pilot_tick.argtypes=[ct.c_void_p,ct.c_int,ct.c_int]+[ct.c_float]*6+[ct.POINTER(ct.c_float)]

class Pilot:
    def __init__(self, profile):
        self.ptr=lib.pilot_create();self.profile=profile;self.moving=False;self.velocity=0.;self.rows=[]
    def __del__(self):
        if self.ptr:lib.pilot_destroy(self.ptr)
    def update(self, now, phase, speed, rate, err, dt):
        f,y,fresh=self.profile(now/1000)
        out=(ct.c_float*4)()
        lib.pilot_tick(self.ptr,phase,fresh,speed,rate,err,f,y,dt,out)
        self.moving=bool(out[1]);self.velocity=out[2]
        self.rows.append([now/1000,f,*out])

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m

base='39720e9db861d7bd8cacf65bfc6853e7222c5c95'
frozen=ROOT/'output/standing-drive-baseline'
for name in ('scripts/balance_sim.py','src/config.h'):
    p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes(subprocess.check_output(['git','show',f'{base}:{name}'],cwd=ROOT))
old=load('drive_old',frozen/'scripts/balance_sim.py');new=load('drive_new',ROOT/'scripts/balance_sim.py')
cfg=new.current_firmware_config();oldcfg=old.current_firmware_config()
plant=new.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json')
neutral=lambda t:(0.,0.,True)
last=None
def factory(profile):
    def make():
        global last
        last=Pilot(profile);return last
    return make
passive=[]
for delta,af,bf,tau,offset,seed in itertools.product((-1.9,-3.5,-4.5),(.7,1.,1.4),(.7,1.,1.4),(.01,.03,.06),(-.8,.8),(1,2)):
    p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=delta,
              feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
    a=old.simulate(oldcfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
    b=new.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed,pilot_factory=factory(neutral))
    assert a.cmd==b.cmd and a.drift==b.drift and a.setpoint==b.setpoint
    passive.append(dict(delta=delta,af=af,bf=bf,tau=tau,offset=offset,seed=seed,fell=b.fell))
print('Neutral sticks: all 324 complete trajectories identical to installed firmware.',flush=True)

def drive(t):
    return (1. if 16<=t<20 else (-1. if 27<=t<31 else 0.),0.,True)
def dropout(t):
    # Loss of fresh input during travel; held input on return must not resume.
    return (1. if 16<=t<24 else 0.,0.,not (18<=t<19))
results=[]
for af,bf,tau,offset in itertools.product((.7,1.,1.4),(.7,1.,1.4),(.01,.03,.06),(-.4,.4)):
    p=replace(plant,A=plant.A*af,B=plant.B*.66*bf,tau_m=tau,eq_tip_delta=-1.9,
              feedback_velocity_scale=1.,fb_hold_ticks=1,vel_meas_noise=.1)
    a=new.simulate(cfg,p,engage_offset_deg=offset,duration_s=40,seed=1)
    for label,profile in [('forward_stop_reverse_stop',drive),('input_loss',dropout)]:
        b=new.simulate(cfg,p,engage_offset_deg=offset,duration_s=40,seed=1,pilot_factory=factory(profile))
        rows=np.array(last.rows);commanded=bool(np.any(abs(rows[:,4])>.1))
        tt=np.array(b.t);vel=np.array(b.wheel_vel)
        after=tt>=36
        results.append(dict(profile=label,af=af,bf=bf,tau=tau,offset=offset,
                            hold_fell=a.fell,drive_fell=b.fell,commanded=commanded,
                            first_ready_s=float(rows[np.flatnonzero(rows[:,2])[0],0]) if np.any(rows[:,2]) else None,
                            peak_velocity=float(np.max(abs(vel))),
                            final_velocity_rms=float(np.sqrt(np.mean(vel[after]**2))) if np.any(after) else None))
        if af==bf==1 and tau==.03 and offset==.4:
            (OUT/f'{label}-nominal.json').write_text(json.dumps(dict(t=b.t,velocity=b.wheel_vel,
                drift=b.drift,roll=b.roll,target=b.target_vel,pilot=last.rows),indent=2)+'\n')
for label in ('forward_stop_reverse_stop','input_loss'):
    rows=[r for r in results if r['profile']==label]
    s=dict(profile=label,cases=len(rows),baseline_failures=sum(r['hold_fell'] for r in rows),
           drive_failures=sum(r['drive_fell'] for r in rows),
           new_failures=sum(not r['hold_fell'] and r['drive_fell'] for r in rows),
           actual_commanded_cases=sum(r['commanded'] for r in rows),
           survivors_with_final_rms_below_point3=sum(not r['drive_fell'] and r['commanded'] and r['final_velocity_rms']<.3 for r in rows))
    print(json.dumps(s),flush=True)
(OUT/'screen.json').write_text(json.dumps(dict(neutral_identical_cases=len(passive),driving=results),indent=2)+'\n')
