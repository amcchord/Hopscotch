"""Frozen installed-v3 vs final source-backed candidate. No device I/O."""
from model import *
import itertools,json,hashlib
BASE='5a07ebabef8f6431e8a9908fa4b53170b24a2308'
frozen=TMP/'baseline'
for name in ('src/config.h','src/balance_pilot.h','src/balance_math.h','src/balance_drive.h','scripts/balance_sim.py','evidence/balance-drive-agility/pilot_bridge.cpp'):
 p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(subprocess.check_output(['git','show',f'{BASE}:{name}'],cwd=ROOT))
subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2',f'-I{frozen}/src','-Itests/stubs',str(frozen/'evidence/balance-drive-agility/pilot_bridge.cpp'),'-o',str(TMP/'v3.dylib')],cwd=ROOT,check=True)
oldlib=ct.CDLL(str(TMP/'v3.dylib'))
for name in ('pilot_create','pilot_destroy','pilot_tick','pilot_learning','pilot_arm_error'):
 a=getattr(lib,name);b=getattr(oldlib,name);b.argtypes=a.argtypes;b.restype=a.restype
oldlib.pilot_drive.argtypes=[ct.c_void_p]+[ct.c_float]*7+[ct.POINTER(ct.c_float)]
class Baseline(Pilot):
 def __init__(self,profile):
  self.governed_motion=True;self.ptr=oldlib.pilot_create();self.profile=profile
  self.drive_active=False;self.moving=False;self.velocity=0.;self.arm=0.;self.rows=[];self.emergencies=0
 def __del__(self):oldlib.pilot_destroy(self.ptr)
 def update(self,now,phase,speed,rate,err,dt):
  f,y,fresh=self.profile(now/1000);out=(ct.c_float*6)()
  oldlib.pilot_tick(self.ptr,phase,fresh,speed,rate,err,f,y,dt,out)
  self.moving=bool(out[1]);self.velocity=out[2];self.arm=out[4];self.rows.append([now/1000,f,y,*out])
 def drive_step_raw(self,error,raw,old,speed,pd,previous,dt,limit):
  was_active=self.drive_active
  out=(ct.c_float*2)();oldlib.pilot_drive(self.ptr,error,old,speed,pd,previous,dt,limit,out)
  self.drive_active=bool(out[1]);return out[0] if self.moving or was_active else pd
 def arm_demand(self,demand):return demand
 def arm_emergency(self,moving,cmd,velerr,err,rate,dt,limit,old):
  self.emergencies+=int(moving and old);return old
 def learning_allowed(self,error):return bool(oldlib.pilot_learning(self.ptr,error))
# Neutral regression uses the original first-order plant and original model code.
spec=importlib.util.spec_from_file_location('damping_old',frozen/'scripts/balance_sim.py');oldsim=importlib.util.module_from_spec(spec);sys.modules[spec.name]=oldsim;spec.loader.exec_module(oldsim)
oldcfg=oldsim.current_firmware_config()
neutral=lambda t:(0,0,True)
neutral_count=0
for delta,af,bf,tau,offset,seed in itertools.product((-1.9,-3.5,-4.5),(.7,1,1.4),(.7,1,1.4),(.01,.03,.06),(-.8,.8),(1,2)):
 p=replace(base,A=base.A*af,B=base.B*.66*bf,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1)
 a=oldsim.simulate(oldcfg,p,engage_offset_deg=offset,duration_s=16,seed=seed)
 b=sim.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed,pilot_factory=lambda:Pilot(None,neutral))
 assert a.cmd==b.cmd and a.drift==b.drift and a.setpoint==b.setpoint
 neutral_count+=1
print('Neutral identical:',neutral_count,flush=True)
rows=[];nominal={}
conditions=list(itertools.product((8,25,45),(10,13,16),(40,50),(.35,.6),(0,.01)))
for A,B,wn,z,delay in conditions:
 p=replace(plant,A=A,B=B,motor_omega=wn,motor_zeta=z,motor_delay_s=delay,sensor_delay_s=.005)
 for profile in ('small','full','reverse','loss','turn','push'):
  profile_fn=profiles['small'] if profile=='push' else profiles[profile]
  pushes=[sim.Push(at_s=4,delta_rate=10,delta_vel=.5),sim.Push(at_s=15,delta_rate=-10,delta_vel=-.5)] if profile=='push' else []
  pair={}
  for label in ('v3','candidate'):
   pilot=Baseline(profile_fn) if label=='v3' else Pilot(None,profile_fn)
   r=sim.simulate(cfg,p,engage_offset_deg=0,duration_s=36,seed=1,pilot_factory=lambda:pilot,settled_start=True,pushes=pushes)
   m=metrics(r,pilot);t=np.array(r.t);v=np.array(r.wheel_vel)
   mask=(t>=3)&(t<min(t[-1],17));vv=v[mask]
   spectrum=np.fft.rfft((vv-np.mean(vv))*np.hanning(len(vv)));freq=np.fft.rfftfreq(len(vv),.02)
   band=(freq>=3)&(freq<=6)
   m['ring_power']=float(np.sum(abs(spectrum[band])**2)/len(vv)**2)
   m['dominant_ring_hz']=float(freq[band][np.argmax(abs(spectrum[band]))])
   m['commanded']=bool(np.any(np.abs(np.array(pilot.rows)[:,1:3])>.01) and any(x[4] for x in pilot.rows))
   pair[label]=m
   if (A,B,wn,z,delay)==(25,13,50,.35,.01):
    nominal[profile+'_'+label]=dict(t=r.t,v=r.wheel_vel,target=r.target_vel,roll=r.roll,command=r.cmd,drift=r.drift,pilot=pilot.rows)
  rows.append(dict(A=A,B=B,wn=wn,z=z,motor_delay_s=delay,sensor_delay_s=.005,profile=profile,**pair))
summary={}
for profile in ('small','full','reverse','loss','turn','push'):
 rr=[r for r in rows if r['profile']==profile]
 summary[profile]=dict(cases=len(rr),v3_falls=sum(r['v3']['fell'] for r in rr),candidate_falls=sum(r['candidate']['fell'] for r in rr),new_falls=sum(r['candidate']['fell'] and not r['v3']['fell'] for r in rr),commanded=sum(r['candidate']['commanded'] for r in rr),v3_ring_median=float(np.median([r['v3']['ring_power'] for r in rr])),candidate_ring_median=float(np.median([r['candidate']['ring_power'] for r in rr])))
print(json.dumps(summary,indent=2),flush=True)
hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ('src/balance_pilot.h','src/balance_drive.h','src/config.h','src/balance_controller.cpp','src/balance_controller.h','scripts/balance_sim.py','evidence/balance-drive-damping/pilot_bridge.cpp','evidence/balance-drive-damping/model.py','evidence/balance-drive-damping/screen.py')}
(OUT/'screen.json').write_text(json.dumps(dict(baseline_source=BASE,source_sha256=hashes,neutral_identical=neutral_count,summary=summary,cases=rows,nominal=nominal),indent=2)+'\n')
