"""Frozen installed v4 versus candidate source; no device I/O."""
from model import *
import itertools,json,hashlib,csv,bisect
BASE='aa9ae13'
frozen=TMP/'baseline'
for name in ('src/config.h','src/balance_pilot.h','src/balance_math.h','src/balance_drive.h','scripts/balance_sim.py','evidence/balance-drive-damping/pilot_bridge.cpp'):
 p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(subprocess.check_output(['git','show',f'{BASE}:{name}'],cwd=ROOT))
subprocess.run(['clang++','-std=c++17','-shared','-fPIC','-O2',f'-I{frozen}/src','-Itests/stubs',str(frozen/'evidence/balance-drive-damping/pilot_bridge.cpp'),'-o',str(TMP/'v4.dylib')],cwd=ROOT,check=True)
oldlib=ct.CDLL(str(TMP/'v4.dylib'))
for name in ('pilot_create','pilot_destroy','pilot_tick','pilot_params','pilot_learning','pilot_arm_error','pilot_drive','pilot_correction','pilot_arm_demand','pilot_emergency'):
 a=getattr(lib,name);b=getattr(oldlib,name);b.argtypes=a.argtypes;b.restype=a.restype
spec=importlib.util.spec_from_file_location('braking_old',frozen/'scripts/balance_sim.py');oldsim=importlib.util.module_from_spec(spec);sys.modules[spec.name]=oldsim;spec.loader.exec_module(oldsim)
neutral=lambda t:(0,0,True)
neutral_count=0
for delta,af,bf,tau,offset,seed in itertools.product((-1.9,-3.5,-4.5),(.7,1,1.4),(.7,1,1.4),(.01,.03,.06),(-.8,.8),(1,2)):
 p=replace(base,A=base.A*af,B=base.B*.66*bf,tau_m=tau,eq_tip_delta=delta,feedback_velocity_scale=1,fb_hold_ticks=1,vel_meas_noise=.1)
 a=oldsim.simulate(oldsim.current_firmware_config(),p,engage_offset_deg=offset,duration_s=16,seed=seed)
 b=sim.simulate(cfg,p,engage_offset_deg=offset,duration_s=16,seed=seed,pilot_factory=lambda:Pilot(None,neutral))
 assert a.cmd==b.cmd and a.drift==b.drift and a.setpoint==b.setpoint
 neutral_count+=1
print('Neutral identical:',neutral_count,flush=True)
# Recorded stick commands are normalized/deadbanded. Undo that mapping before
# feeding the actual pilot. Replay is an input stress test, not a plant fit.
with (ROOT/'telemetry_logs/bal_20260919_wifi_v4_slow_stop.csv').open() as f:
 rr=list(csv.DictReader(line for line in f if not line.startswith('#')))
print('CSV columns',list(rr[0])[:8],flush=True)
bal=[r for r in rr if r['state']=='2']
t0=float(bal[0]['t_ms']);times=[(float(r['t_ms'])-t0)/1000 for r in bal]
def raw(x):
 x=float(x);return 0 if x==0 else np.copysign(abs(x)*.94+.06,x)
def replay(t):
 r=bal[min(len(bal)-1,max(0,bisect.bisect_right(times,t)-1))]
 return raw(r['pilot_forward']) if t<times[-1] else 0,raw(r['pilot_steering']) if t<times[-1] else 0,True
rows=[];nominal={}
for A,B,wn,z,delay in itertools.product((8,25,45),(4,7,13,16),(40,60),(.35,.6),(0,.01)):
 p=replace(plant,A=A,B=B,motor_omega=wn,motor_zeta=z,motor_delay_s=delay,sensor_delay_s=.005)
 for profile in ('small','full','reverse','loss','turn','push','replay'):
  fn=replay if profile=='replay' else profiles['small'] if profile=='push' else profiles[profile]
  pushes=[sim.Push(at_s=4,delta_rate=10,delta_vel=.5),sim.Push(at_s=15,delta_rate=-10,delta_vel=-.5)] if profile=='push' else []
  pair={}
  for label,api in [('v4',oldlib),('candidate',lib)]:
   pilot=Pilot(None,fn,api=api)
   r=sim.simulate(cfg,p,engage_offset_deg=0,duration_s=58 if profile=='replay' else 36,seed=1,pilot_factory=lambda:pilot,settled_start=True,pushes=pushes)
   m=metrics(r,pilot);t=np.array(r.t);v=np.array(r.wheel_vel)
   mask=(t>=3)&(t<min(t[-1],17));vv=v[mask]
   if len(vv)>=5:
    spectrum=np.fft.rfft((vv-np.mean(vv))*np.hanning(len(vv)));freq=np.fft.rfftfreq(len(vv),.02)
    m['ring_power']=float(np.sum(abs(spectrum[(freq>=3)&(freq<=6)])**2)/len(vv)**2)
   else:m['ring_power']=None
   m['commanded']=bool(max(abs(np.array(pilot.rows)[:,5]))>.1 or (profile=='turn' and max(abs(np.array(pilot.rows)[:,6]))>.1))
   pair[label]=m
   if (A,B,wn,z,delay)==(25,13,60,.6,0):
    nominal[profile+'_'+label]=dict(t=r.t,v=r.wheel_vel,target=r.target_vel,roll=r.roll,command=r.cmd,drift=r.drift,pilot=pilot.rows)
  rows.append(dict(A=A,B=B,wn=wn,z=z,delay=delay,profile=profile,**pair))
 if len(rows)%168==0:print('Completed driving conditions:',len(rows),flush=True)
summary={}
for profile in ('small','full','reverse','loss','turn','push','replay'):
 rr=[r for r in rows if r['profile']==profile]
 cc=[r for r in rr if all(r[k]['commanded'] and not r[k]['fell'] for k in ('v4','candidate'))]
 summary[profile]=dict(cases=len(rr),v4_falls=sum(r['v4']['fell'] for r in rr),candidate_falls=sum(r['candidate']['fell'] for r in rr),new_falls=sum(r['candidate']['fell'] and not r['v4']['fell'] for r in rr),v4_commanded=sum(r['v4']['commanded'] for r in rr),candidate_commanded=sum(r['candidate']['commanded'] for r in rr),common_standing_commanded=len(cc),v4_ring_median=float(np.median([r['v4']['ring_power'] for r in cc])),candidate_ring_median=float(np.median([r['candidate']['ring_power'] for r in cc])))
print(json.dumps(summary,indent=2),flush=True)
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'src/config.h',ROOT/'src/balance_drive.h',ROOT/'src/balance_pilot.h',ROOT/'src/balance_controller.cpp',ROOT/'src/balance_controller.h',ROOT/'scripts/balance_sim.py',OUT/'model.py',OUT/'pilot_bridge.cpp',OUT/'final_screen.py']}
(OUT/'final-screen.json').write_text(json.dumps(dict(baseline_source=BASE,source_sha256=hashes,neutral_identical=neutral_count,summary=summary,cases=rows),indent=2)+'\n')
(OUT/'final-nominal.json').write_text(json.dumps(nominal,separators=(',',':'))+'\n')
