#!/usr/bin/env python3
"""Exploratory recovery/arm sensitivity screen. No policy is approved for OTA.

C++ supplies the installed acceleration controller. Python supplies the existing
approximate outer loop. Exact patch assertions fail if that model changes.
Arm inertia/servo parameters in the dynamic screen are hypotheses, not fits.
"""
import ctypes,sys,types,itertools,json,subprocess,hashlib,math,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parent
BUILD=ROOT/'output/startup-recovery-screen'
BUILD.mkdir(parents=True,exist_ok=True)
subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror','-shared','-fPIC','-Isrc',str(OUT/'bridge.cpp'),'-o',str(BUILD/'bridge.so')],cwd=ROOT,check=True)
lib=ctypes.CDLL(str(BUILD/'bridge.so'));lib.create.restype=ctypes.c_void_p;lib.destroy.argtypes=[ctypes.c_void_p]
lib.arm_speed.restype=ctypes.c_float
lib.step.argtypes=[ctypes.c_void_p,ctypes.c_bool]+[ctypes.c_float]*8;lib.step.restype=ctypes.c_float
class Drive:
 def __init__(self):self.p=lib.create()
 def __del__(self):lib.destroy(self.p)
 def step(self,*a):return lib.step(self.p,*a)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--dynamic-arms',action='store_true')
args=parser.parse_args()
s=(ROOT/'scripts/balance_sim.py').read_text()
def patch(a,b):
 global s
 assert s.count(a)==1,a
 s=s.replace(a,b)
patch('if capture_settled:\n                self.capture_was_settled','if capture_settled and False:\n                self.capture_was_settled')
patch('capture_shift = self.engage_capture_shift * capture_weight','''if self.arms_returned:
            self.fast_weight=0.
        elif self.arms_returning:
            self.fast_weight=min(getattr(self,'fast_weight',1.),clampf(1-(self.engage_arm_frac-arm_frac)/.1,0,1))
        capture_shift = self.engage_capture_shift * getattr(self,'fast_weight',1.)''')
patch('hard_stop_latched = False','support_released=False\n    native_drive=Drive()\n    hard_stop_latched = False')
patch('theta += theta_dot * INNER_DT','''theta += theta_dot * INNER_DT
        if not support_released and tip_true >= plant.release_fraction:
            theta=eq0+engage_offset_deg
            theta_dot=0.
        else: support_released=True''')
patch('cmd = c.kp * angle_err - c.kd * gyro_filt','''cmd = c.kp * angle_err - c.kd * gyro_filt
            if c.candidate in (1,2,4):
                active=outer.startup_active if c.candidate==1 else outer.startup_triggered
                cmd=native_drive.step(active,angle_err,gyro_raw,outer.filtered_wheel_vel,outer.last_target_vel,cmd,last_cmd,INNER_DT,cmd_max)''')
patch('        # Mirror balance_math::RecoilUnwind; fresh feedback supplied by model.','''        recovery_accel = c.candidate in (1,2,4) and (self.startup_active if c.candidate==1 else self.startup_triggered)
        # Mirror balance_math::RecoilUnwind; fresh feedback supplied by model.''')
patch('        if self.startup_active:\n            ki = (c.recovery_ki', '''        if recovery_accel:
            self.vel_sp_integral=clampf(self.vel_sp_integral+c.vel_sp_ki*vel_err*dt,-c.recovery_limit,c.recovery_limit)
        elif self.startup_active:
            ki = (c.recovery_ki''')
patch('        arm_dev = abs(self.arm_assist_frac - c.arm_assist_bias)','''        if recovery_accel: p_term=0.
        arm_dev = abs(self.arm_assist_frac - c.arm_assist_bias)''')
patch('        # --- arm assist lifecycle ---','''        if c.candidate in (3,4) and self.ramp_complete and self.startup_active and not getattr(self, 'startup_arm_ready',False):
            self.startup_arm_ready=True
            if self.arm_stage==3: self.arm_stage=0
        # --- arm assist lifecycle ---''')
if args.dynamic_arms:
 patch('motor_acceleration = 0.0','''motor_acceleration = 0.0
    arm_vl=arm_vr=0.0''')
 # Causal motor/arm plant: bounded accelerations and lag. Arm commands from
 # the previous outer tick produce this tick's motion/reaction, not an oracle.
 patch('        tip_true, cen_true = outer.axis_fractions(arm_l, arm_r)','''        arm_limit=c.arm_assist_speed if outer.ramp_complete else c.arm_return_speed
        want_l=clampf(80*(outer.arm_l_target-arm_l),-arm_limit,arm_limit)
        want_r=clampf(80*(outer.arm_r_target-arm_r),-arm_limit,arm_limit)
        al=clampf((want_l-arm_vl)/plant.arm_tau,-plant.arm_accel,plant.arm_accel)
        ar=clampf((want_r-arm_vr)/plant.arm_tau,-plant.arm_accel,plant.arm_accel)
        arm_vl+=al*INNER_DT;arm_vr+=ar*INNER_DT
        arm_l+=arm_vl*INNER_DT;arm_r+=arm_vr*INNER_DT
        tip_true, cen_true = outer.axis_fractions(arm_l, arm_r)''')
 patch('- plant.damping * theta_dot)','''- plant.damping * theta_dot
                      - plant.arm_inertia_ratio*57.2957795*(al-ar)*.5)''')
 patch('arm_l = move_toward(arm_l, outer.arm_l_target, ARM_ACT_SPEED, INNER_DT)',
       '# Arm dynamics already advanced above.')
 patch('arm_r = move_toward(arm_r, outer.arm_r_target, ARM_ACT_SPEED, INNER_DT)',
       '# Mirrored encoder axis provides the signed inertial reaction.')
m=types.ModuleType('screen_model');m.__file__=str(ROOT/'scripts/balance_sim.py');m.Drive=Drive;sys.modules[m.__name__]=m;exec(compile(s,m.__file__,'exec'),m.__dict__)
nominal=(23.,8.,.02,-3.4,87.15,81.965,.975)
cases=[nominal]+list(itertools.product((16.,23.,32.),(6.,8.),(.01,.035,.07,.15),(-1.9,-3.5),(85.7,86.5,87.5,88.7),(82.6,),(.99,.95)))
if args.dynamic_arms:
 cases=[(a,b,lag,-3.4,eq,81.965,.975,inertia,armtau,accel)
  for a,b,lag,eq,inertia,armtau,accel in itertools.product(
   (16.,23.,32.),(6.,8.),(.02,.07,.15),(85.7,87.15,88.7),
   (0.,.05,.15),(.01,.04),(30.,80.))]
policies=(0,3,4) if args.dynamic_arms else (0,1,2,3,4)
rows=[];traces={}
for policy in policies:
 cfg=m.current_firmware_config();cfg.candidate=policy;cfg.arm_assist_speed=lib.arm_speed()
 for i,params in enumerate(cases):
  a,b,lag,delta,eq,cap,release=params[:7]
  p=m.PlantParams(A=a,B=b,tau_m=lag,eq_fwd=eq,eq_tip_delta=delta,fb_hold_ticks=1,vel_meas_noise=.1,accel_noise_deg=.3,gyro_noise_dps=.3,process_accel_noise=.1);p.release_fraction=release
  if args.dynamic_arms:p.arm_inertia_ratio,p.arm_tau,p.arm_accel=params[7:]
  r=m.simulate(cfg,p,engage_offset_deg=cap-p.true_eq(1,0),engage_trim=3.2398,duration_s=15,seed=19)
  late=[j for j,t in enumerate(r.t) if t>=13]
  rms=(sum(r.wheel_vel[j]**2 for j in late)/len(late))**.5 if late else None
  rows.append(dict(policy=policy,case=i,params=cases[i],fell=r.fell,reason=r.abort_reason,rms=rms,peak=max(map(abs,r.wheel_vel)),path=sum(abs(y-x) for x,y in zip(r.drift,r.drift[1:])),final_position=r.final_drift,ramp=r.ramp_complete_s))
  if i==0:traces[str(policy)]=dict(t=r.t,angle=r.roll,eq=r.eq,target=r.setpoint,speed=r.wheel_vel,position=r.drift)
 rr=rows[-len(cases):];print(policy,len(rr),'no-fall',sum(not r['fell'] for r in rr),'quiet',sum(not r['fell'] and r['rms'] is not None and r['rms']<.7 for r in rr),'nominal',rr[0],flush=True)
summary={
 'kind':'unvalidated_dynamic_arm_sensitivity' if args.dynamic_arms else 'unvalidated_planar_sensitivity',
 'policies':{0:'installed v2',1:'acceleration during recovery',2:'acceleration latched after detection',
             3:'ordinary arms made ready once after return',4:'latched acceleration plus earlier arms'},
 'cases_per_policy':len(cases),
 'source_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
  for p in [ROOT/'scripts/balance_sim.py',ROOT/'src/balance_drive.h',ROOT/'src/config.h',OUT/'bridge.cpp',Path(__file__)]},
 'limits':['Exploratory plant is not a validated reconstruction of this failure.',
 'No-fall and low final wheel RMS do not establish physical stability.',
 'Model lacks slip, two-wheel asymmetry, full rigid-body geometry and contact impacts.',
 'Dynamic arm inertia ratio, lag and acceleration limits are hypotheses, not identified values.',
 'Mirrored encoder difference defines the hypothesized reaction sign; verify mechanically before feedforward.',
 'No candidate in this screen is approved for firmware integration or upload.'],
 'results':{}}
for index,policy in enumerate(policies):
 cand=rows[index*len(cases):(index+1)*len(cases)];baseline=rows[:len(cases)]
 summary['results'][policy]=dict(no_fall=sum(not r['fell'] for r in cand),
  final_speed_rms_below_point7=sum(not r['fell'] and r['rms'] is not None and r['rms']<.7 for r in cand),
  regressed=[n['case'] for o,n in zip(baseline,cand) if not o['fell'] and n['fell']],
  improved=[n['case'] for o,n in zip(baseline,cand) if o['fell'] and not n['fell']])
 print('policy',policy,'regressed',len(summary['results'][policy]['regressed']),
       'improved',len(summary['results'][policy]['improved']),flush=True)
prefix='dynamic' if args.dynamic_arms else 'planar'
(OUT/(prefix+'-screen.json')).write_text(json.dumps(dict(summary=summary,rows=rows),indent=2)+'\n')
(OUT/(prefix+'-traces.json')).write_text(json.dumps(traces,indent=2)+'\n')
