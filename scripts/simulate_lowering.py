#!/usr/bin/env python3
"""Offline forward-fall/catch screen using the production C++ maneuver policy.

Unlike the failed v1 screen, wheel acceleration, unstable body gravity, arm
servo lag and unilateral floor forces act during EVERY phase. Preparation uses
an explicit 200 Hz PD / 50 Hz velocity-PI approximation of the existing idle
balance controller (not a full scheduler/firmware emulation). Geometry, wide
angle COM, joint stiffness and contact forces remain unmeasured assumptions.
No screen result establishes a safe physical catch or structural impact load.
"""
import argparse
import csv
import ctypes as C
from dataclasses import dataclass, asdict, replace
import itertools
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
DT = .02
RAD = math.pi/180
PHASES = ['idle','stopping','reaching','descending','ground_hold','retracting',
          'canceling','complete','fault','committing','catching']


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class Model:
    pivot: float = .12
    arm: float = .30
    wheel_radius: float = .055
    equilibrium: float = 86.63
    arm_eq_slope: float = -6.3
    nonlinear_com: float = 0.0
    arm_inertia_ratio: float = .03
    arm_joint_inertia: float = .004
    gravity: float = 8.82
    wheel_coupling: float = 6.0
    wheel_tau: float = .06
    stiffness: float = 240
    damping_ratio: float = .8
    servo_tau: float = .04
    servo_limit: float = 1.5
    joint_stiffness: float = 25
    torque_per_acceleration: float = .004
    center_left: float = 1.768
    center_right: float = -1.767
    initial_speed: float = 0
    floor: bool = True
    right_contact: bool = True
    stale_at: float = 1e9
    stale_phase: int = -1
    slip_at: float = 1e9
    slip_after_catch: float = 1e9
    jam: bool = False
    launch_sign: float = 1
    false_load: bool = False


class Policy:
    def __init__(self):
        out = ROOT/'output'
        out.mkdir(exist_ok=True)
        path = out/'lowering_bridge.so'
        subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror',
                        '-shared','-fPIC','-I'+str(ROOT/'src'),
                        str(ROOT/'scripts/lowering_bridge.cpp'),'-o',str(path)],check=True)
        self.lib = C.CDLL(str(path))
        self.lib.lower_new.restype = C.c_void_p
        self.lib.lower_delete.argtypes = [C.c_void_p]
        self.lib.lower_request.argtypes = [C.c_void_p,C.c_uint32,C.POINTER(C.c_float),C.c_bool,C.c_float,C.c_float]
        self.lib.lower_request.restype = C.c_bool
        self.lib.lower_step.argtypes = [C.c_void_p,C.c_uint32,C.c_float,C.POINTER(C.c_float),C.c_bool,C.POINTER(C.c_float)]
        self.lib.lower_reason.argtypes = [C.c_void_p]
        self.lib.lower_reason.restype = C.c_char_p


def support_angle(q, m):
    a, b = m.pivot+m.arm*math.cos(q), m.arm*math.sin(q)
    radius = math.hypot(a,b)
    if radius <= m.wheel_radius:
        return None
    return (-math.asin(m.wheel_radius/radius)-math.atan2(b,a))/RAD


def simulate(policy,m,name):
    handle = policy.lib.lower_new()
    theta,rate,filtered_rate,catch_rate = m.equilibrium,0.,0.,0.
    arms,velocities,torques = [0.,0.],[0.,0.],[0.,0.]
    contact_torque = [0.,0.]
    wheel,position,filtered_wheel = m.initial_speed,0.,m.initial_speed
    base,integral,offset,sp = m.equilibrium,0.,0.,m.equilibrium
    command = 0.
    old_q_rate = 0.
    joint_rates = [0.,0.]
    centers = [m.center_left,m.center_right]
    signs = [math.copysign(1,c) for c in centers]
    output = (C.c_float*7)()
    trace,transitions = [],[]
    previous_phase = None
    phase = 1
    committed = supported = False
    contact_time = commit_time = first_flat = None
    peak_rate = peak_lead = peak_wheel = peak_torque = 0.
    values = lambda: (C.c_float*11)(theta,catch_rate,sp-theta,wheel,wheel,*arms,*velocities,*torques)
    assert policy.lib.lower_request(handle,0,values(),True,*centers)
    try:
        for tick in range(2500):
            t = tick*DT
            policy.lib.lower_step(handle,round(t*1000),DT,values(),t<m.stale_at and phase!=m.stale_phase,output)
            targets = [output[0],output[1]]
            phase,supported,override = int(output[2]),bool(output[3]),bool(output[4])
            wheel_request,committed = output[5],bool(output[6])
            if committed and commit_time is None: commit_time=t
            if supported and contact_time is None: contact_time=t
            if phase != previous_phase:
                transitions.append(dict(time_s=round(t,3),phase=PHASES[phase],tilt_deg=round(theta,3)))
                previous_phase=phase
            trace.append([t,theta,rate,*arms,*targets,*torques,phase,float(committed),wheel,sp,wheel_request,float(supported)])
            if phase in (7,8): break
            peak_lead=max(peak_lead,*(abs(a-b) for a,b in zip(arms,targets)))
            if not committed:
                # Ordinary stationary cascade with its recorded defaults. No
                # assumed-stable preparation or artificial tilt servo.
                fraction=sum(a/c for a,c in zip(arms,centers))/2
                desired_base=m.equilibrium-6.3*fraction
                base += clamp(desired_base-base,-4*DT,4*DT)
                filtered_wheel += .35*(wheel-filtered_wheel)
                target_vel=clamp(-.0758*position,-1.5152,1.5152)
                error=filtered_wheel-target_vel
                gate=clamp(1-abs(sp-theta)/8,0,1)
                ki=.231*(4 if abs(filtered_rate)<10 and abs(command)<3 and abs(error)>1.21 else 1)
                integral=clamp(integral+ki*error*gate*DT,-8,8)
                p=math.copysign(.462*min(abs(error),1.2121)+1.452*max(0,abs(error)-1.2121),error)
                shed=clamp(1-(abs(filtered_wheel)-12.12)/(21.21-12.12),0,1)
                desired_offset=clamp(p*gate*shed+integral,-8,8)
                offset+=clamp(desired_offset-offset,-12*DT,12*DT)
                sp=clamp(base+offset,70,110)
            old_arms=arms[:]
            for substep in range(8):
                dt=DT/8
                if substep%2==0:
                    command=wheel_request*m.launch_sign if committed else clamp(2*(sp-theta)-.08*filtered_rate,-30,30)
                acceleration=clamp((command-wheel)/m.wheel_tau,-100,100)
                wheel+=acceleration*dt
                position+=wheel*dt
                prior_arms=arms[:]
                for j in range(2):
                    target=targets[j] if override else 0
                    # Loaded joint deflects away from the floor; no hard clamp
                    # of the arm and no omission of body reaction in preparation.
                    deflection=signs[j]*contact_torque[j]/m.joint_stiffness
                    servo=clamp((target+deflection-arms[j])/m.servo_tau,-m.servo_limit,m.servo_limit)
                    arms[j]+=servo*dt
                    if m.jam and arms[j]*signs[j]<-1.6: arms[j]=-1.6*signs[j]
                rates=[(a-b)/dt for a,b in zip(arms,prior_arms)]
                joint_accels=[(a-b)/dt for a,b in zip(rates,joint_rates)]
                joint_rates=rates
                q_rate=sum(v*s for v,s in zip(rates,signs))/2
                q_accel=(q_rate-old_q_rate)/dt
                old_q_rate=q_rate
                q=sum(a*s for a,s in zip(arms,signs))/2
                physical_q=(1-m.nonlinear_com)*q+m.nonlinear_com*math.sin(q)
                eq=m.equilibrium+m.arm_eq_slope*physical_q/1.7675
                body_acceleration=(m.gravity*math.sin((theta-eq)*RAD)/RAD
                    +m.wheel_coupling*acceleration-m.arm_inertia_ratio*q_accel/RAD)
                ground_present=m.floor and t<m.slip_at and (contact_time is None or t-contact_time<m.slip_after_catch)
                contact_torque=[0.,0.]
                for j in range(2):
                    support=support_angle(arms[j]*signs[j],m)
                    if ground_present and (j==0 or m.right_contact) and support is not None and support>theta:
                        force=max(0,m.stiffness*(support-theta)-2*m.damping_ratio*math.sqrt(m.stiffness)*rate)/2
                        # Finite arm-force authority. This is a model assumption,
                        # not a measured hardware torque/impact rating.
                        force=min(force,3/m.torque_per_acceleration)
                        body_acceleration+=force
                        contact_torque[j]=force*m.torque_per_acceleration
                    free=.15*abs(math.sin(arms[j]*signs[j]))
                    jam_load=.8 if m.jam and arms[j]*signs[j]<=-1.599 else 0
                    torques[j]=(signs[j]*(contact_torque[j]+free+jam_load+(1 if m.false_load else 0))
                                +m.arm_joint_inertia*joint_accels[j])
                if theta<=0 and body_acceleration<0:
                    body_acceleration=0;rate=max(0,rate)
                rate+=body_acceleration*dt
                theta+=rate*dt
                if theta<0:theta,rate=0.,max(0,rate)
                filtered_rate += (1-math.exp(-dt/.04))*(rate-filtered_rate)
                catch_rate += (1-math.exp(-dt/.006))*(rate-catch_rate)
                peak_rate=max(peak_rate,abs(rate))
                peak_wheel=max(peak_wheel,abs(wheel))
                peak_torque=max(peak_torque,*map(abs,torques))
            velocities=[(a-b)/DT for a,b in zip(arms,old_arms)]
            if theta<=5 and committed and first_flat is None:first_flat=t
        result=dict(name=name,model=asdict(m),outcome=policy.lib.lower_reason(handle).decode(),
                    final_phase=PHASES[phase],commit_s=commit_time,catch_s=contact_time,
                    first_flat_s=first_flat,elapsed_s=round(t,3),peak_rate_dps=round(peak_rate,3),
                    peak_wheel_rad_s=round(peak_wheel,3),peak_model_torque_nm=round(peak_torque,3),
                    peak_target_lead_rad=round(peak_lead,5),final_tilt_deg=round(theta,3),transitions=transitions)
        return result,trace
    finally:policy.lib.lower_delete(handle)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/lowering-catch')
    parser.add_argument('--nominal-only',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    policy=Policy();base=Model()
    cases=[('nominal',base)]
    if not args.nominal_only:
        cases += [('no_floor',replace(base,floor=False)),('one_arm_only',replace(base,right_contact=False)),
                  ('stale_feedback',replace(base,stale_at=5)),
                  ('stale_in_fall',replace(base,stale_phase=10)),
                  ('stale_supported',replace(base,stale_phase=3)),('no_reach',replace(base,pivot=.23,arm=.26)),
                  ('slow_servo',replace(base,servo_limit=.06)),('contact_loss',replace(base,slip_after_catch=1)),
                  ('both_arms_obstructed',replace(base,floor=False,jam=True)),
                  ('wrong_wheel_sign',replace(base,launch_sign=-1)),
                  ('false_load_no_floor',replace(base,floor=False,false_load=True)),
                  ('high_inertia_limit',replace(base,arm_inertia_ratio=.1))]
        for geometry,plant,stiffness,tau,com,inertia in itertools.product(
                [(.08,.34),(.12,.30),(.16,.28)],[(6,4),(8.82,6),(12,9)],
                [160,320],[.03,.08],[0,1],[0,.03,.06]):
            cases.append((f'sweep_{len(cases)}',replace(base,pivot=geometry[0],arm=geometry[1],
                gravity=plant[0],wheel_coupling=plant[1],stiffness=stiffness,servo_tau=tau,nonlinear_com=com,arm_inertia_ratio=inertia)))
    results=[]
    for name,model in cases:
        result,trace=simulate(policy,model,name);results.append(result)
        if not name.startswith('sweep_'):
            with (args.output/(name+'.csv')).open('w') as stream:
                writer=csv.writer(stream);writer.writerow(['time_s','tilt_deg','rate_dps','arm_l','arm_r',
                    'target_l','target_r','torque_l','torque_r','phase','committed','wheel_rad_s','setpoint',
                    'wheel_request','supported']);writer.writerows(trace)
        print(f"{name}: {result['outcome']}; peak {result['peak_rate_dps']:.2f} deg/s; tilt {result['final_tilt_deg']:.2f}")
    (args.output/'results.json').write_text(json.dumps({'model_scope':__doc__,'results':results},indent=2)+'\n')
    print(json.dumps(results[0],indent=2))
    assert results[0]['outcome']=='lower_complete',results[0]
    for r in results:
        assert r['peak_target_lead_rad']<=.1201,r
        if r['name'] not in ('nominal','high_inertia_limit') and not r['name'].startswith('sweep_'):
            assert r['outcome']!='lower_complete',r


if __name__=='__main__':main()
