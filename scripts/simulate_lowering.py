#!/usr/bin/env python3
"""Screen the production stand-down policy against a planar arm/floor model.

The actual C++ policy runs through a tiny native bridge. This model assumes
the existing wheel balance controller can maintain near-equilibrium balance
during preparation; it does NOT validate that assumption or replace the
standing-drive simulations. After contact, gravity, arm servo lag, unilateral
floor support, compliance and damping determine body motion. Dimensions and
contact stiffness are assumptions swept here, not measured robot geometry.
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
RAD = math.pi / 180
PHASES = ['idle', 'stopping', 'reaching', 'descending', 'ground_hold',
          'retracting', 'canceling', 'complete', 'fault', 'loading']


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


@dataclass
class Model:
    pivot: float = .12       # m ahead of rear axle, assumed
    arm: float = .30         # m, assumed
    wheel_radius: float = .055
    equilibrium: float = 87.5
    arm_eq_slope: float = -6.3  # historical measured small-angle slope, extrapolated
    gravity: float = 8.82    # fitted upright 1/s^2; extended with sin away from upright
    stiffness: float = 240  # support angular stiffness, 1/s^2 (unmeasured)
    damping_ratio: float = .8
    servo_tau: float = .08
    servo_limit: float = .30
    center_left: float = 1.77
    center_right: float = -1.77
    initial_speed: float = 0
    floor: bool = True
    right_contact: bool = True
    stale_at: float = 1e9
    slip_at: float = 1e9
    jam: bool = False


class Policy:
    def __init__(self):
        path = ROOT / 'output' / 'lowering_bridge.so'
        subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-shared', '-fPIC', '-I' + str(ROOT / 'src'),
                        str(ROOT / 'scripts/lowering_bridge.cpp'), '-o', str(path)], check=True)
        self.lib = C.CDLL(str(path))
        self.lib.lower_new.restype = C.c_void_p
        self.lib.lower_delete.argtypes = [C.c_void_p]
        self.lib.lower_request.argtypes = [C.c_void_p, C.c_uint32, C.POINTER(C.c_float), C.c_bool, C.c_float, C.c_float]
        self.lib.lower_request.restype = C.c_bool
        self.lib.lower_step.argtypes = [C.c_void_p, C.c_uint32, C.c_float, C.POINTER(C.c_float), C.c_bool, C.POINTER(C.c_float)]
        self.lib.lower_reason.argtypes = [C.c_void_p]
        self.lib.lower_reason.restype = C.c_char_p


def support_angle(q, m):
    a = m.pivot + m.arm * math.cos(q)
    b = m.arm * math.sin(q)
    radius = math.hypot(a, b)
    if radius <= m.wheel_radius:
        return None
    return (-math.asin(m.wheel_radius / radius) - math.atan2(b, a)) / RAD


def contact_arm_angle(tilt, m):
    height = m.wheel_radius + m.pivot * math.sin(tilt * RAD)
    if height >= m.arm:
        return -math.pi
    return -tilt * RAD - math.asin(height / m.arm)


def simulate(policy, m, name):
    handle = policy.lib.lower_new()
    theta, rate = m.equilibrium, 0.
    arms, velocities, torques = [0., 0.], [0., 0.], [0., 0.]
    wheel = m.initial_speed
    centers = [m.center_left, m.center_right]
    signs = [1 if c > 0 else -1 for c in centers]
    phase, committed = 1, False
    output = (C.c_float * 6)()
    lean = 0.
    trace, transitions = [], []
    previous_phase = None
    contact_time = None
    first_flat = None
    peak_rate = peak_lead = 0.
    values = lambda error: (C.c_float * 11)(theta, rate, error, wheel, wheel,
                                         *arms, *velocities, *torques)
    assert policy.lib.lower_request(handle, 0, values(0), True, *centers)
    try:
        for tick in range(2500):
            t = tick * DT
            fraction = sum(a / c for a, c in zip(arms, centers)) / 2
            equilibrium = m.equilibrium + m.arm_eq_slope * fraction
            policy.lib.lower_step(handle, round(t * 1000), DT, values(equilibrium-theta),
                                  t < m.stale_at, output)
            targets = [output[0], output[1]]
            phase, committed, override = int(output[2]), bool(output[3]), bool(output[4])
            lean += clamp(output[5]-lean, -2*DT, 2*DT)
            if phase != previous_phase:
                transitions.append({'time_s': round(t, 3), 'phase': PHASES[phase], 'tilt_deg': round(theta, 3)})
                previous_phase = phase
            if committed and contact_time is None:
                contact_time = t
            trace.append([t, theta, rate, *arms, *targets, *torques, phase, float(committed), wheel])
            if phase in (7, 8):
                break
            peak_lead = max(peak_lead, *(abs(a-b) for a, b in zip(arms, targets)))
            old_arms = arms[:]
            for _ in range(8):
                dt = DT / 8
                # Rear zero-speed servo during supported lowering. Ordinary
                # stopping here is an explicit simplifying model, not a claim
                # about stopping distances of the physical drive controller.
                wheel += clamp(-wheel, -8*dt, 8*dt)
                q_contact = contact_arm_angle(theta, m)
                ground_present = m.floor and t < m.slip_at
                torques = [0., 0.]
                for j in range(2):
                    target = targets[j] if override else 0
                    candidate = arms[j] + clamp((target-arms[j]) / m.servo_tau,
                                                 -m.servo_limit, m.servo_limit) * dt
                    q = candidate * signs[j]
                    can_contact = ground_present and (j == 0 or m.right_contact)
                    floor_contact = can_contact and q < q_contact
                    obstacle = m.jam and q < -1.9
                    if floor_contact and not committed:
                        q = q_contact
                    if obstacle:
                        q = -1.9
                    arms[j] = q * signs[j]
                    # A stopped loaded servo resists a target beyond contact.
                    # Free-arm gravity torque alone is insufficient (no stall).
                    if floor_contact or obstacle:
                        torques[j] = signs[j] * (.65 + 5*abs(target-arms[j]))
                    else:
                        torques[j] = signs[j] * .18 * abs(math.sin(q))
                fraction = sum(a/c for a, c in zip(arms, centers)) / 2
                equilibrium = m.equilibrium + m.arm_eq_slope * fraction
                if not committed:
                    # The balanced-preparation assumption is deliberately
                    # visible: exact PD/drive dynamics are tested separately.
                    acceleration = 24*(equilibrium+lean-theta) - 10*rate
                else:
                    acceleration = m.gravity * math.sin((theta-equilibrium)*RAD) / RAD
                    if ground_present:
                        supports = [support_angle(arms[0]*signs[0], m)]
                        if m.right_contact:
                            supports.append(support_angle(arms[1]*signs[1], m))
                        support = max([s for s in supports if s is not None] + [0])
                        penetration = support - theta
                        if penetration > 0:
                            # Contact can push up; it cannot pull the body down.
                            acceleration += max(0, m.stiffness*penetration
                                - 2*m.damping_ratio*math.sqrt(m.stiffness)*rate)
                    # Front wheels support the body once it is flat.
                    if theta <= 0 and acceleration < 0:
                        acceleration = 0
                        rate = max(0, rate)
                rate += acceleration*dt
                theta += rate*dt
                if theta < 0:
                    theta, rate = 0., max(0, rate)
                peak_rate = max(peak_rate, abs(rate))
            velocities = [(a-b)/DT for a, b in zip(arms, old_arms)]
            if theta <= 8 and first_flat is None:
                first_flat = t
        reason = policy.lib.lower_reason(handle).decode()
        result = {'name': name, 'model': asdict(m), 'outcome': reason,
                  'final_phase': PHASES[phase], 'contact_s': contact_time,
                  'first_flat_s': first_flat, 'elapsed_s': round(t, 3),
                  'peak_rate_dps': round(peak_rate, 3), 'peak_target_lead_rad': round(peak_lead, 5),
                  'final_tilt_deg': round(theta, 3), 'transitions': transitions}
        return result, trace
    finally:
        policy.lib.lower_delete(handle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'output/lowering')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (ROOT/'output').mkdir(exist_ok=True)
    policy = Policy()
    baseline = Model()
    cases = [('nominal', baseline), ('no_floor', replace(baseline, floor=False)),
             ('one_arm_only', replace(baseline, right_contact=False)),
             ('stale_feedback', replace(baseline, stale_at=3)),
             ('no_reach', replace(baseline, pivot=.23, arm=.26)),
             ('slow_servo', replace(baseline, servo_limit=.06)),
             ('contact_loss', replace(baseline, slip_at=14)),
             ('both_arms_obstructed', replace(baseline, floor=False, jam=True)),
             ('moving_entry', replace(baseline, initial_speed=20))]
    for pivot, arm, stiffness, servo_tau, body in itertools.product([.08,.12,.16],[.28,.34],[160,320],[.04,.16],[(6,84),(8.82,87.5),(12,92)]):
        cases.append((f'sweep_{len(cases)}', replace(baseline, pivot=pivot, arm=arm,
                                                   stiffness=stiffness, servo_tau=servo_tau,
                                                   gravity=body[0], equilibrium=body[1])))
    results = []
    for name, model in cases:
        result, trace = simulate(policy, model, name)
        results.append(result)
        if not name.startswith('sweep_'):
            with (args.output/(name+'.csv')).open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['time_s','tilt_deg','rate_dps','arm_l','arm_r','target_l','target_r',
                                 'torque_l','torque_r','phase','supported','wheel_rad_s'])
                writer.writerows(trace)
        print(f"{name}: {result['outcome']}, peak {result['peak_rate_dps']:.2f} deg/s, final {result['final_tilt_deg']:.2f} deg")
    summary = {'model_scope': __doc__, 'results': results}
    (args.output/'results.json').write_text(json.dumps(summary, indent=2)+'\n')
    # Nominal contact geometry must complete; fault injections must not claim a
    # landing. All modeled commands must remain within the production lead cap.
    assert results[0]['outcome'] == 'lower_complete', results[0]
    for result in results:
        assert result['peak_target_lead_rad'] <= .1201, result
    for name in ['no_floor','one_arm_only','stale_feedback','no_reach','slow_servo','contact_loss','both_arms_obstructed']:
        assert next(r for r in results if r['name']==name)['outcome'] != 'lower_complete', name


if __name__ == '__main__':
    main()
