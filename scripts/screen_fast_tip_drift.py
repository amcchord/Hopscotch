#!/usr/bin/env python3
"""Paired fast recovery sensitivity model; not a physical acceptance test.

The actual C++ gain selection, integrator and recoil helper own recovery in
both policies. The rest of the existing planar simulator is an approximation.
Initial arm contact is an idealized support constraint, not fitted contact.
"""
import argparse
import ctypes
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    build = ROOT/'output/fast-drift-native.so'
    subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                    '-Isrc', 'scripts/fast_tip_recovery_bridge.cpp', '-o', str(build)], cwd=ROOT, check=True)
    lib = ctypes.CDLL(str(build))
    lib.recovery_create.restype = ctypes.c_void_p
    lib.recovery_destroy.argtypes = [ctypes.c_void_p]
    lib.recovery_step.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool] + [ctypes.c_float]*5
    lib.recovery_step.restype = ctypes.c_float

    class NativeRecovery:
        def __init__(self): self.ptr = lib.recovery_create()
        def __del__(self): lib.recovery_destroy(self.ptr)
        def step(self, *values): return lib.recovery_step(self.ptr, *values)

    source_path = ROOT/'scripts/balance_sim.py'
    source = source_path.read_text()

    def patch(old, new):
        nonlocal source
        assert source.count(old) == 1, f'Simulator changed: {old}'
        source = source.replace(old, new)

    patch('if capture_settled:\n                self.capture_was_settled',
          'if capture_settled and False:  # both policies preserve fast v2 trim\n                self.capture_was_settled')
    patch('capture_shift = self.engage_capture_shift * capture_weight', '''if self.arms_returned:
            self.fast_weight=0.
        elif self.arms_returning:
            self.fast_weight=min(getattr(self,'fast_weight',1.),clampf(1-(self.engage_arm_frac-arm_frac)/.1,0,1))
        capture_shift = self.engage_capture_shift * getattr(self,'fast_weight',1.)''')
    patch('hard_stop_latched = False', 'support_released=False\n    hard_stop_latched = False')
    patch('theta += theta_dot * INNER_DT', '''theta += theta_dot * INNER_DT
        if not support_released and tip_true >= plant.release_fraction:
            theta=eq0+engage_offset_deg
            theta_dot=0.
        else: support_released=True''')
    start = source.index('        # Mirror balance_math::RecoilUnwind;')
    end = source.index('        elif self.ramp_complete:', start)
    source = source[:start] + '''        if self.startup_active:
            if not hasattr(self,'native_recovery'): self.native_recovery=NativeRecovery()
            self.vel_sp_integral=self.native_recovery.step(c.candidate,
                not self.ramp_complete and now_ms-self.startup_ms<c.recovery_boost_ms,
                self.ramp_complete,vel_err,self.vel_sp_integral,self.sp_offset,c.vel_sp_ki,dt)
''' + source[end:]
    module = types.ModuleType('fast_drift_screen_model')
    module.__file__ = str(source_path)
    module.NativeRecovery = NativeRecovery
    sys.modules[module.__name__] = module
    exec(compile(source, str(source_path), 'exec'), module.__dict__)

    # Nominal dynamics are rounded exploratory fits to recent healthy motor
    # runs. Broad cases deliberately include equilibrium errors and motor lag
    # outside the observed successful range. Do not hide their regressions.
    nominal = (23., 8., .02, -3.4, 87.15, 82.6, .975)
    broad = list(itertools.product((16.,23.,32.), (6.,8.), (.01,.035,.07,.15),
                                  (-1.9,-3.5), (85.7,86.5,87.5,88.7), (82.6,), (.99,.95)))
    cases = [nominal] + broad
    rows, traces = [], {}
    for candidate in (False, True):
        cfg = module.current_firmware_config()
        cfg.candidate = candidate
        for index, (a,b,lag,delta,eq,cap,release) in enumerate(cases):
            plant = module.PlantParams(A=a, B=b, tau_m=lag, eq_fwd=eq, eq_tip_delta=delta,
                fb_hold_ticks=1, vel_meas_noise=.1, accel_noise_deg=.3, gyro_noise_dps=.3, process_accel_noise=.1)
            plant.release_fraction = release
            r = module.simulate(cfg, plant, engage_offset_deg=cap-plant.true_eq(1,0),
                                engage_trim=3.2398, duration_s=12, seed=19)
            late = [i for i,t in enumerate(r.t) if t>=10]
            late_rms = (sum(r.wheel_vel[i]**2 for i in late)/len(late))**.5 if late else None
            rows.append(dict(candidate=candidate, case=index, parameters=cases[index],
                fell=r.fell, reason=r.abort_reason, peak_speed_rad_s=r.peak_vel_standup,
                peak_drift_rad=r.peak_drift_standup, final_drift_rad=r.final_drift,
                total_wheel_path_rad=sum(abs(b-a) for a,b in zip(r.drift,r.drift[1:])),
                ramp_complete_s=r.ramp_complete_s, final_two_seconds_speed_rms=late_rms))
            if index == 0:
                traces['candidate' if candidate else 'baseline'] = dict(t=r.t, angle=r.roll,
                    target=r.setpoint, wheel_speed=r.wheel_vel, wheel_position=r.drift)
        print(('candidate' if candidate else 'baseline')+' completed', len(cases), flush=True)
    baseline, candidate = rows[:len(cases)], rows[len(cases):]
    pairs = [(o,n) for o,n in zip(baseline,candidate) if not o['fell'] and not n['fell']]
    summary = dict(kind='unvalidated_planar_sensitivity', cases_per_policy=len(cases),
        simulator_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
        bridge_sha256=hashlib.sha256((ROOT/'scripts/fast_tip_recovery_bridge.cpp').read_bytes()).hexdigest(),
        baseline_no_fall=sum(not r['fell'] for r in baseline),
        candidate_no_fall=sum(not r['fell'] for r in candidate),
        improved_to_no_fall=[n['case'] for o,n in zip(baseline,candidate) if o['fell'] and not n['fell']],
        regressed_to_fall=[n['case'] for o,n in zip(baseline,candidate) if not o['fell'] and n['fell']],
        both_no_fall=len(pairs), reduced_total_path=sum(n['total_wheel_path_rad']<o['total_wheel_path_rad'] for o,n in pairs),
        nominal=dict(baseline=baseline[0],candidate=candidate[0]),
        limits=['No-fall does not establish calm balance or physical safety.',
                'Broad sensitivity includes regressions; this is not an across-the-board robustness improvement.',
                'Idealized support, arm geometry and planar motor dynamics omit slip, asymmetry and contact impacts.',
                'Physical drift reduction remains unmeasured.'])
    (args.output/'screen.json').write_text(json.dumps(dict(summary=summary, rows=rows),indent=2)+'\n')
    (args.output/'nominal-traces.json').write_text(json.dumps(traces,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__': main()
