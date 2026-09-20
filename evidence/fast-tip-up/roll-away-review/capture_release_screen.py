#!/usr/bin/env python3
"""Paired sensitivity screen, NOT a fitted replay of the physical fast trial.

Loads the established planar simulator without altering its normal callers.
Mirrors the bounded fast reference policy and adds an explicitly idealized
initial arm support constraint. Native tests exercise the actual C++ policy.
"""
import hashlib
import json
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
source_path = ROOT/'scripts/balance_sim.py'
source = source_path.read_text()


def replace_once(old, new):
    global source
    assert source.count(old) == 1, f'Simulator changed: {old}'
    source = source.replace(old, new)


replace_once('if capture_settled:\n                self.capture_was_settled',
             'if capture_settled and not getattr(c, "supported_fast_capture", False):\n                self.capture_was_settled')
replace_once('capture_shift = self.engage_capture_shift * capture_weight', '''if getattr(c, 'supported_fast_capture', False):
            if self.arms_returned:
                self.fast_release_weight = 0.0
            elif self.arms_returning:
                progress = (self.engage_arm_frac-arm_frac)/0.1
                self.fast_release_weight = min(getattr(self, 'fast_release_weight', 1.),
                                               clampf(1-progress, 0., 1.))
            capture_weight = getattr(self, 'fast_release_weight', 1.)
        capture_shift = self.engage_capture_shift * capture_weight''')
replace_once('hard_stop_latched = False', 'support_released = False\n    hard_stop_latched = False')
replace_once('theta += theta_dot * INNER_DT', '''theta += theta_dot * INNER_DT
        if not support_released and tip_true >= plant.release_fraction:
            theta = eq0 + engage_offset_deg
            theta_dot = 0.0
        else:
            support_released = True''')
module = types.ModuleType('fast_release_screen')
module.__file__ = str(source_path)
sys.modules[module.__name__] = module
exec(compile(source, str(source_path), 'exec'), module.__dict__)

rows = []
for candidate in [False, True]:
    cfg = module.current_firmware_config()
    cfg.supported_fast_capture = candidate
    for eq_error in [-3, -1.5, 0, 1.5, 3]:
        for release in [.99, .975, .95]:
            for lag in [.08, .2, .35]:
                plant = module.PlantParams(eq_fwd=84+3.2398+eq_error,
                                           tau_m=lag, fb_hold_ticks=1)
                plant.release_fraction = release
                result = module.simulate(cfg, plant,
                    engage_offset_deg=82.458-plant.true_eq(1, 0), engage_trim=3.2398,
                    duration_s=12, seed=19)
                rows.append(dict(candidate=candidate, equilibrium_error_deg=eq_error,
                    release_fraction=release, motor_lag_s=lag, fell=result.fell,
                    peak_speed_rad_s=result.peak_vel_standup,
                    ramp_complete_s=result.ramp_complete_s,
                    final_drift_rad=result.final_drift, reason=result.abort_reason))
baseline, candidate = rows[:45], rows[45:]
summary = dict(kind='unvalidated_planar_support_sensitivity',
    simulator_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
    production_policy_commit='f4d2bb7240afdf6bd47807208b18f720522329fb',
    cases_per_policy=45,
    baseline_no_fall=sum(not r['fell'] for r in baseline),
    candidate_no_fall=sum(not r['fell'] for r in candidate),
    improve=[new for old, new in zip(baseline, candidate) if old['fell'] and not new['fell']],
    regress=[new for old, new in zip(baseline, candidate) if not old['fell'] and new['fell']],
    limits=['No-fall does not mean calm balance or small travel.',
            'Support constraint, true equilibrium and motor lag are assumptions, not fitted fast-trial dynamics.',
            'Planar model omits observed left/right wheel asymmetry, tire slip and contact reaction transients.',
            'Regressions when saved trim exceeds true equilibrium must remain visible.'], rows=rows)
(OUT/'capture-release-screen.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps({k: v for k, v in summary.items() if k not in ['rows', 'improve', 'regress']}, indent=2))
print(f"Improved {len(summary['improve'])}; regressed {len(summary['regress'])}")
