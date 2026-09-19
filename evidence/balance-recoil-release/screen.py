"""Compare final mirrored policy against the frozen two-success firmware model.

Plant uncertainty screening, not a physical reliability estimate. No contact,
mounting flex or slip model; firmware freshness guards are tested natively.
"""
import importlib.util
import itertools
import json
import statistics
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent
BASELINE = '75318bc7eac6b9df5a4bac68f4378ef1102b2838'
frozen = ROOT/'output/recoil-frozen'
for name in ('scripts/balance_sim.py', 'src/config.h'):
    path = frozen/name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(subprocess.check_output(['git', 'show', f'{BASELINE}:{name}'], cwd=ROOT))

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    class Observed(module.OuterController):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            module.last = self
            self.first_settle = None
        def tick(self, *a, **kw):
            was = self.startup_active
            super().tick(*a, **kw)
            if was and not self.startup_active and self.first_settle is None:
                self.first_settle = a[0]/1000
    module.OuterController = Observed
    return module

old = load('recoil_old', frozen/'scripts/balance_sim.py')
new = load('recoil_new', ROOT/'scripts/balance_sim.py')
results = []
baseline_runs = []
for label, module in (('two-success baseline', old), ('confirmed recoil release v1', new)):
    cfg = module.current_firmware_config()
    plant = module.load_fitted_params(ROOT/'evidence/balance-review/model-fit-speed.json')
    cases = []
    scenarios = itertools.product((-1.9, -3.5, -4.5), (.7, 1., 1.4), (.7, 1., 1.4),
                                  (.01, .03, .06), (-.8, .8), (1, 2))
    for idx, (delta, af, bf, tau, offset, seed) in enumerate(scenarios):
        p = replace(plant, A=plant.A*af, B=plant.B*.66*bf, tau_m=tau, eq_tip_delta=delta,
                    feedback_velocity_scale=1., fb_hold_ticks=1, vel_meas_noise=.1)
        r = module.simulate(cfg, p, engage_offset_deg=offset, duration_s=16, seed=seed)
        if module is old:
            baseline_runs.append(r)
        else:
            # No changed control output before the arms/base ramp completes.
            before = baseline_runs[idx]
            until = min(before.ramp_complete_s or 16, r.ramp_complete_s or 16)
            n = sum(t < until for t in before.t)
            assert before.cmd[:n] == r.cmd[:n], (idx, 'initial catch changed')
            assert before.setpoint[:n] == r.setpoint[:n], (idx, 'initial target changed')
        early = r.fell and (r.ramp_complete_s is None or r.t[-1] < r.ramp_complete_s+3)
        cases.append(dict(delta=delta, af=af, bf=bf, tau=tau, offset=offset, seed=seed,
                          fell=r.fell, early=early, late=r.fell and not early,
                          settle=module.last.first_settle, peak=abs(r.peak_drift_standup)))
    valid = [c['settle'] for c in cases if not c['fell'] and c['settle'] is not None]
    summary = dict(label=label, early=sum(c['early'] for c in cases),
                   late=sum(c['late'] for c in cases), survived_and_settled=len(valid),
                   median_settle=statistics.median(valid) if valid else None)
    if results:
        pairs = list(zip(results[0]['cases'], cases))
        summary['new_failures'] = sum(not a['fell'] and b['fell'] for a,b in pairs)
        summary['rescues'] = sum(a['fell'] and not b['fell'] for a,b in pairs)
        matched = [b['settle']-a['settle'] for a,b in pairs if not a['fell'] and not b['fell']
                   and a['settle'] is not None and b['settle'] is not None]
        summary['paired_settle_cases'] = len(matched)
        summary['paired_median_settle_delta'] = statistics.median(matched) if matched else None
        summary['initial_catch_identical_cases'] = len(cases)
    print(json.dumps(summary), flush=True)
    results.append(dict(summary=summary, cases=cases))
(OUT/'screen.json').write_text(json.dumps(results, indent=2)+'\n')
