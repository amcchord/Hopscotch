#!/usr/bin/env python3
"""Compare the candidate with July's controller on an approximate planar plant."""
import json
import statistics
from dataclasses import asdict, replace
from pathlib import Path
import balance_sim as sim

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/balance-review'
plant = sim.load_fitted_params(OUT / 'model-fit-speed.json')
results = []
for name in ('july33', 'current'):
    cfg = sim.make_variant(name)
    cases = []
    for fa in (0.7, 1, 1.4):
        for delta in (-1, -1.9, -3.3, -4.5):
            for off in (-0.8, 0, 0.8):
                for seed in (1, 2):
                    r = sim.simulate(cfg, replace(plant, A=plant.A*fa, eq_tip_delta=delta),
                                     engage_offset_deg=off, duration_s=16, seed=seed)
                    early = r.fell and (r.ramp_complete_s is None or r.t[-1] < r.ramp_complete_s + 3)
                    cases.append(dict(A_factor=fa, equilibrium_shift=delta, capture_offset=off, seed=seed,
                                      standup_failed=early, late_failed=r.fell and not early,
                                      drift_at_ramp=r.drift_at_ramp, peak_drift=r.peak_drift_standup))
    survivors = [r for r in cases if not r['standup_failed']]
    pushes = []
    for push in (-6, -4, -2, 2, 4, 6):
        for seed in (1, 2, 3):
            r = sim.simulate(cfg, plant, duration_s=30,
                             pushes=[sim.Push(at_s=15, delta_vel=push)], seed=seed)
            pushes.append(dict(push=push, seed=seed, fell=r.fell, final_drift=r.final_drift))
    summary = dict(variant=name, cases=len(cases), standup_failures=sum(c['standup_failed'] for c in cases),
                   late_failures=sum(c['late_failed'] for c in cases),
                   median_abs_drift_at_ramp=statistics.median(abs(c['drift_at_ramp']) for c in survivors),
                   median_peak_drift=statistics.median(abs(c['peak_drift']) for c in survivors),
                   push_cases=len(pushes), push_failures=sum(c['fell'] for c in pushes))
    print(json.dumps(summary, indent=2))
    results.append(dict(summary=summary, cases=cases, pushes=pushes))
OUT.mkdir(parents=True, exist_ok=True)
(OUT / 'simulation-candidate.json').write_text(json.dumps(results, indent=2)+'\n')
(OUT / 'simulated-config.json').write_text(json.dumps(asdict(sim.make_variant('current')), indent=2)+'\n')
