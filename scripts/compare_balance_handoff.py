#!/usr/bin/env python3
"""Reproduce the rejected, simulation-only gradual outer-loop handoff experiment."""
import json
import statistics
from pathlib import Path
from dataclasses import replace
import balance_sim as sim

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/balance-review'
plant = sim.load_fitted_params(OUT / 'model-fit-speed.json')
results = []
for handoff in (0, 1000, 1500, 2000, 3000):
    config = replace(sim.current_firmware_config(), outer_handoff_ms=handoff)
    early = late = count = 0
    drifts, peaks = [], []
    for factor in (0.7, 1, 1.4):
        for delta in (-1, -1.9, -3.3, -4.5):
            for offset in (-0.8, 0, 0.8):
                for seed in (1, 2):
                    result = sim.simulate(config, replace(plant, A=plant.A*factor, eq_tip_delta=delta),
                                          engage_offset_deg=offset, duration_s=16, seed=seed)
                    count += 1
                    failed = result.fell and (result.ramp_complete_s is None
                                             or result.t[-1] < result.ramp_complete_s + 3)
                    early += failed
                    late += result.fell and not failed
                    if not failed:
                        drifts.append(abs(result.drift_at_ramp))
                        peaks.append(abs(result.peak_drift_standup))
    pushes_failed = sum(sim.simulate(config, plant, duration_s=30,
                                    pushes=[sim.Push(at_s=15, delta_vel=push)], seed=seed).fell
                        for push in (-6, -4, -2, 2, 4, 6) for seed in (1, 2, 3))
    row = dict(handoff_ms=handoff, cases=count, early=early, late=late,
               drift=statistics.median(drifts), peak=statistics.median(peaks), pushfails=pushes_failed)
    results.append(row)
    print(row)
(OUT / 'handoff-comparison.json').write_text(json.dumps(results, indent=2)+'\n')
