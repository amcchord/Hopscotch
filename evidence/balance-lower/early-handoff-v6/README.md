# Earlier moving handoff v6 — offline screen

The [physical v5 trial](../trial-v5-20260920/README.md) ended before commitment:
upright PD drove one wheel past 6 rad/s while waiting for the final arm travel.
The existing continuous loaded arm return was never reached.

V6 adds one earlier handoff condition: both arms have at least 1.6 rad of reach,
both are still advancing at least 0.3 rad/s, neither has 0.4 Nm of load, the body
has fallen at least 0.5° and its forward rate is at least 2°/s. The original final
preparation position remains an alternative. This early path refuses stalled,
loaded or asymmetric arms. All preparation/global motion limits are unchanged.
Wheel ownership changes to the existing fall/catch controller using measured
velocity; catch contact then reverses the arms toward calibrated Forward and
continuous supported descent. No catch or return tuning changed.

Production-policy replay of the recorded samples reproduces v5's fault at
24.365 s. V6 hands off at 24.326 s, 39 ms earlier, with wheel feedback still below
its limit. Replay intentionally stops there; it does not claim physical success
after the new commands diverge from the recording. Native regressions cover that
boundary, blocked early handoff, and subsequent qualified contact followed by
continued return rather than a completed/held supported pose.

## Paired model result

[329 cases](simulation.json) against [installed v5](baseline-simulation.json):
253 → 255 completions, [9 improvements and 7 regressions](comparison.json).
All 72 trial-informed contact cases complete. Delayed-contact cases remain
15/24. All 11 injected failures reject completion, and the obstructed-arm case
still stops before commitment. The asymmetric-servo trial now completes; the
recorded-contact stress examples remain complete. Nominal behavior is unchanged.
[Summary and individual limits](summary.json).

An earlier 1.35-rad handoff prototype was rejected: it reduced completions to
221/329 and worsened contact cases. The retained narrower condition screens for
moving, unloaded arms; it is not a general relaxation of the safety thresholds.
The 1.6-rad prototype without load qualification had 253 completions and delayed
the obstructed-arm rejection, so it was also not retained.

The model still uses unmeasured geometry, mass distribution, friction and
contact/servo dynamics, excludes lateral and structural loads, and stops at
fault rather than predicting the subsequent fall. Sampled limits cannot bound
between-tick peaks. Manual robot validation remains necessary. Reproduce with
`python3 scripts/simulate_lowering.py --baseline-ref 17c499c`.
