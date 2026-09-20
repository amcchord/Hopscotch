# Settling review after two successful starts

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](WIFI_OTA.md) and the [current test guide](BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

This is a recommendation review, not a firmware change. The tested application remains `720f2e3` / SHA-256 `5531c6287dd5ef8d398d426a7596d8c6d91e6c914825d137bc9a1ef485734e35`. No USB access, motion, tuning or flash was performed during this review.

## Where the remaining time goes

| Measured phase, seconds after balance engagement | September 14 | September 19 |
|---|---:|---:|
| Forward motion first crosses zero (filtered speed) | 2.525 | 2.420 |
| Peak reverse filtered wheel speed | −2.043 rad/s | −1.984 rad/s |
| Calm hold declared | 5.681 | 4.700 |
| First stop to settled hold | 3.156 | 2.280 |
| Learned integral at first stop | 2.557° | 2.302° |
| Learned integral at settling | 1.700° | 1.730° |

The initial catch works. A substantial part of the remaining time is a backward recoil while the accumulated correction relaxes. This is consistent with more transient correction than the final stance requires, but it does not isolate the integral as the sole cause: body dynamics and the change in velocity-P authority also contribute. A larger initial catch is not supported by these two traces.

## Best next experiment: faster release during confirmed recoil

Keep initial detection and the proven 800 ms catch. Once the arms/base ramp is complete, recovery remains active, and wheel motion opposes the accumulated correction, let the existing single integral unwind somewhat faster. A bounded 1.5–2× temporary unwind gain is a test candidate (ordinary Ki 0.231; candidate approximately 0.35–0.46), not a selected release value. Preserve the integral itself, all angle/rate/feedback limits, and normal gains after recovery. Confirm the direction long enough to reject noise and use a smooth change of learning rate; avoid an instantaneous reset of the learned angle.

The broader control principle is smooth transfer when changing controller gains; it prevents the transition itself from disturbing the system. [MathWorks gain-scheduling guidance](https://www.mathworks.com/help/control/ug/designing-a-family-of-pid-controllers-for-multiple-operating-points.html) supports that principle, not the proposed robot-specific gains.

A small source-derived model experiment changes only the active, post-ramp integral gain when `velocity_error * integral < 0`. It does **not** yet implement extra direction hysteresis or blending. In 324 stress cases:

| Unwind multiplier | Early / later failures | New failures vs current | Previously failing cases rescued |
|---|---:|---:|---:|
| 1×, current | 86 / 60 | — | — |
| 1.5× | 82 / 46 | 3 | 21 |
| 2× | 79 / 43 | 2 | 26 |

This makes the idea worth investigation, but not a validated upgrade. Median settle time across changing survivor populations falls from 6.30 s to 5.69 s at 2×; **the paired median change among cases that survive and settle under both is zero**. Do not claim a general 0.61-second speedup from that cohort change. The existing planar model omits contact, arm reaction, mounting flex and slip. No physical benefit has been measured for the proposal.

## Robustness priorities

- Retain the 400 ms calm confirmation. Both logs contain an approximately 244–260 ms interval satisfying the logged speed/rate/error limits around the first stop, before the reverse swing. A 200 ms timer could classify that transient as settled. This is a replay of logged fields, not exact instruction-by-instruction controller timing.
- Retain the same settled-position hold and continuous learned correction. Alter one recovery behavior at a time, preserving a clear comparison to the two successful traces.
- Compare repeated unchanged baseline/candidate starts under matched battery/surface/starting-pose conditions. Measure forward and reverse peak speed, total wheel excursion, time to sustained calm and later drift. A candidate must improve the physical motion, not merely declare settling sooner.
- Test small marked disturbances only after ordinary starts repeat well, as a separate experiment. Two successful starts do not yet establish disturbance robustness or a population success rate.

A response-based taper of the initial boost is a possible later investigation if recoil remains. The previous 400 ms timer-only screen introduced early failures, so simply shortening the boost is not the first recommendation. Keep the initial catch intact for the first experiment.

Reproduction: `evidence/balance-settling-review-2026-09-19/review_phases.py` extracts measured phases; `screen_unwind.py` runs the small screen and `screen_unwind.py --broad` runs the 324-case comparison. The modified simulator is generated in ignored `output/`; production source is unchanged. JSON/text evidence is retained alongside the scripts.
