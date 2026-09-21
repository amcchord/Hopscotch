# Fast capture / forward roll-away — September 20, 2026

The distinct CH6 HIGH trial reached quiet capture at **2.895 seconds**, then
rolled forward after the arms returned. The source correction is **`f4d2bb7`**.
The lowering owner installed combined source `a772ecc` in app1 at 21:05:08 UTC.
[Exact image, powered disarmed health and unchanged saved telemetry](installation.json)
are verified. Successful physical balance with the new handoff remains untested.

## Recorded evidence

The device owner archived 236 rows / 4.750 seconds from installed `17c499c`.
All samples carry fast-run bit128. The log ends `bailout_angle_error`.
[Derived events and source hash](fast-trial.json), [timeline](fast-trial.png).
The raw CSV/wire remain in the owner's `worktrees/balance-lower/telemetry_logs/`
checkout as `bal_20260920_fast_tip_v1_wifi`; they were read in place.

| Event | Time | Observations |
| --- | ---: | --- |
| Quiet capture | 2.895 s | Tilt82.458°, rate−0.169°/s; rear wheels−0.086/−0.072rad/s |
| Arms start return | 3.321 s | Tilt82.456°; arm torques1.467/1.564Nm; curve shift−2.860° |
| Both arm loads below0.4Nm | 3.401 s | Arm fraction0.984; loads−0.034/0.298Nm, consistent with support release |
| Early roll recovery | 3.706 s | Tilt81.721°; wheels1.164/3.182rad/s; arms0.827 fraction |
| Bailout | 4.750 s | Tilt43.087°; target88.158°; arms stalled near0.614 fraction |

The fast trajectory and startup wait worked. Quiet capture was still strongly
supported by the arms. The ordinary capture calibration made a temporary
−2.857° engage reference into a permanent −2.860° curve correction, cancelling
most of the saved +3.2398° trim. This is a demonstrable use of supported posture
as an equilibrium estimate. It does **not** prove the saved trim is exactly the
right free equilibrium, or that it alone caused the physical failure. The log
also shows large left/right wheel asymmetry that the planar model cannot explain.

The [previous available run](available-log.json) was slow mode and was excluded
from diagnosis of this distinct fast failure.

## Bounded correction

Fast capture keeps the saved equilibrium trim. Its supported angle remains a
transient reference, fading during the first0.10 of **measured** tip-fraction
return. Progress is monotonic: target movement alone cannot release it, and
arm bounce/feedback noise cannot restore it. The existing base slew limit
(4°/s with its existing velocity gate) bounds the actual target transition.
It is not a step to the stored angle. The existing recovery integrator remains.

Slow capture still calls the same `captureCurveShift` arithmetic with identical
inputs; its transient is still cleared. Fast arm motion2.6s, motor cap2.2rad/s,
return speed1.5rad/s, filters, gains, current limits and ordinary drive are unchanged.

Separately, fast capture now requires **each** rear wheel≤0.75rad/s in absolute
speed throughout the original120ms quiet dwell. This retains the startup bound
and4.5s deadline. [A counterexample against the installed source](capture-probe.cpp)
previously captured at2.740s while both wheels reported5rad/s
([original result](capture-probe.txt)). That omission did **not** cause this
trial, whose capture wheels were stopped. Native tests now reject it, including
opposing wheel speeds, single-wheel motion, fresh dwell and eventual timeout.

## Validation and remaining uncertainty

The actual C++ native test passes all162 trajectory tracking cases (max2.740s,
2.0552rad/s,2.43902rad/s²) and the new capture-reference, old slow arithmetic,
saved-trim sign/clipping, monotonic release, paused/rebounded arms, wheel-motion,
quiet-dwell and deadline regressions. [Validation](validation.json).

The [paired45-case sensitivity screen](capture-release-screen.json) is explicitly
**not a fitted replay of contact dynamics**. It holds the initial supported
posture, then releases at arm fractions0.99/0.975/0.95; it varies true equilibrium
±3° around saved trim and motor lag0.08/0.20/0.35s. Both policies avoid a model
fall in24/45 cases: **10 improve and10 regress**. Regressions occur when the
saved trim overestimates the modeled true equilibrium by1.5–3°. No-fall does
not mean calm balance or small travel. With nominal assumptions, peak wheel
speed falls6.997→5.557rad/s and final drift28.921→−4.364rad. These are screening
outputs, not predictions of the next robot trial. A naive fade over the entire
arm return was rejected after only19/45 no-fall outcomes.

The production correction fixes the supported-reference error. It cannot promise
successful free balance; saved-trim accuracy, support reactions and asymmetric
traction remain physical uncertainties. The next distinct operator fast trial
must retain capture, arm release and subsequent balance in one saved run.

## Reproduction and coordination

```sh
clang++ -std=c++17 -Wall -Wextra -Werror -Isrc tests/test_balance_tip_up.cpp -o output/test_balance_tip_up
output/test_balance_tip_up
/Users/austinmcchord/Development/Hopscotch/.venv/bin/python scripts/analyze_fast_tip_capture.py /Users/austinmcchord/Development/Hopscotch/worktrees/balance-lower/telemetry_logs/bal_20260920_fast_tip_v1_wifi.csv --output evidence/fast-tip-up/roll-away-review
/Users/austinmcchord/Development/Hopscotch/.venv/bin/python evidence/fast-tip-up/roll-away-review/capture_release_screen.py
```

The standalone wheel probe intentionally uses pre-fix `17c499c` headers; the
current native test is the passing regression. `capture_release_screen.py`
mirrors the policy inside the established simulator and leaves that simulator's
ordinary callers unchanged. It asserts transformation sites before running.

The lowering owner owns schema6 metadata, final combined validation/build,
device access and OTA. Schema≤5 fast strings must remain literal; schema6
identifies `ch6_fast_v2`, wheel capture0.75rad/s and transient support release.
This task made no robot requests, arming/motion, configuration writes, OTA,
raw-log copies or shared progress edits.
