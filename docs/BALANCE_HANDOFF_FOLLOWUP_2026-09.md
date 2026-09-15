# September 14: failed stand-up after the feedback correction

The feedback correction did not resolve initial roll-away. Austin reports that
the robot stood, ran away a substantial distance, then became stable after he
stopped it by hand. He attributes the reboot to bumping the USB cable. Do not
count this as an unaided success or diagnose it as a firmware crash.

## Preserved evidence

The [download](../telemetry_logs/bal_20260914_222023_feedback-fix-first.csv)
contains all 1,428 samples over 28.829 seconds. Binary checksum `0x1D0F7192` and
USB checksum `0x6A821A10` validate. The saved end reason is `arms disarmed`.
Feature flags are 7, identifying the installed feedback correction (`cad335c`,
application SHA-256 `f23b6faafba6cb615123955e0744ff73b28793b33b5c384ad4addea17339dcb3`).
The host observer disconnected before the actual attempt; the board retained the
trial and it was downloaded after reconnection. This download's profiler scope
is **live after reboot**, so its counters cannot measure receive loss or stalls
in the preceding run.

The [comparison figure](../evidence/balance-handoff-followup/first-trial-comparison.png)
and [metrics](../evidence/balance-handoff-followup/first-trial-metrics.json) compare
this with the previous assisted timing-fix trial. Historical wheel feedback is
converted by 50/33 for the comparison only; source CSVs remain unchanged.

| Measurement | Previous trial | Feedback correction trial |
|---|---:|---:|
| Balance engagement | 8.846 s | 9.051 s |
| Arm return begins after engagement | 0.420 s | 0.420 s |
| Ramp completes after engagement | 2.280 s | 3.065 s |
| Peak first-four-second wheel displacement | 9.187 rad | 18.667 rad |
| Peak first-four-second average wheel speed, physical units | 11.035 rad/s | 26.322 rad/s |
| Peak first-four-second command | 10.839 rad/s | 25.846 rad/s |
| Engagement tilt | 84.015° | 78.081° |
| Median later calm, arms-forward tilt | 87.677° | 82.600° |

Displacement is wheel rotation, not measured floor distance. Hand-contact times
are unknown. The later calm samples follow assistance and do not prove an
unaided transition. During the new trial, after the first 50 ms of BALANCE,
reported **motion** feedback age never exceeded 1 ms; the initial 121 ms age
belongs to tip-up's slower zero-speed refresh. During BALANCE, the inner tick
maximum was 5.185 ms and IMU age maximum 15 ms; across the entire trial those
maxima were 5.485 ms and 19 ms. No recorded IMU freshness or CAN TX failure flag.
Old decoder timestamps could be refreshed by non-motion replies, so an equal
reported old age does not establish equal old feedback quality.

## What the data suggests, and what remains unknown

All three recent assisted runs show later calm arms-forward tilt about 3.7–4.5°
above engagement (3.97°, 3.66°, 4.52°). The configured tip-to-forward curve spans
1.9°. This suggests that the capture pose and free-standing equilibrium need
better separation. A quiet tip stance may still be supported by the arms; it is
not automatically a free-balance measurement. Changes in support/contact and
arm reaction are missing from the current model.

The starting tilt shifted almost 6° between the last two trials despite similar
arm references. Austin previously reported sensor-mount play. Mount movement is
a possibility, not a confirmed explanation. He has been asked to secure the
sensor and prepare the usual flat pose for a stationary check before another
moving test. Do not infer a mounting change from USB disconnection alone.

## Concrete correction prepared

The capture code clipped the **relative** run correction to ±6° after subtracting
the old stored trim. Consequently, an otherwise valid new measurement depended
on an unrelated previous run's trim. For example, tilt 78.1°, scheduled 82.2° and
old trim +2.3112° produced a reconstructed target of 78.5112° instead of 78.1°.

The correction bounds the **total measured trim** to the existing ±8° balance
trim range first, then subtracts the component already in the base. The example
now reconstructs 78.1° regardless of stored trim. The overall target clamp and
base rate limit remain. This fixes a demonstrated bias; the roughly 0.3–0.4°
clipping in this trial is insufficient evidence to explain all its roll-away.

Native regression cases cover that measurement across stored trim −8° to +8°
and both total-trim limits. New telemetry uses feature bit 8 and
`capture_trim_bounds=absolute`; the binary layout remains schema 2. The simulator
retains the historical relative-bound behavior for historical configurations and
mirrors the corrected behavior for current firmware.

Inner PD, velocity gains, target curve, arm speed/poses and calibration remain
unchanged. Firmware is prepared for a controlled follow-up, not certified as a
reliable unaided stand-up solution. Consolidated checks pass four native suites,
16 Python tests, syntax/whitespace and the ESP32-S3 build (1,135,969 flash bytes,
50,720 static RAM bytes).

## Further experiments rejected for deployment

Screened larger target-curve spans, different low-speed gains and faster arm
returns using a model with corrected wheel units and more frequent feedback.
These are approximate screening assumptions, not a new validated plant fit.
A 3 rad/s return improves aggregate results in 324 stress cases (early/late
failures 121/34 versus 145/49), but **15 previously passing cases fail**, and a
similar 2.5 rad/s change failed in July's physical run `223630`. It is not deployed.
Larger angle schedules also introduce regressions. Retained scripts/configuration
and results under `evidence/balance-handoff-followup/`; none of these experiments
changes the released arm speed or gains.

Next: confirm the sensor is fixed and inspect it stationary with both motor
groups disarmed. Verify/package the capture-bound correction; record exact
installed identity before any further operator-triggered diagnostic trial.
Another unchanged full stand-up has not been requested.
