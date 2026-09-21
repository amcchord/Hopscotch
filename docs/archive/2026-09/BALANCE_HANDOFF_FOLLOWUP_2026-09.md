# September 14: failed stand-up after the feedback correction

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../WIFI_OTA.md) and the [current test guide](../../BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

The feedback correction did not resolve initial roll-away. Austin reports that
the robot stood, ran away a substantial distance, then became stable after he
stopped it by hand. He attributes the reboot to bumping the USB cable. Do not
count this as an unaided success or diagnose it as a firmware crash.

## Preserved evidence

The [download](../../../telemetry_logs/bal_20260914_222023_feedback-fix-first.csv)
contains all 1,428 samples over 28.829 seconds. Binary checksum `0x1D0F7192` and
USB checksum `0x6A821A10` validate. The saved end reason is `arms disarmed`.
Feature flags are 7, identifying the installed feedback correction (`cad335c`,
application SHA-256 `f23b6faafba6cb615123955e0744ff73b28793b33b5c384ad4addea17339dcb3`).
The host observer disconnected before the actual attempt; the board retained the
trial and it was downloaded after reconnection. This download's profiler scope
is **live after reboot**, so its counters cannot measure receive loss or stalls
in the preceding run.

The [comparison figure](../../../evidence/balance-handoff-followup/first-trial-comparison.png)
and [metrics](../../../evidence/balance-handoff-followup/first-trial-metrics.json) compare
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

## Capture correction released for the next diagnostic

Austin reported ready after the sensor/flat-pose request. Fresh USB preflight
confirmed both motor groups disarmed. Flashed source
`a8aa9a123fdb3e197d56e3d9ba8d5b9a6cb0ace7`, application SHA-256
`068d043b64f957f577bae658446638e56c3c2b60ea523946bff4614a8d76a4c3`, at
`0x10000` only; OpenOCD verification passed. Package:
`artifacts/balance-capture-fix/`. Postflash: six motors online, no motor errors,
both groups disarmed, trim 0.35°, stationary tilt −1.8°, IMU age 5.977 ms with no
fault, receiver max 517 µs, CAN receive misses and TX failures zero. The existing
calibration partition was not written; exact calibration reread follows the
trial (the initial script used unsupported `arm status`, now corrected to
`cal status`).

The host recorder is active, with log note `capture-trim-secured-sensor`. Austin
has been asked for one short ordinary stand-up, prompt support/disarm if it runs
away, five seconds untouched only if stable, and no tap. No tool armed motors or
initiated physical movement. Physical improvement remains pending.

## Capture-correction trial outcome

The requested trial also ran away and needed a hand stop, then balanced well. All 1,119 samples were saved/checksummed (`0x3F3F58BA`, USB `0x28979A65`), end `drive disarmed`. The actual run was observed without reboot; run receiver maximum 516 µs, no stalls, CAN receive misses/TX failures zero. The first four balance seconds reached 19.735 rad/s average wheel speed and 15.244 rad wheel travel. Calibration was reread unchanged; learned trim is 2.44°. Thus the capture-bound bug fix is retained, but it does not solve the early runaway.

Austin requested early runaway detection and a strong reactive correction instead of searching for a fixed angle. The unflashed 3.5°-curve candidate was discarded; the selected wheel-feedback learning response and its mixed model evidence are documented in [startup recovery](BALANCE_STARTUP_RECOVERY_2026-09.md).
