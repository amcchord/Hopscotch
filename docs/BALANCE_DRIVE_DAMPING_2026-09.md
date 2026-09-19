# Driving damping and graded arm recovery — September 19, 2026

This fork changes driving after a successful stand-up. Austin authorized implementation, simulation, and an application-only upload, then a stop. Physical movement and later telemetry review belong to Austin's subsequent test. Stand-up repeatability is still unresolved and is outside this change.

## Evidence and reasoning

The installed acceleration/arm-assist v3 (`5a07eba`, application `75ccdb44…afa8118`) removed most of the earlier full-stick delay. The successful September 19 driving trial began responding in about 0.30 seconds, but developed 4–5 Hz fore/aft rocking during movement and braking. Austin confirmed the wobble was entirely front-to-back. See [physical trial findings](BALANCE_DRIVE_TRIALS_2026-09-19.md).

The terminal bout at 32 seconds and later is excluded from tuning because the precise reported wall-contact time is unknown. Seven earlier windows show:

- Raw body rate / common wheel speed magnitude of 12.95–14.57 degrees/radian around 4.33–4.67 Hz.
- Wheel speed / wheel command magnitude of 1.13–1.25 and phase of approximately −24° to −36°. A first-order motor lag cannot represent gain above one.
- The original filtered body rate has only 0.49–0.53 of the raw signal amplitude and roughly 50°–54° additional lag at that frequency.

These are observed **closed-loop frequency ratios**, not a causal identification of robot mass, gravity coefficient, damping, wheel radius, or actuator transfer function. The previous model used much smaller body/wheel coupling and a first-order motor. It missed the physical rocking. The revised model includes optional motor resonance and transport/sensor delay; its parameters remain uncertain.

Normal arm recovery reached +0.45/−0.30 center fractions, far beyond the ±0.10 planned assistance. The inherited emergency threshold was 13.5 rad/s, below the new 20 rad/s driving request limit. At the first logged driving emergency, command was 17.988 rad/s and the body was closing on its target, while braking still produced a substantial speed error. That is poor evidence that the wheels have exhausted their authority.

The failed stand-up immediately before this driving trial never enabled the driving controller or planned arms. This revision does not claim to fix that separate failure.

## Firmware changes

| Driving behavior | v3 | This revision (v4) |
|---|---|---|
| Rate feedback | Original stationary filter, about 60 ms time scale | Separate fresh-sample filter, 6 ms time constant |
| Driving rate gain | 3.1 | 1.5 |
| Angle / speed gains | 8.8 / 3.0 | Unchanged |
| Requested acceleration / braking | 12 / 16 rad/s² | 6 / 8 rad/s² |
| Maximum forward/back / turn request | 20 / 4.5 rad/s | Unchanged |
| Ordinary recovery-arm demand | Existing bounded demand | 30% of that demand, capped at ±0.12 fraction |
| Driving emergency arms | Command above 13.5 plus speed disturbance | Near the actual 30 rad/s wheel cap **and** a confirmed outward lean; severe outward lean remains immediate |

The faster rate path responds above 90% to a step in 20 ms at the nominal 200 Hz loop. At the measured rocking frequency, halving the gain approximately compensates for the new filter's larger amplitude while removing much of the old phase lag. This is a driving-only change; the complementary angle filter, original stationary/startup gyro filter, original PD loop, startup recovery, learned trim, calibration and arm-return sequence remain intact.

The acceleration reference still changes immediately with the stick. The smaller ramp makes a full 0→20 rad/s request take about 3.33 seconds and a 20→0 request take 2.5 seconds. These are **requested speed ramp times**, not measured response or stopping times. The balance controller retains its full 100 rad/s² acceleration authority and 30 rad/s common wheel limit.

Ordinary arm scaling is applied to the already bounded legacy demand, so its largest positive/negative requests become +0.12/−0.09, approximately 12°/9° at the shoulders. Existing event/recoil/cooldown behavior remains. Planned acceleration/braking arms retain their existing gain, 80 ms smoothing and ±0.10 bound; gentler ramps normally request about 0.05/0.067 fractions. An active recovery still takes priority.

During driving or braking, a full emergency catch requires a speed disturbance above the existing threshold plus:

1. Command within 3 rad/s of the 30 rad/s wheel limit, angle error at least 2°, and body motion away from the target at least 8°/s, continuously for 60 ms in the same direction; **or**
2. Angle error at least 8° and outward rate at least 20°/s, immediately, even before the wheels reach their limit.

This uses the new fast rate. A closing lean, ordinary cruise below the rail, a single sample, alternating directions, invalid timing or leaving driving cannot accumulate the confirmation. The full legacy arm range remains available for emergencies. Outside driving, the old emergency rule is preserved.

Radio freshness, centered/calm unlock and reacquisition, input pause thresholds, disarm behavior, IMU/CAN checks, wheel mixing and balance priority are unchanged. No tool arms motors or initiates movement.

## Simulation and validation

The reproducible [evidence directory](../evidence/balance-drive-damping/README.md) contains frozen-v3 comparison, the actual C++ driving/pilot/filter/arm helpers, physical frequency analysis, tuning experiments, outputs, source hashes and a comparison figure.

- **324 neutral cases:** command, setpoint and travel traces match the frozen installed model exactly. The model's optional resonance/delay path defaults off for historical regressions.
- **432 revised-plant driving cases:** 72 parameter combinations each for small inputs, full inputs, reversal, input loss, turn input and modest impulses. Candidate: zero falls; v3: 24 falls; zero new falls. Only 69/72 cases in each profile actually unlock driving, so these are 414 commanded cases, not 432 proven driving successes.
- Median 3–6 Hz wheel-speed spectral power falls across all six profiles. This is a model diagnostic, not a promised physical reduction.
- **Seven native executables and 25 Python tests pass**, including new filter response/invalid input, graded arm limits, direction-confirmed/emergency catches, unchanged PD, command bounds, radio/input gates, mixer priority and 6,000-row schema-4 transfers for both old/new feature flags. Syntax, whitespace and firmware build pass.

The existing pilot test initially assumed full speed would be reached within three seconds. It correctly failed after reducing acceleration. Its elapsed-time allowance now derives from the configured ramp; the velocity/slew/stop assertions remain. The first model-neutral comparison caught a Python-double/C++-float rounding artifact in the bridge. The bridge now avoids rounding an unchanged PD pass-through, matching the established comparison approach.

The revised nominal plant reproduces a fast oscillation with v3 and strongly reduces it with the candidate. It still has quiet-hold ripple unlike the excellent physical stationary hold. Its planar dynamics omit tire slip, yaw mechanics, arm reaction torque, contact, power/current limits and mounting flex. Turn profiles exercise entry into driving with zero forward request; actual yaw signs, limits and balance priority are tested natively, not validated by 3D simulation. Gravity/body coupling is uncertain, and the chosen sweep is a sensitivity range, not a confidence interval.

**Braking tradeoff:** lower rocking does not prove shorter stopping distance. In the selected nominal full-speed case, the candidate first crosses below 0.3 rad/s about 7.32 seconds after centering, traveling about 50.6 wheel radians over the following nine seconds. The old controller's early zero crossings are large oscillations, not clean stops. Stronger speed gains and braking ramps reduced modeled stopping time/travel but increased body motion and, at the strongest settings, input pauses. They were not selected. Actual braking distance remains unverified. The small-input model exceeds +0.5 rad/s about 0.64 seconds after the stick changes; it does not predict a return to the previous several-second full-stick dead zone.

## Telemetry and release

Sample layout stays schema 4 / 240 bytes / 6,000 rows (1,440,000-byte PSRAM buffer). Feature bit 512 identifies this algorithm (total 1023); exports include the driving filter, gains, ramp and arm thresholds. Older stored logs keep their original metadata. Existing raw gyro, filtered stationary rate, speed, request, command, planned/total arms and emergency flags remain available. The new 200 Hz filtered driving rate is not separately sampled; the 50 Hz raw gyro cannot reconstruct it exactly. Live `bal status` identifies “Driving v4” and its constants.

The v3 log was downloaded again before programming to preserve the exact on-device record. This copy is the same 2,381-sample physical trial, not a new run. The initial USB-only preflight found both groups disarmed but motor traffic stopped and stale CAN faults; powered checks are required after programming.

Release image/package identity and actual upload/stationary outcome are recorded below after deployment. The release uses only application offset `0x10000`; bootloader, partitions, NVS, LittleFS and calibration are preserved. Exact prior packages and the original full-device backup remain available. Download any new log **before restoring v3 or v2**: v3 would label the new same-layout log with older driving metadata, and v2 cannot read schema 4. Never use the helper's unrelated `--rollback` option.

After the verified upload and stationary checks, this task stops. No passive test observer or physical trial is started.

### Deployed outcome

Application-only programming at `0x10000` and readback verification succeeded. Frozen firmware source is **`a2774d4999b1c0f9ce707a9377aecd2a3661a3fe`**, application SHA-256 **`90a3df4d16276860059caacc059959ea6d774d2dc8528b68353aee90043e757c`**, 1,145,552 bytes. Build flash use is 1,145,193 bytes and static RAM 50,848 bytes. Exact package: `artifacts/balance-drive-damping/`; prior v3 package also checksum-verified, and the original full-device backup hash was rechecked.

Postflash USB status identifies Driving v4 / 0.006 s / gain1.50 / accel6 / brake8. IDLE, both groups disarmed; IMU age6.046ms/no latched fault, transmitter linked with receiver maximum488µs. Stored trim3.33° and center/back calibration deltas1.768/−1.767 and3.661/−3.670 remain. The previous571,828-byte saved log is still present. Forward references are transient software coordinates, captured on operator arming; their zero values before any motor communication are not a calibration reset.

**Powered motor health is not verified.** No motor replies arrived after boot, all motor readings are defaults, and CAN errors increased. The earlier battery-on question has not been answered. This condition predates the upload; it must not be reported as healthy motors or proof of a firmware regression. The task stops at the completed upload and available stationary checks. No physical movement, arming, calibration/settings writes or trial observer were initiated. Later powered checks and physical log review remain for the operator's follow-up.
