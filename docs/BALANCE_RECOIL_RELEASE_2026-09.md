# Confirmed recoil release — September 19 candidate

Austin authorized implementing and uploading the settling experiment after two successful starts. The baseline is source `720f2e31939a249a215b2b7f7c197130f2301310`, application SHA-256 `5531c6287dd5ef8d398d426a7596d8c6d91e6c914825d137bc9a1ef485734e35`, retained in `artifacts/balance-startup-recovery/`.

## Measured reason for the change

The successful September 14/19 runs stop forward motion at 2.525/2.420 s after engagement but finish recovery at 5.681/4.700 s. Their reverse speed reaches about −2 rad/s while the accumulated angle correction relaxes from 2.557/2.302° at the first stop to 1.700/1.730° at settling. This is consistent with excess transient correction, but does not isolate it from body dynamics or velocity-P damping. [The preceding review](BALANCE_SETTLING_REVIEW_2026-09-19.md) records the evidence and limitations.

## Exact firmware change

The same equilibrium integral now releases faster only during confirmed recoil:

- Startup recovery must still be active, the measured arm/base ramp complete, and both wheel samples at most 30 ms old.
- Wheel velocity must oppose the existing integral, reach 0.35 rad/s, and maintain that direction/threshold for 60 ms. Direction changes and stale/invalid timing reset confirmation.
- The multiplier rises linearly from 1 to 2 over 120 ms. Ordinary Ki is 0.231, so the fully enabled release Ki is 0.462. Below 0.15 rad/s, it blends back toward 1 over 120 ms; a changed direction or leaving recovery removes the multiplier immediately without resetting the integral.
- Extra release can remove the existing correction but cannot accumulate extra correction of the opposite sign when crossing zero. Ordinary integration retains its original crossing behavior.
- Existing ±6° integral/offset limits, 6°/s learning-rate bound, anti-windup and feedback gate remain.

Initial trigger/800 ms catch, inner PD, arm schedule and return speed, calibration, stable balance gains, 400 ms calm confirmation and settled-position hold are unchanged. No second estimator or integral reset was introduced. Telemetry feature bit 32 and sample `flags` bit `0x01` identify this phase; the binary schema stays at 220 bytes/sample. Old saved logs retain their original metadata when downloaded through new firmware.

## Verification and limits

Production C++ helper tests exercise both recoil directions, confirmation, hysteresis, gain ramps, stale/invalid inputs, reset after recovery, and 50,000 bounded boosted updates, alongside the previous 50,000 ordinary updates. Model transition checks verify unchanged initial behavior and return to normal learning. The host analyzer reports the new phase only when feature bit 32 is present.

Replay of the production helper on the two successful logs first enables extra release at 2.630/2.560 s, after the first stop and ramp completion. Inputs are recorded from the old controller, including its integral; this verifies phase selection, **not the motion the new firmware will produce**.

The final mirrored policy was compared with the frozen two-success baseline across 324 plant-uncertainty cases:

| Metric | Baseline | Confirmed recoil release |
|---|---:|---:|
| Early / later model failures | 86 / 60 | 79 / 42 |
| New failures / rescued failures versus baseline | — | 0 / 25 |
| Surviving cases that declare settling | 126 | 140 |
| Median settling across each survivor cohort | 6.30 s | 5.58 s |

All 324 cases have identical commands and targets before ramp completion. Among the 120 cases that survive and settle in both versions, the paired median settling-time change is **0.0 s**. The changing-cohort median cannot be claimed as a general 0.72-second gain. These stress cases are not a measured failure probability; the model omits contact, flex, slip and actual CAN loss. Physical improvement remains unverified.

Reproduce with `evidence/balance-recoil-release/screen.py`; full scenario results and production replay are retained alongside it. The earlier ungated multiplier screen now explicitly loads frozen baseline source/config from Git, preserving its historical meaning after these edits.

## Release and physical comparison

Consolidated validation passed: all five native suites, 21 Python tests, Python/shell syntax and whitespace checks, and the pinned ESP32-S3 firmware build. Flash usage is 1,138,913 bytes; static RAM 50,768 bytes, with unchanged 1,320,000-byte PSRAM log. Application file is 1,139,280 bytes, SHA-256 `5d3465e0269f9df19065d1583de35d093b46f947fdd27e49ef0de0bbf28d4dd0`.

Validation and exact release identity are recorded in `evidence/balance-recoil-release/`. The new package is `artifacts/balance-recoil-release/`. Upload application only at `0x10000` after a fresh disarmed check; preserve NVS, LittleFS, partition table and bootloader.

To restore the two-success baseline, verify disarm and run:

```sh
.venv/bin/python scripts/flash_prepared_balance.py --package artifacts/balance-startup-recovery --flash
```

Do **not** add `--rollback`: that option selects the much older rebuilt `e8b1280` image.

For one comparison, use the usual forward arm position and neutral CH1/2/4; set CH7/9/10 HIGH, wait two seconds, then CH11 HIGH one second and LOW once. Support/disarm promptly on runaway. If it settles, leave it untouched for five seconds, then support and lower both CH9/10; keep battery/USB connected for retrieval. Compare forward and reverse speed, wheel excursion, time to sustained calm, and later drift against the two successful runs. Skip disturbance taps for this first comparison.

## Upload outcome

Flashed and verified source `1d80257e5609a173d9bbe104997eab1f9d4c7341`, application `5d3465e0269f9df19065d1583de35d093b46f947fdd27e49ef0de0bbf28d4dd0`, application only at `0x10000`. Fresh preflight confirmed both groups disarmed, no pending log and the prior 1,303-sample run already safely archived. OpenOCD verification passed.

After boot: IDLE, both groups disarmed, six motors online/no errors, calibrated center/back deltas and 3.51° learned trim retained. Forward coordinates reset to zero normally on boot; raw arm poses match preflash. IMU age 5.953 ms, tilt −1.9°, no fault; receiver max 446 µs, CAN receive misses/TX failures zero. Original saved log size remains 287,048 bytes. No tool arming, calibration reset or filesystem upload. Passive observer started with note `confirmed-recoil-release-v1`; onboard logging starts automatically at tip-up. Physical result remains pending.
