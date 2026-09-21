# Standing drive response v2 — September 19, 2026

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../WIFI_OTA.md) and the [current test guide](../../BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

Austin tested CH1 steering / CH2 forward-back on the first standing-drive release and reported that all directions worked, but forward/back had a long delay and both axes were too slow. This release changes driving response and limits. Stand-up, startup catch/recoil release, calibration, motor caps, and stationary gains remain unchanged.

## Evidence and diagnosis

The full first-drive log is archived in `telemetry_logs/bal_20260919_155240_standing-drive-first-test.csv` with 2,881 validated samples, binary checksum `0x90FA4B79`, USB `0xED14696F`. The [first-drive findings](BALANCE_STANDING_DRIVE_2026-09.md#first-physical-test-and-response-follow-up) include the plot and health results. Full forward/reverse inputs took 6.42/6.84 seconds to exceed 0.25 rad/s in the requested direction, then overshot to roughly ±3.1 rad/s against a ±1 request. Driving stayed unlocked and the receiver did not stall: this was not delayed stick delivery.

At low error, cruising feedforward (+1 rad/s for a +1 request) almost cancels the old stationary velocity P correction (−0.462° × inner Kp 2 = −0.924 rad/s). The resulting initial command is only +0.076 rad/s at the old equilibrium, instead of a useful lean-initiating counter-motion. The log shows integral adjustment continuing for seconds before appreciable travel. Stronger driving P is intended to initiate the lean promptly and oppose speed overshoot sooner; physical benefit needs another operator test.

## Changes

| Parameter | First drive | Response v2 |
|---|---:|---:|
| Maximum average requested wheel speed | 1 rad/s | **2 rad/s** |
| Maximum per-wheel turn differential | 0.5 rad/s | **1.5 rad/s** |
| Forward acceleration | 0.5 rad/s² | **1.5 rad/s²** |
| Braking/reversal slew | 0.75 rad/s² | **2 rad/s²** |
| Turn slew | 0.75 rad/s² | **3 rad/s²** |
| Low-error velocity P while driving/braking | 0.462°/(rad/s) | **1.0°/(rad/s)** |

Full forward request now ramps in about 1.33 seconds; half request in about 0.67 seconds. A full turn request ramps in 0.5 seconds. These are command-shaping times, not measured physical acceleration or stopping time. Speed remains independent of the ground-mode knob. 2 rad/s is about 19 wheel RPM; no assumed wheel radius is used to promise a floor speed.

The stronger low-error gain applies only while `BalancePilot.moving()` is true, including the braking interval. The existing high-error slope, soft-knee continuity, angle-error gating, arm coordination, high-speed shedding, ±8° correction bound, 12°/s setpoint slew, single integral, and cruising feedforward remain. After zero requests and 400 ms calm, it returns to the gentle stationary gain and holds the new location. Any gain transition still passes through the existing setpoint slew limit.

No deadband, centered-unlock delay, input/feedback freshness threshold or safety exit changed. A held stick through startup or a pause still requires centered/calm reacquisition. CH7 HIGH suppresses ground drive before stand-up; CH7 LOW retains ground mode. Both physical motor switches must remain armed for standing driving. Persistent trim still qualifies only after rest.

Telemetry keeps schema 3, 236-byte samples and 6,000-sample capacity. New feature bit 128 gives features **255**, identifies `ch1_ch2_velocity_response_v2`, records the new limits and moving low-error gain. Older v1 logs continue exporting their original features 127 and original limits, not the downloader's new settings. No storage reset, filesystem update or schema migration.

## Verification and limitations

Six native suites and 23 Python tests, source/shell syntax, whitespace and the pinned firmware build passed. Native checks exercise actual production gain selection at rest/moving/braking, return to stationary gain, both signs, knee continuity, the initial counter-motion sign, bounded command slew, centered reacquisition and old/new telemetry layouts. Existing safety/parser/transport checks remain green.

All 324 neutral model trajectories exactly match installed v1. The final shorter forward–stop–reverse–stop and input-loss screens each have 9 model falls versus 11 for the holding reference, zero new failures; 36/31 of 54 cases actually unlock/command motion. The rest cannot be called successful driving tests. The model does not reproduce the real robot's quiet hold and delayed-motion dead zone: no commanded survivor meets the arbitrary final RMS <0.3 rad/s metric. It cannot establish precise speed tracking, yaw, tire slip, stopping distance or physical reliability.

A separate 54-case-per-profile longer-duration experiment compares the selected 2 rad/s/1.5 acceleration/2 braking/1.0 moving P against installed v1. Each selected profile has 9 model failures versus 10 for v1, with zero new failures. Three rejected 3 rad/s/2 acceleration/3 braking variants introduced 1–2 new input-loss failures depending on moving gain, so they were not flashed. This supports the moderate first increase, not a claim of a maximum safe speed. `experiment.py` freezes v1 source/config; `screen.py` calls the actual new C++ helper and velocity correction. Results are retained under `evidence/balance-drive-response/`.

Application SHA-256 **`b28793cdd8d2b45cdfbcb344cc594ba85ba79fadcdb2e373bca321a1d86b172b`**, file 1,141,840 bytes; build flash used 1,141,469 bytes, static RAM 50,800 bytes. The source commit and actual device checks accompany the frozen package. Physical response-v2 validation is pending.

## Release and test

Application-only upload at 0x10000 after a fresh supported/disarmed status check, preserving calibration, stored 3.89° trim, NVS and LittleFS. The first-drive log is already checksummed/archived. After upload, check motor/IMU/CAN health and export the existing v1 log to verify its metadata is preserved before another moving run.

Use the usual stand-up sequence: CH1/CH2/CH4 centered, CH7/CH9/CH10 HIGH, wait two seconds, CH11 HIGH one second then LOW. Keep sticks centered for a second after settling. Try about one-quarter CH2 for one second, then center until stopped; repeat backward. Then try brief quarter-stick turns. The new turn cap is three times higher, so begin with small steering inputs. Finish a short run well before the 120-second log limit; support and lower CH9 and CH10, leaving power/USB for retrieval. There may be a brief wheel counter-motion to create the lean needed for acceleration. Support/disarm promptly if the response is uncontrolled.

Restore the tested but sluggish first-drive image with a fresh disarmed check and:

```sh
.venv/bin/python scripts/flash_prepared_balance.py --package artifacts/balance-standing-drive --flash
```

Exact v1 source `90f07e83e3472cebbc5ca2646e585bfbce20f2d7`, app `83d50ac277799094e58395cab3a9296e66c27718226da847a0442eea5a4c7c7d`. The earlier recoil-release stand-only package and full original backup also remain. Do not use `--rollback`, which selects a much older image. Download any new run before restoration.

## Verified upload

Source **`d1ae97d5ef31b96a9f3f39a2765998f06b8d965b`**, application **`b28793cdd8d2b45cdfbcb344cc594ba85ba79fadcdb2e373bca321a1d86b172b`**, was uploaded at 0x10000 and OpenOCD reported `Verify OK`. Fresh preflight and postflash both confirmed drive/arms disarmed, IDLE, six online/error-free motors, calibration retained and 3.89° learned trim. Postflash IMU age 1.102 ms/no fault, receiver max 495 µs, CAN misses/TX failures zero. Forward coordinates normally zeroed on boot while raw poses and center/back deltas were retained. No tool arming or movement, partition/filesystem upload or settings reset.

The saved first-drive log was downloaded again through the new firmware: all 2,881 sample rows and original configuration/integrity metadata are identical, including features127/v1 limits and both checksums. This checks historical-version export without relabeling the first trial as v2. New copy `bal_20260919_160318_v1-export-on-drive-response-v2.csv` is explicitly a compatibility export of the same run, not a second physical run.

`evidence/balance-drive-response/device-checks.json` and `compat-check.json` record release checks. The frozen package is `artifacts/balance-drive-response/`; first-drive and recoil-release packages remain for restoration. Passive observer started with note `standing-drive-response-v2`; Austin was invited to test small inputs and report delay/stopping. Physical response-v2 benefit remains unverified until that run is retrieved and assessed.
