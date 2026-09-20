# Centered-stick braking investigation — September 19, 2026

Status: braking v5 was installed with the first combined release at 03:56 UTC
September 20. Austin subsequently reported ground/standing driving worked well.
The later installed lowering-v2 image retains this braking code unchanged; see
[current state](progress/CURRENT.md) for its identity and the hold on further
lowering trials. This report preserves the original slow-stopping investigation
and first combined release evidence.

## What the wireless log shows

The authenticated Wi-Fi download contains all **2,981 samples**, validated against the stored binary checksum `0xEAFD0F41` and transport checksum `0xCB02414A`. The 59.611-second run includes 50.785 seconds in BALANCE and ends by drive disarm. Preserve both the cleaned CSV and byte-exact `.wire` export in `telemetry_logs/bal_20260919_wifi_v4_slow_stop.*`.

Driving unlocked 2.56 seconds after balance engagement and remained ready. Measured wheel speed spans −17.37 to +14.33 rad/s. No emergency-arm events, saturated inner ticks, IMU faults or CAN transmit faults appear in this run. The longest recorded balance inner interval is 5.244 ms. Cumulative timing counters observed separately include time outside this run; those counters do not establish a Wi-Fi-induced balancing delay.

Five sufficiently long neutral intervals first reach |speed| <0.3 rad/s after **1.38–2.12 seconds**. The ramped reference takes **1.20–2.08 seconds** to reach zero in these intervals. The remaining sixth interval is interrupted by new throttle after 0.56 seconds. Recoil then prolongs settling in some stops; the largest reversal is 7.10 rad/s during a turning interval. The log has no contact/intervention markers, so these are observations, not controlled stopping-distance measurements.

The firmware intentionally reduced braking to 8 rad/s² when fixing the front/back rocking. At a 16 rad/s reference, that alone takes two seconds to request zero. The acceleration controller continues balancing during the ramp and while settling; it does not immediately capture a stationary hold point when the stick centers.

## Final candidate: progressive reference braking

The extra controller gain and early PD handoff were rejected. The production inner balance controller is byte-for-byte the installed source; this change belongs to the pilot reference ramp.

- Reference deceleration remains **8 rad/s² below 4 rad/s**, rises smoothly to **20 rad/s² between 4 and 8 rad/s**, and stays at 20 above 8. This applies when centering CH2, requesting a lower speed or reversal, and when stale/invalid input requests a stop.
- The full-speed reference reaches zero in about **1.4 seconds instead of 2.5**. A 16 rad/s reference takes about 1.2 seconds instead of 2.0. These are reference-ramp times, not measured physical stopping times.
- During the faster part of braking, planned arm assist stays within the previous v4 braking amplitude, about **0.0667 of the calibrated center travel**. Faster reference braking does not produce a larger shoulder swing.
- Acceleration stays 6 rad/s², full speed 20 rad/s, steering differential 4.5 rad/s. Existing balance authority, damping, motor limits, arm recovery, fresh-input checks and calm neutral reacquisition are unchanged. Low-speed pilot trajectories/arm assist remain bit-for-bit v4 in the native regression.
- Telemetry remains the 240-byte schema-v4 layout. Feature bit 1024 identifies braking-v5 settings; pilot flag 32 marks the faster part of reference braking. Exporting an old v4 log continues to describe its original limits.

The concurrent flat-driving gate fix is included. The separately requested experimental [CH11 supported return to flat](BALANCE_LOWER_2026-09.md) is also included after its native/model checks and integration review. CH11 while balancing now requests lowering; its first physical test needs restraint and verified arm clearance.

## Experiments, checks and limits

Early handoff to stationary PD produced an earlier zero crossing but fewer calm stops and worse settling. Extra speed feedback also looked promising in short stop tests, but broader disturbance/replay screens revealed additional falls. Both approaches are rejected and retained only as offline experiments. A uniformly faster ramp also worsened one weak-coupling disturbance case; keeping the proven rate near rest avoided that regression.

The selected source passes **324 startup/neutral regressions with identical command, drift and setpoint traces**, plus a **672-case paired drive screen** against frozen installed source `aa9ae13`: small/full input, reversal, loss of input, steering, pushes, and replayed recorded sticks. There are **no additional falls** in those cases. Both versions fail the same 59 extreme cases (8 per ordinary profile, 9 with pushes, 10 in replay); some cases never become ready for commanded motion. This is a comparison, not a claim that every simulated plant is controllable.

A separate 192-case stop comparison also adds no falls (16 with either version). In its 170 common commanded, nonfallen cases, median travel over six seconds after center falls from **16.76 to 14.10 wheel radians (~16%)**. Median first near-zero speed changes only slightly, **3.64 to 3.60 seconds**, and recoil peaks decrease from **2.07 to 2.01 rad/s**. Calm-stop count increases from 123 to 125; median calm time within each settling subset increases from 4.20 to 4.80 seconds. This supports less coasting, not a claim of universally quicker complete settling. Physical testing must check the tradeoff.

The model varies gravity/drive coupling, resonant motor response, damping and latency. It uses the actual C++ helpers, but is planar and does not validate yaw, traction, contact, full arm inertia or real stopping distance. The physical run is smoother and stops sooner than several simulated cases. High-speed transient spectral power rises modestly in some model profiles; the next run should check that visible rocking has not returned. Small-input, pure-turn and small-push traces retain the baseline behavior.

Native checks cover high/low-speed braking, forward/reverse symmetry, unchanged low-speed trajectories and arms, input loss, arm bounds, and the calm hold gate. The braking-only combination passed nine native executables, 27 Python tests, radio C++/Lua checks, dashboard SHA checks, syntax/whitespace checks and the pinned ESP32 build. Final combined validation and OTA identity are recorded in the release evidence below. A PATH conflict initially selected a Python3.14 PlatformIO installation; the successful build uses the existing pinned Python3.13 PlatformIO environment. Credentials are included by a compiler path to the existing private header, never copied between worktrees.

## Reproduction and next test

Use `evidence/balance-drive-braking/analyze.py` for physical metrics and the plotted stops. `screen_ramps.py`, `screen_capture.py`, `screen_boost.py`, and `screen_mild_boost.py` preserve the parameter screens. Rejected experiments use archived public helpers through `experiment_model.py`; `final_screen.py` compares the chosen source with installed v4 source `aa9ae13`.

The next operator test should use small forward/back inputs followed by centered CH2, with enough clear travel space to compare stopping and recoil. The operator performs all movement. Download the saved run over Wi-Fi after both motor groups are disarmed, before another run replaces it. No USB cable is needed for normal telemetry or OTA.

## Installed combined release

This section records the first combined v5/v1 image, subsequently superseded by
[lowering v2](../evidence/ota-lowering-v2/README.md). Its hashes and slot are
historical; use [current state](progress/CURRENT.md) for the installed image.

Source **`43b1967dd03d26d8b2ccf6a698901c56272039d2`**, branch `codex/drive-braking`; application **1,194,224 bytes**, whole-file SHA-256 `a3b1e64c109bfc91eb43842769efebefbc9b32d16baea290de0af2f6b7a8b982`. Verified running **app1**, ESP embedded digest `bc1e158acac24fa08a9fb81b26933b00243baa71f67c106c9696135182f0a9b3`. [Release evidence](../evidence/balance-drive-braking/README.md) records the manifests, preflight, upload and postflight.

Final combined validation passed: **10 native executables, 27 Python tests, syntax/whitespace checks and the pinned ESP32 build**. Radio C++/Lua and dashboard checks passed; their source is unchanged by integration. The **81 lowering scenarios** were rerun successfully against the combined source. Production braking helper/config/model hashes match the completed simulation screen; lowering helper bytes match its reviewed source. The merged log feature mask is 4095.

Two Wi-Fi attempts aborted: the standard uploader lost its connection, then a paced upload reached a shorter 30-second response timeout. The old app0 identity and uptime were checked after both, and maintenance released. A slower 1KiB/50ms transfer with the standard 120-second timeout succeeded in **62.8 seconds**. The operator turned off the transmitter near the end of that transfer. Pacing, timeout and radio state all changed, so this does **not** isolate radio interference or powered CAN as the cause. Both unsuccessful attempts are retained in the evidence.

After reboot, all six motors were online, disabled, fresh and fault-free; the IMU reported no fault. The saved **2,981-sample v4 log re-downloaded with both CSV and raw wire bytes identical** to the pre-update capture. Application-only OTA did not rewrite the filesystem, partition table or settings. Calibration was not directly re-read through the web API, which does not expose those settings. No motor arming, balancing, driving or lowering was initiated by the release agent.

The private credentialed application/ELF are retained locally in `worktrees/drive-braking/artifacts/drive-braking-v5/release/` relative to the project root and excluded from Git. The previous Wi-Fi/OTA package remains at the project root `artifacts/wifi-ota/release/`; rollback uses that application through authenticated OTA after disarming and preserving newer logs. There is no automatic boot rollback.

The original driving evaluation used the usual stand-up routine, then brief
moderate forward/back requests followed by centered CH2. Very small requests
remain deliberately unchanged. For future testing, follow the
[current test guide](BALANCE_TESTING.md) and retrieve the log after disarming both
groups. CH11 lowering trials are currently on hold.
