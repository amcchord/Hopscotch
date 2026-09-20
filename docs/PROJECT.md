# Hopscotch Robot Controller

## What This Is

Hopscotch is firmware for a remote-controlled 4-wheeled robot with two arms. It runs on an M5Stack AtomS3R (ESP32-S3) and controls 6 Robstride brushless motors over CAN bus. ELRS remains the motion-control link. The LAN dashboard provides live telemetry, saved-run downloads, disarm requests and application OTA; tuning and calibration use the USB console.

Start with the [Wi-Fi / OTA operating guide](WIFI_OTA.md), [balance test procedure](BALANCE_TESTING.md) and [current installed release](progress/CURRENT.md). Dated balance reports describe historical releases, not current upload instructions.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | M5Stack AtomS3R — ESP32-S3-PICO-1-N8R8, 240 MHz dual-core, 8 MB flash, 8 MB PSRAM, 128x128 IPS display |
| CAN Adapter | Atomic CAN Base (CA-IS3050G isolated transceiver), mounted on bottom pins |
| CAN Pins | **GPIO5 = TWAI TX**, **GPIO6 = TWAI RX** (from the MCU's perspective) |
| ELRS Receiver | CRSF protocol at 420000 baud on **GPIO1 (RX)**, **GPIO2 (TX)** via Grove port |
| Drive Motors | 4x Robstride RS05 — CAN IDs 10 (front right), 20 (back right), 30 (back left), 40 (front left) |
| Arm Motors | 2x Robstride RS00 — CAN IDs 1 (left arm), 2 (right arm) |
| WiFi | Joins the configured network; credentials are in gitignored `src/network_secrets.h`. |

See `docs/RobotDiagram.png` for the physical layout. The robot has four wheels in a rectangular arrangement and two arms mounted at the front.

## Robstride CAN Protocol — Critical Details

All Robstride motors communicate over CAN 2.0B **extended frames (29-bit ID)** at **1 Mbps**.

### 29-bit Extended ID Layout

```
Bits [28:24]  Communication type (5 bits)
Bits [8:23]   Data field (16 bits — usage depends on command type)
Bits [0:7]    Motor CAN ID (8 bits)
```

For **non-motion commands** (enable, stop, param read/write):
```
(commType << 24) | (masterHostId << 8) | motorId
```

For **motion control** (type 0x01):
```
(0x01 << 24) | (torque_u16 << 8) | motorId
```

The host/master ID used by this project is **0xFD**.

### Communication Types

| Type | Value | Description |
|------|-------|-------------|
| ObtainID | 0x00 | Broadcast device discovery |
| Motion Control | 0x01 | MIT-style composite control (pos, vel, kp, kd + torque in ID) |
| Feedback | 0x02 | Motor status response |
| Enable | 0x03 | Enable motor |
| Stop | 0x04 | Stop/disable motor |
| Set Zero | 0x06 | Set current position as mechanical zero |
| Set CAN ID | 0x07 | Change motor's CAN ID (requires power cycle) |
| Read Param | 0x11 | Read single parameter |
| Write Param | 0x12 | Write single parameter |
| Fault | 0x15 | Fault feedback |

### Feedback Frame Format (Type 0x02 Response)

**CAN ID fields:**
- Bits [22:23]: Run mode (0=MIT, 1=Position, 2=Speed, 3=Current)
- Bits [16:21]: Error code (6 bits, 0 = no error)
- Bits [8:15]: Motor's own CAN ID
- Bits [0:7]: Destination host ID (0xFD)

**Data payload:** 4x uint16, big-endian (per RS00 User Manual section 4.1.2):
| Bytes | Field | Scaling |
|-------|-------|---------|
| 0–1 | Position | uint16 over [−4π, +4π] rad |
| 2–3 | Velocity | uint16 over [−33, +33] rad/s |
| 4–5 | Torque | uint16 over [−14, +14] Nm |
| 6–7 | Temperature | uint16 × 0.1 = °C |

### Motor Polling / Keep-Alive

Sending a **motion control frame (type 0x01) with all-zero data and zero torque** acts as a status ping. The motor responds with a type 0x02 feedback frame without moving. This is the recommended way to poll motor status without enabling the motor.

### Position Modes

The motor has **two** position modes (per RS00 User Manual):

- **Mode 1 (PP - Profile Position)**: Motor generates internal trapezoidal trajectory. Speed via `vel_max` (0x7024), acceleration via `acc_set` (0x7025). **Does NOT support changing speed during operation.** Zero calibration is blocked.
- **Mode 5 (CSP - Cyclic Synchronous Position)**: Real-time position updates from host. Speed limit via `limit_spd` (0x7017), position via `loc_ref` (0x7016). **This is what Hopscotch uses for drive motors.** Zero calibration is supported.

### CSP Position Control Sequence

1. Stop motor (type 0x04, clear fault)
2. Set `RUN_MODE` (0x7005) = 5 (CSP mode)
3. Enable motor (type 0x03)
4. Set zero position (type 0x06) — supported in CSP mode
5. Write `SPEED_LIMIT` (0x7017) = desired max speed (float, rad/s)
6. Write `TARGET_POSITION` (0x7016) = target position (float, rad)

### Key Parameter Addresses

| Address | Name | Description | R/W |
|---------|------|-------------|-----|
| 0x7005 | run_mode | 0=MIT, 1=PP, 2=Speed, 3=Current, 5=CSP | W/R |
| 0x7016 | loc_ref | Position target (rad) | W/R |
| 0x7017 | limit_spd | CSP mode speed limit (0–33 rad/s) | W/R |
| 0x7018 | limit_cur | Current limit (0–16A) | W/R |
| 0x700A | spd_ref | Speed mode target (−33 to 33 rad/s) | W/R |
| 0x701E | loc_kp | Position Kp (default 40) | W/R |
| 0x701F | spd_kp | Speed Kp (default 6) | W/R |
| 0x7019 | mechPos | Load mechanical angle (rad) | R |
| 0x701B | mechVel | Load speed (rad/s) | R |

### Motor Specs

| Model | Max Torque | Max Speed | No-Load Speed | Gear Ratio |
|-------|-----------|-----------|---------------|------------|
| RS00 | 14 Nm | 33 rad/s | 315 RPM | 10:1 |
| RS05 | 14 Nm | 33 rad/s | 315 RPM | 10:1 |

## Driving Model

### Tank-Style Arcade Mixing

Two RC channels (throttle + steering) are mixed into differential wheel speeds:
- `left_speed = throttle + steering`
- `right_speed = throttle - steering`

Left-side motors (IDs 30, 40) have their direction reversed in software to match the physical mounting direction. Default max speed is 33 rad/s (~315 RPM), the RS05 maximum.

### Closed-Loop Rolling Position Horizon

All drive motors run in **position control mode** with closed-loop velocity monitoring. Each motor has a state machine with three states: **Idle**, **Driving**, and **Braking**.

**Driving (stick active):**

1. Read the motor's current position and velocity from CAN feedback
2. Compute horizon distance using the **greater of commanded and actual speed**: `max(|cmd|, |actual|) * 3.0s`. This prevents the target from falling behind a fast-moving motor, which would trigger unwanted deceleration.
3. Set target position = `current_pos + direction * horizon_distance`
4. Set speed limit = commanded speed (proportional to stick deflection)
5. If the controller stops updating, the motor reaches its last target and **stops within 3 seconds**

**Braking (stick returns to center):**

1. On transition from Driving to stick-at-center, enter Braking state
2. Set target position slightly ahead in the current travel direction (`actual_vel * 0.5s`) to allow smooth coast-down
3. Ramp the speed limit down at 2 rad/s per second
4. Keep updating the deceleration target based on actual velocity feedback
5. Once actual velocity drops below 0.5 rad/s, transition to Idle (position hold)

**Idle (stopped):**

Target is held at current position with a low speed limit (0.5 rad/s).

**Emergency stop / signal loss:**

All targets snap to current position with low speed limit. Braking state is cleared.

### Debug Serial Output

Every 2 seconds the firmware prints a comprehensive status block including:
- Control loop timing (average, max, overruns)
- Per-motor feedback (position, velocity in rad/s and RPM, torque, temperature, errors)
- Per-motor drive controller state (Idle/Drive/Brake), commanded vs actual speed, target vs actual position, speed error
- CAN bus health (TX/RX counts, error counters)

State transitions (arm/disarm, drive/brake/idle) are logged as they happen.

### Arm Control

Arms are controlled in rate mode: the RC stick input is integrated into a position target, clamped to a configurable range. On signal loss the arms hold their current position.

## ELRS / CRSF

The CRSF protocol provides 16 channels of 11-bit data (raw range 172–1811, center 992). Ground-drive and several trigger mappings are stored in settings; standing-drive CH1/CH2 and arm-speed/nudge CH5/CH4 are fixed. Web settings routes are disabled. Source defaults below use zero-based indices; operator docs use CH1–CH16. The [GX12 audit](RADIO_TELEMETRY.md) records the saved handset sources, which may differ after radio adjustments.

| Function | Channel (0-indexed) |
|----------|-------------------|
| Steering | 0 |
| Throttle | 1 |
| Drive Arm/Disarm | 9 |
| Arms Arm/Disarm | 8 |
| Left Arm | 12 |
| Right Arm | 13 |

Signal loss is detected if no valid CRSF frame arrives within 500 ms.

## Balance Telemetry and Test Capture

Balance mode automatically records one 120-second run at 50 Hz. The PSRAM sample contains the state-machine and outer-loop values plus windowed evidence from all 200 Hz PD ticks, including maximum inner-loop interval, tick count, saturation count, raw accelerometer angle/gyro/acceleration norm, unclamped and applied wheel commands, every setpoint component, CAN feedback age, rear-wheel torque, arm-assist state and torque, yaw correction, and cached power data.

The run also captures its actual live-tuned gains, stored trim at entry, compile-time control constants, firmware build time, operator note, numbered event markers, end reason, sample count, and checksum. Schema v4 adds pilot intent and planned arm assistance to the earlier layout: 6,000 samples × 240 bytes = 1,440,000 bytes in PSRAM. The file is written to `/bal_log.bin` only after balance and arm return are idle and both motor groups are disarmed. Learned-trim persistence is deferred to the same safe service.

`scripts/robot_wifi.py log` is the default download path. With the control-owned maintenance gate granted, the device validates the saved binary and copies its CSV export into bounded PSRAM, then releases the gate before sending it. The host verifies schema, checksums, row count and samples and retains cleaned `.csv` plus original `.wire` data. Earlier stored schemas keep their original metadata. `bal log` and `scripts/save_telemetry.sh` provide the USB fallback. The run-scoped profiler freezes at balance exit; transfer activity is excluded.

Use USB `bal note <text>` before a run, or keep operator observations alongside the downloaded CSV when untethered. While balancing, CH12 is a non-actuating event marker and its normal arm-position trigger is suppressed. Wi-Fi has no note, gain or motion command endpoint. Live telemetry is best-effort JSON schema 1, offered at 10 Hz; it does not replace the complete 50 Hz capture. The full test, capture and analysis procedure is in [BALANCE_TESTING.md](BALANCE_TESTING.md).

## Software Architecture

The current application also includes [progressive reference braking](BALANCE_DRIVE_BRAKING_2026-09.md),
[ground-drive ownership gating](GROUND_DRIVE_2026-09.md) and
[experimental CH11 forward-fall/catch v2](BALANCE_LOWER_2026-09.md). New captures
use schema 4 with feature flags 8191; the sample layout remains 240 bytes.
Further lowering trials are on hold after the failed v2 attempt; see
[current state](progress/CURRENT.md).

### Timing

| Task | Rate | Period |
|------|------|--------|
| Control task / state-machine updates | 200 Hz / 50 Hz | 5 ms / 20 ms |
| Balance PD loop | 200 Hz | 5 ms |
| Display refresh | 25 fps; 5 fps balancing | 40 ms; 200 ms balancing |
| WebSocket telemetry | 10 Hz offered; best-effort delivery | 100 ms offer interval |

Balance (priority 18), control (12), stall sentinel (24), and display/debug (1)
run on core 1. Wi-Fi/lwIP, Arduino network events, async TCP (3), and the network
task (2) run on core 0. Control publishes a fixed-size snapshot through a
one-element overwrite queue. Networking does not allocate or format on the
control core. Flash access and Wi-Fi association still require disarmed
maintenance because they can affect both cores; see [control isolation](WIFI_OTA.md#control-isolation).

### Module Map

| File | Responsibility |
|------|---------------|
| `main.cpp` | Task setup, control owner, arming logic, failsafe, debug telemetry |
| `config.h` | Pin definitions, timing constants, motor specs |
| `settings.h/.cpp` | Persistent JSON config on LittleFS |
| `robstride.h/.cpp` | Low-level CAN protocol driver (TWAI) |
| `motor_manager.h/.cpp` | 6-motor management, arming, feedback routing, direction reversal |
| `crsf.h/.cpp` | CRSF packet parser, channel extraction, link detection |
| `drive_controller.h/.cpp` | Arcade mixing, closed-loop rolling horizon, braking state machine |
| `arm_controller.h/.cpp` | Rate-mode arm control |
| `balance_controller.h/.cpp` | Balance state machine, 200 Hz controller and saved run |
| `display.h/.cpp` | 128x128 sprite-based status display |
| `web_server.h/.cpp` | Core-0 HTTP/WebSocket, saved-log export, Wi-Fi and OTA |
| `network_snapshot.h` / `network_safety.h` | RAM snapshot contract and maintenance request/grant interlock |
| `data/network.html` | Active dashboard embedded in `firmware.bin`; other `data/` assets are legacy |
| `scripts/robot_wifi.py` | Host status, validated log download and application OTA |
| `scripts/patch_asynctcp.py` | Source-hash-checked fixes for the pinned TCP library |

### Build and OTA Update

PlatformIO environment `m5stack-atoms3r` targets `esp32-s3-devkitc-1` with
Arduino 2.0.16 / espressif32 6.7.0 and an 8 MiB partition table. Configure the
ignored `src/network_secrets.h` before a source build. Run `./scripts/build.sh`
while developing; at release freeze the consolidated checks include the build.
Reuse the resulting validated package for deployment without rebuilding it.

Use the [single-command update procedure](WIFI_OTA.md#update-the-firmware). The
host helper checks the manifest, backs up the run, uploads with pacing, and
verifies the running image, disarmed health and saved-run preservation. OTA
writes only the inactive application slot; it also
updates the dashboard. **Do not run `uploadfs`**: LittleFS contains settings,
calibration and the saved run. USB remains available for console work and
[recovery](WIFI_OTA.md#recovery); legacy app0-only upload scripts are not general
recovery tools after OTA. A boot-broken image has no automatic rollback.

## Web UI

Open [hopscotch.local](http://hopscotch.local/) on the configured LAN, or use the
IP on the display. The embedded UI shows pose, motors, RC channels, timing,
memory and maintenance blockers. Read-only telemetry and firmware information
are public on the trusted LAN; log export, disarm, reconnect and OTA require the
device bearer token. The browser holds it only in the current page.

The [API reference](WIFI_OTA.md#http-and-websocket-api) is authoritative.
`/api/settings`, `/api/change-can-id` and `/api/reset-settings` return **410**,
including the old settings GET/export. There is no Wi-Fi arm, steering, balance,
calibration or tuning endpoint. Use USB for supported console operations.

## Key Lessons Learned

1. **CAN ID bit order matters.** The Robstride protocol puts motor ID in bits 0–7 and master ID in bits 8–15. Getting these swapped results in TX succeeding (other nodes still ACK at the hardware level) but no application-level responses.

2. **Motion control ping for discovery.** Robstride motors don't broadcast on their own. Send a type 0x01 frame with all-zero data to poll status without enabling the motor.

3. **The Atomic CAN Base has no termination resistor.** You must add 120 Ω between CAN_H and CAN_L at the bus endpoints.

4. **AtomS3R recovery transport.** Normal updates use OTA. The configured USB upload transport is `esp-builtin` (JTAG); esptool baud-rate changes have been unreliable while firmware runs on the USB-JTAG port. Determine the selected OTA slot before any USB recovery write.

5. **Partition table.** The AtomS3R has 8 MB flash. Use `default_8MB.csv`, not `default_16MB.csv`.
