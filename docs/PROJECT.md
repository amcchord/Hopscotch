# Hopscotch Robot Controller

## What This Is

Hopscotch is firmware for a remote-controlled 4-wheeled robot with two arms. It runs on an M5Stack AtomS3R (ESP32-S3) and controls 6 Robstride brushless motors over CAN bus. The robot is driven via an ELRS radio receiver and exposes a web dashboard for diagnostics and configuration.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | M5Stack AtomS3R — ESP32-S3-PICO-1-N8R8, 240 MHz dual-core, 8 MB flash, 8 MB PSRAM, 128x128 IPS display |
| CAN Adapter | Atomic CAN Base (CA-IS3050G isolated transceiver), mounted on bottom pins |
| CAN Pins | **GPIO5 = TWAI TX**, **GPIO6 = TWAI RX** (from the MCU's perspective) |
| ELRS Receiver | CRSF protocol at 420000 baud on **GPIO1 (RX)**, **GPIO2 (TX)** via Grove port |
| Drive Motors | 4x Robstride RS05 — CAN IDs 10 (front right), 20 (back right), 30 (back left), 40 (front left) |
| Arm Motors | 2x Robstride RS00 — CAN IDs 1 (left arm), 2 (right arm) |
| WiFi | Joins a configurable network (default: SvensHaus / montreal19) |

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

**Data payload:** 4x uint16, big-endian:
| Bytes | Field | Scaling |
|-------|-------|---------|
| 0–1 | Position | uint16 over [−4π, +4π] rad |
| 2–3 | Velocity | uint16 over [−50, +50] rad/s (universal, not per-model) |
| 4–5 | Torque | uint16 over [−17, +17] Nm |
| 6–7 | Temperature | uint16 × 0.1 = °C |

### Motor Polling / Keep-Alive

Sending a **motion control frame (type 0x01) with all-zero data and zero torque** acts as a status ping. The motor responds with a type 0x02 feedback frame without moving. This is the recommended way to poll motor status without enabling the motor.

### Position Control via Parameter Writes

Position mode is set using parameter writes (type 0x12), not a dedicated CAN ID:
1. Write `RUN_MODE` (0x7005) = 1 (position mode)
2. Write `SPEED_LIMIT` (0x7017) = desired max speed (float, rad/s)
3. Write `TARGET_POSITION` (0x7016) = target position (float, rad)

### Motor Specs

| Model | Max Torque | Max Speed | KP Range | KD Range |
|-------|-----------|-----------|----------|----------|
| RS00 | 17 Nm | 50 rad/s | 500 | 5.0 |
| RS05 | 17 Nm | 33 rad/s | 500 | 5.0 |

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

The CRSF protocol provides 16 channels of 11-bit data (raw range 172–1811, center 992). The channel-to-function mapping is configurable via the web UI. Defaults:

| Function | Channel (0-indexed) |
|----------|-------------------|
| Steering | 0 |
| Throttle | 1 |
| Drive Arm/Disarm | 9 |
| Arms Arm/Disarm | 4 |
| Left Arm | 12 |
| Right Arm | 13 |

Signal loss is detected if no valid CRSF frame arrives within 500 ms.

## Software Architecture

### Timing

| Task | Rate | Period |
|------|------|--------|
| Control loop | 50 Hz | 20 ms |
| Display refresh | 25 fps | 40 ms |
| WebSocket telemetry | 10 Hz | 100 ms |

### Module Map

| File | Responsibility |
|------|---------------|
| `main.cpp` | Setup, 50 Hz loop, arming logic, failsafe, debug telemetry |
| `config.h` | Pin definitions, timing constants, motor specs |
| `settings.h/.cpp` | Persistent JSON config on LittleFS |
| `robstride.h/.cpp` | Low-level CAN protocol driver (TWAI) |
| `motor_manager.h/.cpp` | 6-motor management, arming, feedback routing, direction reversal |
| `crsf.h/.cpp` | CRSF packet parser, channel extraction, link detection |
| `drive_controller.h/.cpp` | Arcade mixing, closed-loop rolling horizon, braking state machine |
| `arm_controller.h/.cpp` | Rate-mode arm control |
| `display.h/.cpp` | 128x128 sprite-based status display |
| `web_server.h/.cpp` | AsyncWebServer + WebSocket |
| `data/` | Web UI files (HTML/CSS/JS) served from LittleFS |

### Build

This is a PlatformIO project targeting `esp32-s3-devkitc-1` with Arduino framework.

```bash
pio run                      # Build
pio run --target upload      # Upload firmware (uses esp-builtin JTAG)
pio run --target uploadfs    # Upload web UI to LittleFS
pio device monitor           # Serial monitor at 115200
```

Upload protocol is `esp-builtin` (JTAG), not esptool, because the AtomS3R's USB-JTAG serial port is unreliable with esptool baud rate changes while firmware is running.

## Web UI

Served on port 80 at the device's WiFi IP address.

**REST API:**
- `GET /api/settings` — current settings JSON
- `POST /api/settings` — update settings
- `POST /api/disarm` — emergency disarm all motors
- `POST /api/change-can-id` — change a motor's CAN ID (`{"oldId": N, "newId": M}`)
- `POST /api/reset-settings` — factory reset

**WebSocket** (`/ws`): 10 Hz JSON telemetry with motor positions/velocities/torques/temps, all 16 RC channels, arming state, link quality, and WiFi status.

## Key Lessons Learned

1. **CAN ID bit order matters.** The Robstride protocol puts motor ID in bits 0–7 and master ID in bits 8–15. Getting these swapped results in TX succeeding (other nodes still ACK at the hardware level) but no application-level responses.

2. **Motion control ping for discovery.** Robstride motors don't broadcast on their own. Send a type 0x01 frame with all-zero data to poll status without enabling the motor.

3. **The Atomic CAN Base has no termination resistor.** You must add 120 Ω between CAN_H and CAN_L at the bus endpoints.

4. **AtomS3R upload protocol.** Use `upload_protocol = esp-builtin` (JTAG) in `platformio.ini`. The default esptool protocol fails to change baud rate reliably when firmware is already running on the USB-JTAG port.

5. **Partition table.** The AtomS3R has 8 MB flash. Use `default_8MB.csv`, not `default_16MB.csv`.
