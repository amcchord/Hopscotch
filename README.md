# Hopscotch

Firmware for a remote-controlled 4-wheel robot with two arms and an experimental self-balancing mode. Runs on an ESP32-S3, controls six brushless motors over CAN bus, and is driven with a RadioMaster GX12 transmitter over ELRS.

**Current firmware:** Installed source `c442e12` combines [CH11 lowering v7](docs/BALANCE_LOWER_V7_2026-09.md), the successful [fast-standing v2](docs/FAST_TIP_UP_2026-09.md), and [OTA progress/RC suspension](docs/WIFI_OTA.md). Lowering hands neutral driving back to stationary control, brakes after both arm contacts and tolerates brief rocking during support confirmation so arm return can continue. Ordinary driving and fast standing remain unchanged. Lowering v7 needs a manual trial. [Installed evidence](evidence/lowering-v7-integration/README.md), [current state](docs/progress/CURRENT.md).

Earlier [successful stand-ups](docs/BALANCE_STARTUP_RECOVERY_2026-09.md) and [driving trials](docs/BALANCE_DRIVE_TRIALS_2026-09-19.md) remain historical evidence. Their frozen packages and USB flash instructions are not the current update workflow.

![Robot Diagram](docs/RobotDiagram.png)

## Hardware

| Component | Details |
|-----------|---------|
| MCU | [M5Stack AtomS3R](https://docs.m5stack.com/en/core/AtomS3R) — ESP32-S3, dual-core 240 MHz, 8 MB flash, 8 MB PSRAM, 128x128 IPS display |
| CAN Adapter | [Atomic CAN Base](https://docs.m5stack.com/en/atom/Atomic%20CAN%20Base) (CA-IS3050G isolated transceiver) |
| Drive Motors | 4x Robstride RS05 — CAN IDs 10, 20, 30, 40 |
| Arm Motors | 2x Robstride RS00 — CAN IDs 1, 2 |
| Transmitter | [RadioMaster GX12](https://www.radiomasterrc.com/) running ELRS |
| Receiver | ELRS RX, CRSF protocol at 420 kbaud via Grove port |

All six motors communicate over CAN 2.0B extended frames at 1 Mbps. The Atomic CAN Base has no built-in termination resistor — you must add 120 ohm between CAN_H and CAN_L at the bus endpoints.

## Features

### Driving

Tank-style arcade mixing maps throttle and steering sticks to differential wheel speeds. All four drive motors run in **CSP (Cyclic Synchronous Position)** mode with a closed-loop rolling position horizon. A three-state machine (Idle, Driving, Braking) handles smooth acceleration, coast-down, and position hold. If the controller stops sending updates, the robot coasts to a stop within 3 seconds.

### Arms

Two arm motors are driven in rate mode — stick input is integrated into a position target clamped to a configurable range. Arms support calibration, programmable positions (nudge/jump), and are used by the balance controller to tip the robot upright.

### Self-Balance Mode

An optional balancing mode activated via RC switch combinations. At the start of the tip-up sequence the rear wheels are switched from CSP position mode to Robstride Speed mode (while the robot is still static on all fours): a 200 Hz complementary-filter + PD control loop commands wheel velocity directly, while the front wheels hold position in CSP. A 50 Hz outer cascade (position P -> velocity PI) adjusts the tilt setpoint to hold station. During arm return, sustained rising wheel speed triggers a short, bounded boost to the existing equilibrium-learning integral. Recovery first targets zero wheel speed, retains its learned correction through the arm handoff, then captures the settled location for position holding. Starts without a recovery event retain the original position reference.

The balance point is self-calibrating: an arm-position-to-balance-point curve provides the shape, every settled capture re-zeros its absolute level, and a persisted trim learns residuals from recent calm intervals across runs. Movement of the sensor relative to the chassis during a run remains an unresolved physical uncertainty. The arms double as a second balance actuator: from their top-dead-center stance they throw against pushes through an engagement lifecycle (fast attack, one recoil handoff, calm-gated re-arm) intended to limit repeated self-triggering during disturbances, with a full-stop emergency throw when the wheels saturate. Safety systems include tilt/rate/saturation aborts, stale-feedback abort, CAN bus-off recovery, motor-side CAN watchdogs, a two-stage dead-man on the control heartbeat, and level-based disarm enforcement.

The full design record -- 30+ instrumented runs and 50+ lessons in the current Speed-mode campaign -- lives in `telemetry_logs/TUNING_HISTORY.md`. Offline tooling: `scripts/fit_balance_model.py --speed-only` fits the plant model from telemetry, and `scripts/balance_sim.py` is an approximate planar simulator (capture/arm-return scenarios, push response, control-task stall injection) used to screen controller changes before robot time. It reads current firmware constants but does not simulate full self-righting, contacts, slip, sensor mounting movement, or the actual task scheduler. The repeatable flash, safety, test, marker, download, and analysis workflow is in [`docs/BALANCE_TESTING.md`](docs/BALANCE_TESTING.md).

### Wi-Fi telemetry and OTA

The robot joins its configured Wi-Fi network and serves an embedded dashboard
at [hopscotch.local](http://hopscotch.local/). The last tested DHCP address was
[192.168.1.172](http://192.168.1.172/); use the display's address if it changes.
[Setup, API, safety design and recovery](docs/WIFI_OTA.md).

- Live telemetry offered at 10 Hz: pose, motor feedback, RC state, timing and memory
- Checksummed saved-run downloads without a USB cable
- Authenticated application OTA into the inactive firmware slot, preserving settings and calibration
- Control-owned disarmed interlock for flash operations and Wi-Fi association
- Authenticated emergency-disarm request; the radio remains the motion-control path

The dashboard is included in the application, so OTA also updates the UI.
Configure the gitignored `src/network_secrets.h` before building. Settings and
calibration continue to live in LittleFS; configuration/tuning uses the USB console.

### Serial Console

A serial CLI at 115200 baud provides debug output every 2 seconds (loop timing, per-motor feedback, drive states, CAN health) and supports balance-gain tuning between attempts. Active runs accept only disarm, status, notes and markers, so the saved gain snapshot remains meaningful.

## Transmitter Setup

The firmware uses the following default functions. GX12 sources are from the
[September 19 radio audit](docs/RADIO_TELEMETRY.md); later radio adjustments and
persisted robot mappings must be checked before a physical test.

| Function | Channel | GX12 Control |
|----------|---------|-------------|
| Steering | CH1 | Right stick X |
| Throttle | CH2 | Right stick Y |
| Arm Speed | CH5 | SE |
| Arm Nudge | CH4 | Input Rud |
| Tip-up speed | CH6 | SB; high = experimental fast, center/low = regular slow |
| Balance Select | CH7 | SC |
| Arms Arm/Disarm | CH9 | SA |
| Drive Arm/Disarm | CH10 | SD |
| Execute / stand-up / supported lowering / calibration trigger | CH11 | SG |
| Arm-position cycle / balance event marker | CH12 | SH |
| Left Arm setting | CH13 | P1; unused by sequential arm controller |
| Right Arm setting | CH14 | P2; unused by sequential arm controller |

Ground-drive and several trigger mappings are stored in robot settings;
standing-drive CH1/CH2 and arm-speed/nudge CH5/CH4 are fixed in firmware. Web
configuration is disabled; use the USB console for supported configuration and
diagnostics. Signal loss is detected if no valid CRSF frame arrives within 500 ms.

The [CH6 fast tip-up](docs/FAST_TIP_UP_2026-09.md) targets roughly
three seconds for lift/capture, with the usual CH11 trigger and unchanged slow
mode. Its v2 support-release correction completed an operator-confirmed successful stand-up; substantial catch/recoil travel remains for later tuning.

## Building and Updating over Wi-Fi

### Prerequisites

- Python 3 and [PlatformIO](https://platformio.org/) for a source build
- Local `src/network_secrets.h` configured as described in the [operating guide](docs/WIFI_OTA.md#local-configuration-and-access)
- Computer and robot on the same LAN; USB is optional for normal updates and telemetry

### Build

```bash
./scripts/build.sh
```

### Update the Application

Use the [single-command frozen-package procedure](docs/WIFI_OTA.md#update-the-firmware).
The CLI checks the manifest, archives the saved run, performs a paced upload,
and verifies the running image, disarmed health and unchanged saved telemetry.
Reuse completed release checks and the exact package; rebuilding is for changed
firmware. Support and disarm the robot, lower both arm switches and release CH11.
Motor power may remain on while disabled. Switch the transmitter off for the
currently demonstrated reliable upload path; transmitter-on reliability is
being investigated separately.

The dashboard can also upload the same application file. **Do not run
`uploadfs`, even after UI changes:**
`data/network.html` is embedded in the application, and LittleFS holds settings,
calibration and the saved run. USB recovery is [slot-aware](docs/WIFI_OTA.md#recovery);
OTA does not provide automatic rollback from a boot-broken application.

### Telemetry

Use the dashboard for live monitoring. After each disarmed run, download the
full onboard capture and analyze the resulting file:

```bash
.venv/bin/python scripts/robot_wifi.py log
.venv/bin/python scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details
```

The downloader creates validated `.csv` and original `.wire` files. Analysis is
a separate command; replace `<run>` with the downloaded filename. If mDNS is
unavailable, place `--host http://192.168.1.172` before `status`, `log` or `ota`.

### USB Console

Runtime gain tuning, calibration, detailed console diagnostics and log clearing
still use USB. Wi-Fi does not provide a motion or tuning command console.

```bash
./scripts/monitor.sh
```

## Software Architecture

The firmware splits the two ESP32-S3 cores into a **control core** and a **comms core**:

- **Core 1 (control)** — a stall-forensics sentinel (prio 24), the 200 Hz balance task (prio 18: direct IMU acquisition, complementary filter, PD loop, wheel speed commands), and the control task (prio 12, 5 ms cadence with 50 Hz state-machine/drive/arm updates: serial console, CRSF, CAN, arming, failsafe). The Arduino `loopTask` (prio 1) keeps only the display and debug output.
- **Core 0 (comms)** — Wi-Fi, lwIP, Arduino network events, `async_tcp` (priority 3), and the network task (priority 2). Control publishes bounded RAM snapshots; all network formatting happens here. OTA, saved-file reads and Wi-Fi association require a disarmed interlock because core separation alone cannot prevent cross-core stalls.

Control-task gaps over 100 ms are recorded as forensic events (profiler section attribution plus a sentinel-gap discriminator). The profiler is reset at balance entry and frozen at exit, so `bal log` reports only the physical run rather than idle-time download activity.

Balance telemetry schema v7 (same 240-byte samples as v4/v5/v6) records up to 120 seconds at 50 Hz, including tip-up, in a 1,440,000-byte PSRAM buffer. It includes 200 Hz timing/saturation aggregates, raw and filtered IMU signals, setpoints, commands, CAN feedback, wheel/arm motion and torque, pilot intent/planned arm assistance, power, safety-exit reason and event markers. It is persisted as a checksummed binary only after balance ends and both groups are disarmed. `scripts/robot_wifi.py log` exports validated CSV over Wi-Fi; `scripts/save_telemetry.sh` remains the USB fallback. Live dashboard JSON schema 1 is separate from the saved-log schema.

### Module Map

| File | Responsibility |
|------|---------------|
| `main.cpp` | Setup, main loop, arming, failsafe, debug output |
| `config.h` | Pin definitions, timing constants, tuning parameters |
| `settings.h/cpp` | Persistent JSON configuration on LittleFS |
| `robstride.h/cpp` | Low-level Robstride CAN protocol driver (TWAI) |
| `motor_manager.h/cpp` | Six-motor management, arming FSM, feedback routing |
| `crsf.h/cpp` | CRSF packet parser, channel extraction, link detection |
| `drive_controller.h/cpp` | Arcade mixing, rolling position horizon, braking |
| `arm_controller.h/cpp` | Rate-mode arm control with calibration |
| `balance_controller.h/cpp` | Balance state machine and 200 Hz PD loop |
| `display.h/cpp` | 128x128 sprite-based status display |
| `web_server.h/cpp` | Core-0 networking, WebSocket telemetry, saved-log export and OTA |
| `network_snapshot.h` / `network_safety.h` | Fixed-size snapshot contract and control-owned maintenance gate |
| `data/network.html` | Dashboard embedded in the application |
| `scripts/robot_wifi.py` | LAN status, validated log download, disarm, reconnect and verified OTA |

### Timing

| Task | Rate |
|------|------|
| Control loop | 50 Hz (5 ms task cadence) |
| Balance loop | 200 Hz |
| Display refresh | 25 fps (5 fps while balancing) |
| WebSocket telemetry | 10 Hz offered, including while balancing; slow clients drop frames |
| CRSF telemetry uplink | 5 Hz |
| Stall sentinel | 100 Hz |

## Configuration

Compile-time defaults live in `src/config.h`. Parameters are retained in `settings.json` on LittleFS. The previous web mutation routes are disabled; use the USB console for tuning and calibration.

Balance gains can be tuned over USB between attempts without reflashing.
These are command examples, not a tuning preset; inspect `bal status` before
changing the current gains:

```
bal kp 2.0      # inner PD: rad/s wheel speed per deg of angle error
bal kd 0.08     # inner PD: rad/s per deg/s of roll rate
bal dkp 0.05    # outer: target return velocity per rad of drift
bal vkp 2.2     # outer: high-slope setpoint response per rad/s of velocity error
bal vki 0.35    # outer: integral gain (the single equilibrium learner)
bal note test-name  # tag the next/current telemetry capture
bal mark        # add a numbered event marker (CH12 does this while balancing)
```

See `docs/PROJECT.md` for the full Robstride CAN protocol reference and detailed documentation.

## License

MIT
