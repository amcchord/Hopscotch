# Balance Mode Test Guide

This guide is the repeatable procedure for collecting the data needed to tune Hopscotch's balance mode. The firmware captures up to 120 seconds, including tip-up, in PSRAM, saves it to LittleFS after balance ends and **both drive and arms are disarmed**, and exports a checksummed CSV over USB. Capture reaching its limit does not stop the robot; end initial tests before that point to retain the outcome.

The [September review](BALANCE_REVIEW_2026-09.md) documents the evidence, changes, rejected experiments and offline validation. The candidate has a documented device-flash/check session; physical balancing is still pending. Read the review addendum/current state before repeating any upload.

## Safety and Test Area

- Use a clear, level area with several meters of travel in both directions.
- Keep a spotter at the robot and keep the drive-disarm control immediately reachable.
- Start with light taps. Run one genuinely large disturbance per test so the cause and recovery remain unambiguous.
- Before requesting `bal log` or starting another run, end balance, lower both drive and arm switches, and wait for the saved-log message. Keep power on until the save and download finish.
- Do not intentionally create CAN, power, or control-task failures with an upright robot. The firmware records naturally occurring failures without requiring destructive fault injection.

Record any mechanical variables that changed: floor surface, tire condition, battery position/state of charge, payload, arm calibration, or chassis work. Those variables move the physical equilibrium and matter as much as software gains.

## Flashing This Test Build

Connect USB with both motor groups disarmed. Before flashing, download the previous run and retain `cal status`, `bal status`, and the settings export from the web dashboard/API. Take a device flash backup where the interface permits; the packaged rollback is a rebuild of source baseline `e8b1280`, not a readback of the current device.

The prepared files and hashes are in `artifacts/balance-candidate/`; its `FLASHING.md` describes the frozen image. For a source rebuild, use:

```bash
./scripts/build.sh
./scripts/upload.sh
```

This test round does **not** require `uploadfs`. Avoiding `uploadfs` preserves the existing LittleFS settings, calibration, and saved telemetry. After flashing, open the monitor:

```bash
./scripts/monitor.sh
```

At boot, confirm that the console reports a 1,320,000-byte balance log buffer in PSRAM and that all six motors come online without faults. While disarmed, run `cal status` and `bal status`; verify retained calibration, plausible tilt/rate, fresh IMU age (normally a few milliseconds), no latched fault, and at least 200 ms of continuous healthy samples. Check the disarm/rearm behavior while supported on the floor. Do not force-engage a flat robot.

Hardware and calibration are unchanged from July. Do not recalibrate or reset trim merely to install this firmware. Keep the sensor mounting consistent; record any movement.

## Commands and RC Markers

| Action | Command/control | Behavior |
|---|---|---|
| Tag a run | `bal note baseline-hard-floor` | Stores up to 63 characters in the next/current log |
| Show controller state | `bal status` | Shows gains, setpoint state, buffered samples, and pending-save state |
| Mark a disturbance | Press CH12 while balance is active | Increments the `marker` column on the same 50 Hz control tick; CH12 arm-home behavior is suppressed while balancing |
| Serial marker | `bal mark` | Equivalent marker for bench tests |
| Download latest run | `./scripts/save_telemetry.sh --label baseline` | Requests `bal log`, validates schema/checksum/row count, saves CSV, and prints analysis |
| Delete latest run | `bal log clear` | Requires inactive balance and both motor groups disarmed; download first |

Set gains between attempts. While balance is active the serial console accepts only `disarm`, `disarm arms`, `bal status`, `bal note` and `bal mark`. After serial/web disarm, lower both RC arm switches before attempting to rearm. A web response means the disarm was requested; the control task executes it.

Press CH12 immediately before each intentional push. Wait for the robot to settle and for the arm-assist lifecycle to return to READY before the next push. The `marker` value is cumulative, so each transition identifies a new test event without adding serial traffic to the live control path.

## Recommended Test Ladder

Download the log after **every** run. If a run falls or needs a hand stop, end it immediately and save that log before trying again.

1. **Stand-up baseline** — tag `standup-baseline`; perform a normal single-trigger tip-up; do not touch the robot for 30 seconds. Measure stand-up travel, arm-return behavior, stationary sway, equilibrium learning, and return-to-origin speed.
2. **Small disturbances** — tag `small-taps`; after full settle, mark and apply three light pushes in each direction. Allow a complete recovery between events.
3. **Medium disturbances** — tag `medium-taps`; repeat with firmer pushes, alternating direction. Stop escalating if wheel command, travel, or arm motion looks uncontrolled.
4. **Recoil/lifecycle test** — tag `recoil`; use one marked backward push large enough to deploy the arms, then let the robot handle the natural forward recoil without a second touch. Repeat in the opposite direction in a separate run.
5. **Station keeping** — tag `station-60s`; leave the robot untouched for 60 seconds. This isolates low-frequency sway, drift return, yaw divergence, and trim convergence.
6. **Repeatability** — repeat the baseline from a power cycle and, if relevant, with a second battery state or floor surface. Use a descriptive `bal note` for each condition.

For every run, also note the observed peak floor travel and whether intervention was required. The controller records motion but cannot know whether a final stop came from a hand, wall, tether, or the floor surface.

## Saving a Run

1. End balance/return the robot to support. Lower **both CH10 drive and CH9 arm switches** (or their configured equivalents) and wait for `Log saved and checksummed`. Lowering CH7 alone does not disarm the motors and is insufficient for saving. If an abort occurred, support the robot and disarm both groups directly.
2. Run:

   ```bash
   ./scripts/save_telemetry.sh --label small-taps
   ```

3. Keep the resulting `telemetry_logs/bal_YYYYMMDD_HHMMSS_label.csv` and matching `.serial` file. The candidate's transfer checksum covers the exact exported bytes, including metadata. The host additionally verifies device-file checksum status, row count, column structure, finite values and timestamps. Interrupted or rejected transfers are retained as `.failed.serial` for diagnosis.
4. If the script reports a timeout or truncated transfer, do not run another balance test. Retry the download; the binary file remains on the robot.

The full v2 export can be around 2 MB, so the default download timeout is four minutes. Override the serial device or timeout only when needed:

```bash
./scripts/save_telemetry.sh --port /dev/cu.usbmodem1101 --timeout 300 --label retry
```

## What the Log Captures

The CSV contains the full 50 Hz state-machine/outer-loop stream plus aggregates from every 200 Hz inner-loop tick.

| Group | Important fields |
|---|---|
| Run identity/integrity | build date/time, test note, start/end uptime, end reason, schema, sample count, checksum |
| Timing | `sample_dt_ms`, `inner_dt_max_us`, `inner_ticks`, control profiler maxima, run-scoped stall ring |
| IMU/filter | filtered `roll`/`roll_rate`, raw accelerometer angle, raw gyro X, acceleration norm, maximum successful-sample age `imu_age_ms` |
| Inner PD | angle error, unclamped command, applied common command, left/right command after yaw correction, saturation ticks, dead-man age/state |
| Setpoint construction | scheduled/raw/smoothed base, capture shift, run curve shift, effective setpoint, offset target/applied offset |
| Outer cascade | target and filtered velocity, velocity error, P term, integral, position gate, high-speed shed, drift |
| Wheels/CAN | rear position/velocity/torque, feedback ages, yaw error/correction |
| Arm assist | measured/target position, velocity/torque, tip fraction, requested/applied assist, filtered demand, calm timer, READY/ACTIVE/HANDOFF/COOLDOWN stage |
| Power | cached bus voltage and summed motor current |
| Operator alignment | cumulative `marker` set by CH12 or `bal mark` |

`flags` retains the state-machine bits used by historical logs: `0x02` angle-error timer, `0x04` rate timer, `0x08` saturation timer, `0x10` capture stable, `0x20` arms returning, and `0x40` ramp complete.

`diag_flags` adds forensic conditions:

| Bit | Meaning |
|---:|---|
| `0x0001` | Inner command clipped |
| `0x0002` / `0x0004` | Soft / hard control-heartbeat dead-man |
| `0x0008` | Stale-feedback grace window after a control wake-up |
| `0x0010` | Glide-learning Ki boost active |
| `0x0020` | Arm-return ramp paused for a crisis |
| `0x0040` | Emergency arm throw active |
| `0x0080` | Setpoint offset target clamped |
| `0x0100` | Yaw correction clamped |
| `0x0200` | Row captured immediately before a safety exit |
| `0x0400` | IMU stale/invalid or sample deadline missed; latched until a deliberate new attempt |
| `0x0800` | At least one rear-wheel speed command failed to enter the CAN transmit path |

`imu_age_ms` occupies the former reserved byte, preserving the 220-byte v2 sample size. `telemetry_features=1` identifies the extension; age saturates at 255 ms. Older v2 files export an empty age field because their age is unknown. Successful host download validates a `transport_fnv1a` trailer; old firmware exports retain their weaker legacy checks. FNV-1a detects accidental corruption, not deliberate tampering.

The file checksum covers stored sample bytes, not all binary-header metadata. The transport checksum protects the complete exported payload in transit. Captures marked `duration_limit` or `buffer_full` stop recording without establishing a fall time. Power loss before idle save loses the PSRAM run; keep power on after an attempt.

The profiler is reset when a balance run starts and frozen when it ends. `bal log` therefore reports only that run; the download itself no longer creates misleading idle stall events.

## Analysis

The save script automatically runs:

```bash
python3 scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details
```

The detailed report calls out sample/control gaps, feedback age, saturation, setpoint clipping, peak error/command/velocity/drift, IMU disagreement, acceleration range, bus sag/current/torque, arm lifecycle activity, emergency throws, end reason, markers, and checksum status. Generate a plot with:

```bash
python3 scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details --plot
```

Keep the raw CSVs and the short operator observations together. Those two sources are the handoff needed for the next tuning pass.
