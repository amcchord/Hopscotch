# Balance Mode Test Guide

**September 20 update:** ground/standing driving worked well; the first v1
lowering attempt tipped backward. Forward-fall/catch v2 is now installed and
verified ([deployment record](../evidence/ota-lowering-v2/README.md)). Its physical
catch remains unvalidated. Follow the [v2 trial guide](BALANCE_LOWER_2026-09.md)
for a restrained first attempt.

This guide is the repeatable procedure for collecting the data needed to tune Hopscotch's balance mode with the Wi-Fi/OTA firmware. The robot captures up to 120 seconds at 50 Hz, including tip-up, in PSRAM and saves it to LittleFS after balance ends and **both drive and arms are disarmed**. Download the checksummed CSV over Wi-Fi after every run. Only the latest run is stored on the robot. Capture reaching its limit does not stop the robot; end initial tests before that point to retain the outcome.

The installed combined release adds [progressive braking v5](BALANCE_DRIVE_BRAKING_2026-09.md), [flat-ground drive](GROUND_DRIVE_2026-09.md) and [experimental CH11 supported lowering](BALANCE_LOWER_2026-09.md), retaining the startup/stationary controller. Read the [current state](progress/CURRENT.md) for its identity and remaining hardware checks. The [Wi-Fi / OTA guide](WIFI_OTA.md) is the update and recovery procedure. Dated balance reports preserve earlier evidence; their old package/USB instructions do not identify the current release. OTA and powered disarmed feedback checks passed; new motion behavior remains physically unverified.

## Safety and Test Area

- Use a clear, level area with several meters of travel in both directions.
- Keep a spotter at the robot and keep the drive-disarm control immediately reachable.
- Start with light taps. Run one genuinely large disturbance per test so the cause and recovery remain unambiguous.
- Before downloading or starting another run, end balance, lower both drive and arm switches, and wait for saving to finish. The dashboard/`status` must show `saving_log: false`, `maintenance_allowed: true`, and `maintenance: false`. Keep power on until save and download finish.
- Do not intentionally create CAN, power, or control-task failures with an upright robot. The firmware records naturally occurring failures without requiring destructive fault injection.

Record any mechanical variables that changed: floor surface, tire condition, battery position/state of charge, payload, arm calibration, or chassis work. Those variables move the physical equilibrium and matter as much as software gains.

## Updating the Test Firmware

Normal updates and test captures do not require USB. Use the single-command
[frozen-package OTA procedure](WIFI_OTA.md#update-the-firmware). It performs fresh
preflight, saved-run backup, paced upload, image/health verification and saved-run
comparison automatically. Reuse completed release validation. Supported,
disarmed motor power may remain on; leave both arm switches low and CH11 released.
The current transport needs the transmitter off for the demonstrated reliable
path. Wait for the final verified report before turning it back on and testing.

**Do not run `uploadfs`, even for web changes.** The dashboard is embedded in the
application; LittleFS contains calibration, settings and the saved run. Do not
recalibrate or reset trim merely to update. Keep the sensor mounting consistent
and record any movement. The old web settings export is disabled. USB
`cal status` and `bal status` remain available for detailed diagnostics; use the
[slot-aware recovery notes](WIFI_OTA.md#recovery) if OTA is unavailable.

Before the first powered trial on this combined release:

1. Check live pose, fresh IMU data, no latched IMU fault, and retained firmware
   identity. Motors should be offline while motor power is off.
2. For the operator's stationary check, keep both arm switches low and CH11
   released, then power motors while supported. Confirm all six motors return
   fresh feedback without faults, with plausible voltage/current, while both
   groups remain disarmed.
3. Verify RC channel/link readiness before requesting movement. Boot and
   maintenance require both switches observed low before rearming. A stale
   browser view cannot establish readiness; check a fresh state. Do not
   force-engage a flat robot.

## Normal Start on Austin's Transmitter

Confirm the actual channel mapping first; Austin adjusted the radio after the
[saved configuration audit](RADIO_TELEMETRY.md). The table below uses firmware
defaults. With drive and arms disarmed, put the arms at their usual forward
starting position. The firmware establishes that forward reference when the
arms finish arming.

| Channel | Setting for a normal stand-up |
|---|---|
| CH1, CH2, CH4 | Neutral |
| CH7 | HIGH: balance selected |
| CH9 | HIGH: arms armed |
| CH10 | HIGH: drive armed |
| CH11 | Start LOW; after arming completes, one HIGH pulse, then LOW |
| CH12 | Leave LOW for this first test |

Allow at least two seconds after raising the arm switches and confirm both groups finish arming. Hold CH11 high for about one second, then lower it. A normal single pulse begins tip-up after the double-tap detection window; a double-tap requests force-engage instead. End the initial capture by 30 seconds if controlled, or earlier if intervention is needed. Support the robot, lower CH9 and CH10, and keep power on until saving and download finish.

## Driving while standing

After stand-up settles, keep CH1/CH2 centered for at least one second. CH2 requests forward/back travel and CH1 steers; keep CH7/CH9/CH10 HIGH. Begin with small, separate forward, backward and steering inputs. Center the sticks between each and wait for a stop. Driving limits remain 20 rad/s average wheel request and 4.5 rad/s per-wheel differential turn. Acceleration stays 6 rad/s²; v5 braking stays 8 rad/s² below 4 rad/s, rises smoothly to 20 between 4 and 8 rad/s, then stays at 20. Start around 10% stick (about 0.85 rad/s forward request after deadband). Planned arm movements assist acceleration/braking; confirm clean starts/stops before increasing input. The controller holds the new position/heading after stopping. Physical stopping distance with v5 remains unverified.

A held stick through startup cannot unlock standing drive. If control pauses after stale input or a large balance disturbance, center both sticks and let it settle before trying again. When the controller is Idle, CH1/CH2 can now drive on the ground even with CH7 HIGH; pending stand-up, active balance/lowering and arm return own the wheels exclusively. Center both sticks after a balance handoff before ground drive resumes. Support and lower CH9/CH10 to end; leave power connected for log save/download. See [progressive braking findings](BALANCE_DRIVE_BRAKING_2026-09.md) for behavior, evidence and limits.

## Supported return to flat

A fresh CH11 pulse after stand-up and arm return have settled now requests an
experimental supported descent. It is not a disturbance marker. Test this
separately from braking, with a catch restraint and verified arm sweep/reach;
follow the full [CH11 lowering procedure](BALANCE_LOWER_2026-09.md). CH12 remains
the balance event marker. V2 intentionally leaves upright balance to fall forward and catch on the arms;
physical support and graceful landing are not established by simulation.

## Commands and RC Markers

| Action | Command/control | Behavior |
|---|---|---|
| Live state | Dashboard or `python3 scripts/robot_wifi.py status` | Latest best-effort pose, motors, RC and maintenance state |
| Tag a run over USB | `bal note baseline-hard-floor` | Stores up to 63 characters in the next/current log; untethered, keep observations in a sidecar note |
| Detailed USB controller state | `bal status` | Shows gains, setpoint state, buffered samples, and pending-save state |
| Mark a disturbance | Press CH12 while balance is active | Increments the `marker` column on the same 50 Hz control tick; CH12 arm-home behavior is suppressed while balancing |
| USB marker | `bal mark` | Equivalent marker for bench tests |
| Download latest run | `python3 scripts/robot_wifi.py log` | Validates schema/checksums/row count and saves `.csv` plus `.wire`; run analysis separately |
| Disarm over Wi-Fi | `python3 scripts/robot_wifi.py disarm` | Authenticated request; verify the control task has disarmed both groups in fresh telemetry |
| Delete latest run over USB | `bal log clear` | Requires inactive balance and both motor groups disarmed; download first |

Set gains over USB between attempts. Wi-Fi has no gain, note, marker or motion
command endpoint. While balance is active the serial console accepts only
`disarm`, `disarm arms`, `bal status`, `bal note` and `bal mark`. After USB/web
disarm, lower both RC arm switches before attempting to rearm. A web response
means the disarm was requested; the control task executes it. Keep the radio
disarm control available; Wi-Fi delivery is best-effort.

Press CH12 immediately before each intentional push. Wait for the robot to settle and for the arm-assist lifecycle to return to READY before the next push. The `marker` value is cumulative, so each transition identifies a new test event without adding serial traffic to the live control path.

## Recommended Test Ladder

Download the log after **every** run. If a run falls or needs a hand stop, end it immediately and save that log before trying again. The labels below can be USB `bal note` values or names in your local operator notes when testing untethered.

1. **Stand-up baseline** — tag `standup-baseline`; perform a normal single-trigger tip-up; do not touch the robot for 30 seconds. Measure stand-up travel, arm-return behavior, stationary sway, equilibrium learning, and return-to-origin speed.
2. **Small disturbances** — tag `small-taps`; after full settle, mark and apply three light pushes in each direction. Allow a complete recovery between events.
3. **Medium disturbances** — tag `medium-taps`; repeat with firmer pushes, alternating direction. Stop escalating if wheel command, travel, or arm motion looks uncontrolled.
4. **Recoil/lifecycle test** — tag `recoil`; use one marked backward push large enough to deploy the arms, then let the robot handle the natural forward recoil without a second touch. Repeat in the opposite direction in a separate run.
5. **Station keeping** — tag `station-60s`; leave the robot untouched for 60 seconds. This isolates low-frequency sway, drift return, yaw divergence, and trim convergence.
6. **Repeatability** — repeat the baseline from a power cycle and, if relevant, with a second battery state or floor surface. Record each condition in the run note or local sidecar.

For every run, also note the observed peak floor travel and whether intervention was required. The controller records motion but cannot know whether a final stop came from a hand, wall, tether, or the floor surface.

## Saving a Run

1. End balance/return the robot to support. Lower **both CH10 drive and CH9 arm switches** (or their configured equivalents) and release CH11. Wait for fresh telemetry to show both groups disarmed, `saving_log: false`, `maintenance_allowed: true`, and `maintenance: false`. With USB connected, `Log saved and checksummed` confirms completion. Lowering CH7 alone is insufficient. If an abort occurred, support the robot and disarm both groups directly.
2. Run:

   ```bash
   .venv/bin/python scripts/robot_wifi.py log
   ```

3. Keep `telemetry_logs/bal_YYYYMMDD_HHMMSS_wifi.csv` and matching `.wire` together. The CLI validates the device-file checksum, exact transfer checksum including metadata, row count, column structure, finite values and timestamps before writing them. It refuses to overwrite existing files. Add floor/contact/push observations alongside them.
4. If download fails or validation rejects it, do not start another run. Retry while disarmed; the binary file remains on the robot. The Wi-Fi helper uses a 120-second request timeout and does not automatically archive a failed response. Use a unique `log --output telemetry_logs/<descriptive-name>.csv` for a custom filename.
5. Run the [analysis](#analysis) on the validated CSV. Wi-Fi download does not run it automatically.

The dashboard's **Download latest run** returns the raw export. Prefer the CLI
for a validated archive. To validate a browser download, choose a new output
path and run `python3 scripts/validate_telemetry.py <raw-download> <clean-output.csv>`.
This validator writes the output path, so do not point it at an existing archive.

### USB fallback

If Wi-Fi is unavailable, use `./scripts/save_telemetry.sh --label small-taps`
with the same disarmed preconditions. It saves cleaned `.csv` plus `.serial`,
retains failed transfers as `.failed.serial`, and automatically prints analysis.
Large CSV transfers can take minutes; the USB helper defaults to four minutes.
Override its port or timeout only when needed:

```bash
./scripts/save_telemetry.sh --port /dev/cu.usbmodem1101 --timeout 300 --label retry
```

## What the Log Captures

New captures use **schema 4, 240 bytes/sample, feature flags 8191** and a
1,440,000-byte buffer for 6,000 samples. The CSV contains the full 50 Hz
state-machine/outer-loop stream plus aggregates from every 200 Hz inner-loop
tick. Live Wi-Fi JSON schema 1 is a separate snapshot format, offered at 10 Hz;
network delivery can skip frames and must not be used to reconstruct a full run.

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
| Standing drive | `pilot_forward`, `pilot_steering`, `pilot_turn`, `pilot_flags`, planned assistance `pilot_arm` |
| Power | cached bus voltage and summed motor current |
| Operator alignment | cumulative `marker` set by CH12 or `bal mark` |

`flags` retains the state-machine bits used by historical logs: `0x02` angle-error timer, `0x04` rate timer, `0x08` saturation timer, `0x10` capture stable, `0x20` arms returning, and `0x40` ramp complete. With telemetry feature bit 16, `0x80` means the startup recovery was triggered during this run. With feature bit 32, `0x01` means the temporary recoil-unwind multiplier is above 1, including its blend-out. It is an enabled-gain flag, not a measurement of the applied integral change; existing rate/angle/zero-crossing limits still apply.

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
| `0x1000` | Early wheel recovery active (feature bit 16 only) |
| `0x2000` | Temporary startup learning boost active |
| `0x4000` | Startup correction rate/angle limit, anti-windup or stale-input hold |
| `0x8000` | Recovery settled; current position captured for holding |

### Historical schema compatibility

`imu_age_ms` occupies the former reserved byte in the 220-byte v2 prefix.
`telemetry_features` bit 1 identifies the extension; age saturates at 255 ms.
Older v2 files export an empty age field because their age is unknown. Successful
host download validates a `transport_fnv1a` trailer over either Wi-Fi or USB;
old firmware exports retain their weaker legacy checks. FNV-1a detects accidental
corruption, not deliberate tampering.

The two-success baseline logs use feature flags 31: IMU age (1), corrected RS05 units (2), fast CAN receive/front-hold cadence (4), absolute capture trim (8), and wheel-feedback startup recovery v1 (16). The September 19 recoil-release candidate adds bit 32 (total 63): post-ramp active recovery, opposing velocity ≥0.35 rad/s for 60 ms, release below 0.15 rad/s, 120 ms gain blend, maximum 2× ordinary Ki. Three reserved configuration bytes still store the initial trigger speed, high-speed fallback and confirmation duration. Metadata describes the stored algorithm version; older files downloaded with new firmware keep their original feature interpretation. That release retained schema 2 and its 220-byte sample layout. `meas_drift` always measures travel from engagement, including after a new holding position is captured.

The file checksum covers stored sample bytes, not all binary-header metadata. The transport checksum protects the complete exported payload in transit. Captures marked `duration_limit` or `buffer_full` stop recording without establishing a fall time. Power loss before idle save loses the PSRAM run; keep power on after an attempt.

Schema 3 appends `pilot_forward`, `pilot_steering`, `pilot_turn` and `pilot_flags`
to the schema-2 prefix (236 bytes; feature bit 64). Pilot flags are ready=1,
moving/braking=2, turning=4, fresh input=8, and acceleration/handoff=16 on schema 4.
Schema 4 appends `pilot_arm` (240 bytes; feature bit 256). Driving v4 retains
that layout and adds versioned damping/recovery metadata under feature bit 512,
for total flags 1023. It does not separately sample the fast driving-rate filter.
The combined release adds braking feature bit 1024 and lowering bit 2048 (total
4095). Pilot flag 32 identifies accelerated reference braking; bit 64 identifies
active lowering, with its phase in bits 8–11. Forward-fall/catch v2 adds feature
bit 4096 (total 8191); the lowering guide documents its changed phases/rate field.
Old files retain their original version/configuration and export unknown new
fields as blank. Download new logs before restoring an older reader.

The profiler resets at balance entry and freezes at exit; Wi-Fi and USB exports
report the same physical run. The dashboard's network timing counters instead
exclude the first five boot seconds and maintenance, and reset after maintenance.
Use run-scoped evidence to judge physical control timing.

## Analysis

After a Wi-Fi download, replace `<run>` with the actual downloaded filename and run:

```bash
python3 scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details
```

The detailed report calls out sample/control gaps, feedback age, saturation, setpoint clipping, peak error/command/velocity/drift, IMU disagreement, acceleration range, bus sag/current/torque, arm lifecycle activity, emergency throws, end reason, markers, and checksum status. Generate a plot with:

```bash
python3 scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details --plot
```

The optional plot requires matplotlib. Keep validated CSV, original `.wire`
(or USB `.serial`) and short operator observations together for the next tuning pass.
