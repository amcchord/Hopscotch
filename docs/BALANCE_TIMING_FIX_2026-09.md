# September 14: slow stand-up diagnosis and receiver timing correction

The latest attempt barely moved because the arm-control loop was repeatedly
stalled in radio receiver processing. Increasing motor speed or balance gains
would not restore those missing control updates.

## Evidence

The [saved attempt](../telemetry_logs/bal_20260914_213606_slow-start-before-fix.csv)
passed both the device-file and USB transport checksum checks. It contains just
20 samples spanning 14.849 seconds, entirely in TIP_UP, ending with
`tip-up timeout / arm tracking`. The run lasted 15.276 seconds. Individual sample
gaps reached 1.228 seconds; the receiver section consumed up to 1,210,924 µs.
The entire control section peaked at 1,211,819 µs. CAN processing peaked at
1,067 µs and radio telemetry transmission at 1,315 µs. The independent 200 Hz
IMU/inner task continued running, so a healthy IMU did not imply healthy arm
control. These profiler values are explicitly scoped to this run.

Each arm update advanced its target by 0.014 radians, appropriate for 0.7 rad/s
at 50 Hz. With updates separated by a second, the result was slow creeping until
the 15-second timeout. Do not replace these steps with large catch-up jumps.

The old receiver loop capped reads at 1,024 bytes but repeatedly called
`available()` and single-byte `read()`. A byte budget did not bound elapsed
time. The log isolates the delay to this function; it does not identify the
precise underlying UART-driver contention mechanism.

## Firmware changes

- Read UART data in blocks of at most 128 bytes using the installed Arduino
  ESP32 2.0.16 nonblocking `read(buffer, size)` overload. Its implementation calls
  `uartReadBytes(..., 0)`. Do not use the timeout-based `readBytes()` overload.
- Retain the 1,024-byte limit and add a 2,000 µs elapsed-time budget, checked
  between blocks with unsigned rollover-safe subtraction. A single driver call
  can exceed that budget; on-device measurements are required. Finish each
  already-read block before yielding, preserving partial frames across updates.
- Allocate a 2,048-byte receive buffer before UART initialization to retain
  bursts between 50 Hz control ticks. At 420 kbaud, the physical maximum is
  approximately 840 bytes per 20 ms interval.
- Reject CRC-valid packets with too little payload for RC channels or link
  statistics. A truncated channel packet must not refresh link freshness or
  change arming/trigger inputs. Accept additional payload fields for protocol
  compatibility. The [TBS CRSF specification](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md)
  defines the 22-byte channel payload, 10-byte link statistics, and extension rule.
- Expose receiver lifetime maximum duration, processed bytes, budget yields and
  channel-frame age through the disarmed `status` command.

Balance gains, arm motion rates, calibration, stored trim and the RS05 startup
fix remain unchanged. Timing must be correct before evaluating another gain
change. This release targets the demonstrated startup delay; it has not yet
established reliable unaided balance or eliminated the earlier initial runaway.

## Validation

The native suite executes the production receiver implementation. It covers
all 16 packed channels, CH11 high/low, fragmented frames, corrupt CRC, short
payloads, invalid lengths/noise, extended packets, link expiry, partial reads,
continuous incoming data, elapsed-time/byte budgets and clock rollover.
A simulated UART reproduces the old per-byte stall: 1,228,800 µs versus 320 µs
with bulk reads. This comparison verifies the implementation change under a
controlled driver model, not an on-device speed claim.

Consolidated checks and flash/live measurements are retained under
`evidence/balance-slow-start/`. The frozen image, hashes and rollback application
are packaged under `artifacts/balance-timing-fix/`; release outcome is recorded
in [current state](progress/CURRENT.md).

## Earlier assisted run and USB recovery

The earlier USB transfer was repaired in source `213c9bc` and flashed with
application SHA-256
`0a72f4d040df679f624a0a1b39fda73309f36aea6c8a7c08cf097d5e7d9cf12b`.
Writes now honor USB queue capacity and use a longer timeout only for explicit,
disarmed downloads; host reception drains continuously. The completed
[5,648-row assisted-run download](../telemetry_logs/bal_20260913_230816_rs05-startup-fix-assisted.csv)
passed file CRC `0x251DC7CD` and transport FNV `0x937A55CF`. Its profiler prefix
was collected after reboot and is marked `live`; it is not the old run's
profile. The saved run configuration and samples remain valid.

That run reached BALANCE at 12.526 seconds. The initial surge occurred during
arm return and before ramp completion: reported wheel speed reached about
8.60 rad/s and wheel displacement about 9.91 radians (1.58 wheel revolutions).
This differs from the final July run's strongest post-ramp surge. Austin reported
hand assistance before vertical balance; the intervention time and whether
later balance was fully hands-off are unknown. The recording ended at its
120-second duration limit, not a measured fall. It also contains approximately
1.1-second outer-control gaps, reinforcing the need to fix timing first.
The persisted equilibrium trim moved from about 0.97° to 1.09° during that run.

## Next physical test

Use the [existing channel routine](BALANCE_TESTING.md): arms in the usual
forward starting position while disarmed; CH1/CH2/CH4 neutral; CH7, CH9 and CH10
high; wait two seconds; pulse CH11 high for one second, then low once. Avoid a
double tap. Expect the arms to move at their intended speed again. Use a clear
area and be ready to support/disarm if it starts running away. After the attempt,
support the robot, lower both CH9 and CH10, and keep battery/USB connected to save
and retrieve the automatically recorded data.
