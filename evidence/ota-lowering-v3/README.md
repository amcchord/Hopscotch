# Lowering v3 + OTA transport 2 + cooperative log export

Installed source `8449ddb5758d60d6b3b3e84702bef4d386b06791` at
2026-09-20 19:01:29 UTC, app1. See [deployment.json](deployment.json) for exact
preflight, transfer, postflight and recovered-run hashes. The application is
1,197,808 bytes, whole-file SHA-256
`53a574a75a0122b0aa3a16567c79bd282631b232c72184c7c3e836c53f51e0f7`,
ESP digest `45e7a83cd649e4a1e5182058e3cfb9e4cc80fad4cda27be4a2c9f4465451ab11`.
Its configured private binary, ELF and partitions remain in
`worktrees/balance-lower/artifacts/lowering-v3-export/candidate/` under the
project. Only firmware.bin was uploaded. Previous exact v2 and v5 packages
remain in the drive-braking worktree; no secrets/binaries were copied between
worktrees or published. No motion, calibration or settings change was requested.

The transmitter-off upload returned HTTP 200 in 62.842 seconds. The new digest,
slot, OTA transport version 2, fresh IMU, six healthy disabled powered motors,
both groups disarmed and released maintenance were verified. Lowering v3 still
needs a restrained physical trial. Transmitter-on acceptance is recorded
separately when completed.

## Export watchdog and recovery

The first normal updater attempt failed **before upload** while archiving the
latest log. Its 120-second download timed out; the robot returned on the same
v2 image with reset reason 6 (task watchdog), previously 1 (power-on). See
[failure evidence](preflight-export-failure.json). The old export performs
checksum and thousands of floating-point CSV writes synchronously on core 0;
the PSRAM sink had no serial-backpressure waits. Idle-task starvation is the
working explanation; no panic backtrace was available to establish the exact
watched task. In the pinned Async TCP header, defining the running core leaves
its optional task-watchdog macro undefined, so that per-event watchdog block
is not compiled by this configuration.

The fix yields one scheduler tick after each 10 ms of export work, during file
verification and memory writes, without disabling/feeding a watchdog. The
authenticated disarmed download also gets a 120-second receive timeout and
15-second ACK timeout. Export wire content and checksums are unchanged.
Maintenance remains held during all filesystem reads and then releases before
the immutable memory buffer drains.

The latest run was **not backed up before this recovery**. Rather than repeat
the watchdog-triggering export, the one-time [recovery script](bootstrap_export_fix.py)
verified the exact installed image and fresh idle eligibility, installed only
the application, and immediately retrieved the unchanged filesystem's saved
run. Austin also explicitly allowed discarding the log during this recovery;
discarding it proved unnecessary. This exception is not a routine updater flag.

The recovered [CSV](../../telemetry_logs/bal_20260920_recovered_v2_bailout.csv)
and [wire](../../telemetry_logs/bal_20260920_recovered_v2_bailout.wire) validate:
4,789 samples / 96.010 seconds, v2 flags 8191, `bailout_angle_error`.
This later run contains waiting-to-lower samples but never enters preparation,
forward commitment or catching; it does not replace the earlier 1,984-row
[rebound trial](../balance-lower/trial-v2-20260920/README.md) used for v3.
The new export finished with uptime increasing from 4,247 to 21,927 ms, without
another reset. The pre-update file hash was unavailable, so cross-update byte
identity is not claimed for this recovery.

## Reused and changed-code validation

- Combined lowering/OTA source `d8613e6`: 10 native executables, 37 Python tests,
  syntax/whitespace, pinned configured build, radio/dashboard and actual-method
  OTA transport harness passed. Logs and source preservation are in
  [first-impact evidence](../balance-lower/first-impact-v3/README.md).
- Same-model 304-case comparison: 231 to 248 completions, 21 improvements and
  four regressions. These limits are retained; model success is not physical
  validation. Motion sources did not change for the export fix.
- Export fix: existing native transport checks plus a 6,000-row simulated
  formatting load check bounded yielding, timer rollover and exact bytes/FNV;
  passed. All 10 updater host tests passed after adding earlier preflight
  persistence and progress reporting. [Pinned final build](export-fix-build.txt)
  passed in 4.91 seconds, RAM 53,384 / flash 1,197,437 bytes. Existing Arduino
  event-core macro warning remains.
- No redundant model/radio/dashboard rerun after the export-only change. The
  successful hardware recovery is the export fix's physical acceptance.

The earlier frozen `d8613e6` package was never installed. Its manifest is retained
as preparation history. The [new manifest](candidate-manifest.json) identifies
these exact installed bytes; the deployment record supersedes its queued status.
