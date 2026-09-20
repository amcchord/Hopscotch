# Wi-Fi / OTA release — September 19, 2026

Source: `6018cd4` on local branch `codex/wifi-ota`, based on `112e7e7`.
Project/control checkout: `/Users/austinmcchord/Development/Hopscotch`.
Radio worktree was not changed. No remote push or Internet deployment.

## Installed application

- ESP32-S3 / M5Stack AtomS3R, 8 MiB flash / 8 MiB PSRAM.
- Running partition: app0 (`0x10000`); two existing 3,342,336-byte OTA slots.
- File size: **1,187,632 bytes**; application SHA-256:
  `3bd85bfdd26f48df86c5d87b76797282f4c2479a7e96b2838217f479553171cf`.
- Verified ESP image content digest:
  `08f5faeb998e090cb1d1a4cb339814817149975e8eb83b9210f58007a92e14cd`.
  This excludes the image's appended digest and differs from the complete
  upload-file SHA-256 by design.
- Build reports 1,187,269 bytes flash use (35.5% of one slot), 53,288 bytes
  static RAM (16.3%). Runtime free PSRAM is about 6.9 MB before a download.
- Exact private application, ELF, partition binary and manifest:
  `artifacts/wifi-ota/release/`. Local Wi-Fi credentials are compiled in;
  do not publish these binaries or the full-flash backup.
- The firmware reports NetworkTask, HTTP/async_tcp and Arduino Wi-Fi event
  task on **core 0**. Balance remains core 1 / priority 18, control core 1 /
  priority 12. SDK-default Arduino events were core 1 / priority 19; the
  explicit core-0 override is compile-time enforced.

## Preservation and USB verification

The original full 8 MiB backup is
`artifacts/wifi-ota/pre-upgrade-flash.bin`, SHA-256
`5a1e1099ca7df7b56bb77c7a14353df3f597edfb01b186e892028ce470c837ef`.
Its application exactly matches the previously frozen driving-v4 package.

The initial release `694bd47` was read back and matched its package byte for byte.
At that point the complete LittleFS partition matched the original backup byte for byte,
including settings, calibration, saved telemetry and old web assets. Filesystem
SHA-256: `0b437fc7a9f9d52418a76a7d54d3d93cd9f0a31808597b01f64e90e378c4520d`.
Bootloader and partition-table bytes also matched. The subsequent application-only
OTA installed the TCP callback fix without filesystem writes and verified its
new running image digest. See
[readback-checks.json](readback-checks.json).

The 2022 OpenOCD reader produced word-duplication errors during a later USB
readback. Those reads were rejected. OpenOCD 2.1200.20260304 at 1 MHz detected
CRC errors, retried transfers and produced the verified readbacks above.
An attempted esptool read failed in its flasher stub. Release of both native
USB serial DTR/RTS lines restored normal SPI flash boot after the debug session;
no further flash write was needed. Prefer OTA for normal operations.

## Completed checks

The first heavy load runs **failed** with a reproducible core-0 null-pointer
panic in AsyncServer's accept callback; [decoded crash](tcp-crash.txt) and
[failed run](stress-before-fix.json) are retained. Investigation also found
Async TCP 3.1.4's error callback writing to a PCB after lwIP had freed it, which
can damage a newly allocated handshake's callback pointer. The TCP fix
`78bf920` (included in `6018cd4`) fixes both paths with a version/hash-checked build patch. Native
regression checks execute the actual replacement callbacks, including an
inaccessible freed-PCB pointer. Other library and controller versions remain pinned.

- Eight core native executables, 27 Python tests, shell/Python syntax,
  whitespace, and full PlatformIO build passed; see [build/test log](final-validation.log).
- Maintenance conditions cover all 1,024 combinations, including individual
  motors left enabled after partial arming, plus request/grant cancellation races.
- Existing extended radio C++/Lua encoding, framing/backpressure, navigation,
  stale state and malformed-data tests passed; [radio log](radio-validation.log).
- Dashboard JavaScript parses and its HTTP-compatible SHA-256 matches Node's
  independent implementation for eight sizes; [dashboard log](dashboard-validation.log).
- Multiple complete authenticated Wi-Fi updates succeeded in both slot directions,
  including repeat installation of the same image. Each reboot rejoined the LAN
  disarmed; the final running image digest identifies the installed bytes. USB readback
  covered the preceding image. See [final OTA](final-ota.log).
- Missing token rejected for log, disarm and update; missing file rejected;
  unsafe simulation state rejected OTA and log access without arming motors;
  truncated data, wrong hash and invalid ESP image rejected; mid-upload TCP
  disconnect aborted maintenance without changing the active slot.
  [Hardware negative checks](rejection-checks.json). These checks were repeated successfully on the final `6018cd4` image.
- Explicit Wi-Fi reconnect acknowledged HTTP 202, rejoined without reboot,
  and retained disarmed/healthy-IMU state; [reconnect check](reconnect.json).
- Wireless log download returned 2,381 validated samples, exactly matching the
  pre-upgrade archived data rows. A repeated 951,141-byte wire export took 38.02 s
  and matched the earlier full cleaned CSV. [Download benchmark](download-benchmark.json).

All physical checks used **motor power off**. No motion or arm command was sent.
The existing driving-v4 controller, gains, calibration, trim and channel maps
are retained. Powered CAN/motor health and actual balance/drive behavior require
Austin's next supervised test. The radio firmware payload is now included in the
robot application; its actual appearance on the GX12 was not inspected here.

## Load-test status

After the TCP fix, a 180-second saturation run kept continuous uptime, healthy
IMU and RC reception, and both 200 Hz tasks progressed without an interval over
7.5 ms. Maximum intervals were 5.501 ms (balance) and 5.916 ms (control), with
at least 110,380 bytes of observed free heap. The transport itself saturated:
one reader disconnected and two of 611 HTTP requests timed out. The original
strict transport harness therefore failed; these results demonstrate control
isolation, not lossless networking. See [measurements](overload-control-isolation.json).

The harness now models dashboard reconnection and distinguishes deliberate
overload from normal use. The first normal-use test was stopped early after RC
link loss and arm-switch changes; Austin confirmed he was adjusting the radio.
An authenticated disarm was sent, leaving all six motors offline and both groups
disarmed. [Interrupted measurements](normal-operator-interrupted.json).
Austin then requested network-only testing without RC-link quality as a pass/fail
condition. Exited calibration (CH11 had been held high) without saving, and
verified the existing center/backward deltas were unchanged. Final normal-use
and recovery tests use `--ignore-radio`; their measurements are below.

Final `6018cd4` results, with motor power off and RC-link quality excluded:

| Check | Normal use | Deliberate overload |
| --- | --- | --- |
| Duration | 61.087 s | 121.908 s |
| Active telemetry readers | 1 | 3 + one nonreading socket |
| Delivered frames | 350 (5.7 Hz) | 531 / 505 / 558 (4.1–4.6 Hz each) |
| HTTP requests | 46 | 586 |
| HTTP / WebSocket errors | 0 | 0 |
| Reconnects / resets | 0 / 0 | 0 / 0 |
| Largest received-frame gap | 1.470 s | 3.104 s |
| 200 Hz balance / control max interval | 5.495 / 5.835 ms | 5.496 / 5.835 ms |
| Intervals >7.5 ms | 0 | 0 |
| Minimum observed free heap | 159,316 bytes | 103,468 bytes |

Both harness checks passed. The overload run exercised per-client skipped
frames while every active reader continued receiving. See [normal](final-normal.json)
and [overload](final-overload.json). Telemetry is best-effort and must not be
used to close a control loop. The saved run was downloaded once more from the
final firmware and exactly matched the earlier cleaned export;
[final log check](final-log-check.json). Authentication/rejection/interruption
checks were also repeated on this final image.

Operational details and recovery limitations: [Wi-Fi / OTA guide](../../docs/WIFI_OTA.md).
The existing bootloader does not provide automatic rollback from a boot-broken
but structurally valid future application. Integrity checks and interrupted-upload
protection were tested; they are not a signed-image trust chain.
