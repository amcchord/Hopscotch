# OTA throughput candidate — local validation, not installed

Worktree: `worktrees/ota-throughput`, branch `codex/ota-throughput`, baseline
`d5f18fa` (v10 deployment record). [Findings, changes and release handoff](../../docs/OTA_THROUGHPUT_2026-09.md).

Production changes are limited to `src/web_server.cpp`, `src/web_server.h`,
new `src/ota_metrics.h` and the host uploader `scripts/robot_wifi.py`.
All motion, control scheduling, RC suspension, display, dashboard, telemetry
capture schema and pinned build dependencies are preserved byte-for-byte.

Validation passed:

- Consolidated check: 12 native executables, 42 Python tests, syntax/whitespace
  and configured pinned ESP32 build (`output/validation.txt`, ignored).
- Production upload/control lifecycle harness also exercises response timing
  headers before reboot, Update.write/verification timing and receive-gap
  counters. Native metrics tests cover wraparound, accumulation and reset.
- Host tests cover both upload profiles, exact multipart transport on localhost,
  bounded send sizes, absence of sleeps in fast mode, measured socket/sleep
  timings, invalid parameters, fast-mode backup/verification, and no retry on
  disconnect. Header parsing tolerates old firmware and invalid timing values.
- Pinned TCP poll / OTA transport acceptance harness passes, including bounded
  timeout and disconnect ownership (`output/transport.txt`).
- Radio C++/Lua and dashboard syntax/eight SHA vectors pass
  (`output/radio.txt`, `output/dashboard.txt`).
- Pinned Arduino source reviewed: `_scanMethod` defaults FAST_SCAN, setters
  persist policies, and `begin()` passes them into the station configuration.
  The build verifies the supported APIs; strongest-AP choice is not yet observed
  on hardware. No new AP scan or network probe was performed.

Dependencies were bootstrapped in this worktree. The ignored local build config
adds only the existing root private header's include directory; no private
header, dependency tree or saved log was copied from another worktree. Built
binaries stay private in ignored `.pio/`. Existing event-core macro warning
remains. No candidate image was frozen for deployment.

The new firmware and fast sender are not hardware-validated; no throughput
improvement is claimed yet. Default sender pacing is preserved pending the
release owner's normal authorized comparison. No robot requests, firmware
upload, reconnect, settings change, movement, GitHub push or other checkout
edit occurred in this task. Next: focused candidate handoff to the lowering /
integration owner for the next combined release and measured comparison.
