# OTA progress display and RC suspension — queued, not deployed by this task

Implemented in the root integration checkout on `codex/wifi-ota`, based on
`045e956`. This focused change is for the next combined firmware release; no
robot requests, upload, reboot, settings change, arming or motion occurred here.
The release owner reports its newer installed baseline is `a772ecc`, app1.

The robot's entire display becomes a large percentage with exact application
bytes written/expected, average KiB/s and elapsed seconds. The 10 Hz renderer
uses a bounded FreeRTOS snapshot queue, so it never takes the HTTP/flash mutex.
The phase proceeds through preparation, upload, verification and reboot; 100%
is reserved for successful SHA/ESP verification. Failure retains the partial
byte count for five seconds while disarmed, then restores normal status.

The control task checks the existing disarmed/idle predicates, resets pending
triggers/targets, sends stop to all six motors and suspends the CRSF UART before
publishing OTA flash permission. Suspending removes the Arduino UART driver,
including RX interrupts and telemetry. Motion controllers remain behind the
granted-maintenance early return. A failed/rejected operation restores an empty
UART and parser, invalidates channel/link freshness, and retains the switch-low
rearm latch. Non-OTA maintenance keeps its existing RC receive behavior.

The gate prepares the captured request kind before its atomic grant. A canceled
general request cannot accidentally grant a replacement OTA request without
its UART suspension. Failure, cancellation and retry paths retain bounded state.

Validation:

- `scripts/check_balance_candidate.sh`: 11 native executables, 38 Python tests,
  Python/shell syntax, whitespace and pinned ESP32 build.
- `bash scripts/check_radio_telemetry.sh`: production C++ CRSF TX/radio fixtures
  and Lua lifecycle/schema/staleness checks.
- `node tests/test_network_dashboard.js`: syntax and eight independent SHA vectors.
- New native checks cover progress arithmetic/clock rollover/verification cap,
  cancellation vs OTA grant ordering, UART suspension under a continuous stream,
  no TX while suspended, dropped partial/queued frames and fresh input on resume.
- `tests/test_ota_lifecycle.py` compiles actual production upload, progress,
  abort, watchdog and control-interlock code with the production CRSF receiver.
  It checks motor-stop/UART ordering before Update.begin, motion exclusion,
  ownership, verification/reboot, disconnect, timeout, bad offsets, multiple
  files, begin/write/hash/ESP failures, late chunks, rejection and RC recovery.
  Flash, SHA and hardware queues are stubbed; this is lifecycle coverage, not
  hardware integrity or RF reliability evidence.

Full validation log: local ignored `output/ota-progress-validation.txt`.
The existing `ARDUINO_EVENT_RUNNING_CORE` redefinition warning remains.
Configured binaries stay private in ignored `.pio/`; no release image was
frozen because the owner must build the final combined source.

Next: the lowering/integration owner cherry-picks the focused source/test
commit, preserving its current fast-feedback scheduling, cached `start_status`
JSON, cooperative export, and source-published OTA interlock. Its `a772ecc`
already contains transport v2; do not reapply `67f3746`. Run combined validation
once after scope freeze, then the separately authorized release. Observe the
screen and RC recovery on hardware. The upload installing this feature runs
under the old firmware; these changes apply to later uploads after reboot.
