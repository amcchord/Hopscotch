# Project journal

## 2026-09-19 — Wi-Fi telemetry and disarmed OTA

Worked in the control checkout on `codex/wifi-ota`, based on `112e7e7`;
implementation `694bd47`, TCP callback fix `78bf920`. Preserved the user's existing telemetry files and
separate radio worktree. Backed up the complete connected ESP32 before changing
its application. Initial inspection found radio arm switches high; issued
safe disarm requests, and Austin lowered both switches. Motor power stayed off.

Implemented station Wi-Fi, embedded browser dashboard, bounded RAM telemetry,
checksummed wireless saved-log export and authenticated dual-slot application
OTA. A control-owned maintenance gate excludes active/partially armed motors,
balance, calibration and other maintenance, and inhibits arming until completion.
Arduino's network-event task required an explicit core-0 override because its
SDK default could outrank the balance task on core 1. Kept driving-v4 control
math and radio channel mappings unchanged. No Internet service was deployed.

Software checks and on-device OTA, rejection, interruption and reconnect tests
passed. Fixed per-client telemetry backpressure and reconnect response ordering
found by bench tests. The initial application matched its USB readback, and the
complete filesystem, bootloader and partition table matched the original backup.
The older USB read tool produced transfer errors; the newer CRC-checked reader
retried successfully. Released native USB reset/download lines to restore normal
boot after inspection. Firmware source and complete evidence are local; no push.

Heavy concurrent traffic then reproduced a core-0 TCP accept panic. Decoded
the backtrace, fixed the library's null accept callback and access to a freed
connection PCB, and added native callback regression tests plus a repeatable
hardware load tool. The corrected application was installed in app1 by OTA, then the maintenance
status refinement `6018cd4` was installed in app0;
its running image digest matched the package. Transport saturation can drop
telemetry connections, so load validation distinguishes bounded reconnects
under overload from uninterrupted normal dashboard operation. Austin requested
that RC-link quality not gate network testing while he adjusts the radio.
Exited calibration triggered by held CH11 without saving, verified saved arm
deltas unchanged, and continued network-only checks with motors off.

Final normal-use and overload checks passed: 5.7 Hz for a single reader;
three active readers plus one stalled client and repeated HTTP requests held
continuous uptime, with no transport errors and 200 Hz maxima of 5.496/5.835 ms.
Repeated all upload rejection/interruption cases, verified the saved CSV again,
and rejoined Wi-Fi through the maintenance endpoint without reboot. Prepared
the powered stationary handoff; no powered motor or balance test was performed.

Current application, final stress measurements, hashes, private artifact
locations, rollback limitation and next safe operator step are recorded in
[CURRENT.md](CURRENT.md) and [release evidence](../../evidence/wifi-ota/README.md).
Next action is powered stationary motor/feedback verification with arm switches
low, then only Austin's supervised physical tests.

## 2026-09-19 — Make OTA and wireless telemetry the documented workflow

Documentation-only follow-up in the root control checkout on `codex/wifi-ota`,
based on `aa9ae13`. Updated README, project reference, Wi-Fi/OTA runbook, balance
test guide, radio integration docs and current-state records. Normal firmware
updates now use application OTA; live monitoring and validated saved-run
downloads use Wi-Fi. Documented `.csv`/`.wire` retention, separate analysis,
maintenance prerequisites, active-slot/image verification, disabled settings
APIs, and the remaining USB tuning/calibration/recovery operations. Removed
active instructions to upload LittleFS, including for embedded dashboard changes.

Added current-workflow pointers to 22 historical balance/evidence documents and
updated the tuning-history handoff without changing prior test results. The
release remains source `6018cd4`; installed state and powered-test limits refer
to the last verification session, not a new device inspection.

Validation: checked Markdown file/heading links, documented CLI arguments via
offline `--help`, API/schema/configuration claims against current source, frozen
release size and both SHA-256 identities against its manifest, whitespace, and
the documentation diff for local secrets. All passed. No firmware build or
hardware tests were repeated for prose-only changes. No device commands, flash,
settings changes, remote push or Internet deployment occurred in this pass.
Preserved the user's untracked pre-radio-upgrade telemetry pair.

Concurrent ground-drive work reported separate ownership in
`worktrees/ground-drive` and coordination with the standing-drive task. Replied
that this task owns only root documentation; the next release owner must update
CURRENT after actual integration/deployment. Their worktrees were untouched.
Next operator step remains a disarmed powered feedback check, then supervised
physical testing when RC is ready; use the Wi-Fi guide for subsequent updates
and downloads.
