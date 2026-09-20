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

## 2026-09-19 — Checkpoint all local work for GitHub publication

Austin explicitly authorized committing and publishing all current and past
agent work, including work in progress. Inventoried the control checkout and
all three worktrees, seven local branches, remote heads, stashes and reflog-only
commits. The sole reflog-only radio commit is patch-identical to its reachable
rebased commit; no unique past work is stranded there. GitHub repository
`amcchord/Hopscotch` is public. Credential-bearing firmware/full-flash images,
private radio backups, local secrets and generated caches remain ignored/local.

The ground-drive owner committed implementation `b323220` and validation
`1337c10`: nine native executables, 27 Python tests and PlatformIO build passed.
The standing-drive owner committed `749e562` and `59895c7`: new physical v4 trial,
analysis/model screens and an unwired braking prototype, explicitly WIP with no
new firmware deployment. Raw `.wire` data is marked non-text on that branch and
was checked byte-for-byte by its owner. Radio-display fix `c26027a`, root Wi-Fi
implementation/docs, and earlier local branches are included in publication.

Committed the previously untracked pre-radio-upgrade CSV and `.serial` pair:
2,381 samples; transport/file validation passes and reconstructed CSV exactly
matches the existing cleaned file. This is an archive of the pre-upgrade trial,
not a new physical run. Scanned outgoing blobs and captures for known local
passwords/token and common credential patterns; no new credentials found. The
only heuristic match was an existing default already present on origin/main.
No source integration or hardware action is part of this repository checkpoint.
Publication results are recorded below after remote verification.

Publication verified: an atomic push created/updated all seven branch refs on
GitHub, and `git ls-remote` matched every captured commit exactly. This includes
`main` (`2ffed40`), `agent/balance-telemetry-sync` (`e8b1280`),
`codex/balance-review-ready` (`112e7e7`), `codex/radio-telemetry` (`c26027a`),
`codex/ground-drive` (`1337c10`), `codex/drive-braking` (`10766f7`, including
its owner's docs/ground-drive cherry-picks), and `codex/wifi-ota` (`0cac989`
at the initial publication). No force push, branch merge into main, or device
action was performed by this publication task. Root archival CSV and raw serial
bytes were also compared with their committed blobs. Subsequent commits on the
active braking branch are additional WIP checkpoints, not release validation.

Final in-progress checkpoint: the braking owner committed all newer experimental
files as `f156e48`; scanned and pushed it, then verified the remote hash. Strong
braking boosts added model falls and are rejected; the provisional helper is
default-disabled/unwired. This is preservation of unfinished work, not a release.
The owner briefly held edits for verification. An independent-index snapshot
was considered but unnecessary because the owner's commit arrived first; no
extra branch was created and no other worktree/index was modified here.
All seven branch checkpoints, including the final braking work, are published.
Agents may continue with new work after this snapshot.
