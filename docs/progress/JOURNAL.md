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

A new clean `codex/balance-lower` worktree appeared during final verification,
starting at the already published `10766f7`. Published that branch as well,
bringing this checkpoint to eight branches. This task did not change its files.

## 2026-09-20 — Integrate the verified combined OTA release

The release owner verified combined source `43b1967` on the robot and published
release record `eb2bb23` on `codex/drive-braking`. Merged that branch into the root
`codex/wifi-ota` checkout, preserving the previous documentation/publication
commits and the complete append-only journal. Only CURRENT/JOURNAL conflicted;
no source conflict or firmware modification was needed.

CURRENT, README, operating/test guides and tuning history now describe progressive
braking v5, ground-drive ownership gating, experimental supported CH11 lowering,
feature flags 4095 and the verified app1 identity. The exact private package stays
in the release owner's worktree; the previous Wi-Fi package remains available
for rollback. Two failed upload attempts preserved the old image; the successful
paced transfer, changed transmitter state and unresolved failure cause are
recorded without assigning causality. Postflight confirms six disabled, healthy
motors and preserved 2,981-row CSV/wire; physical motion tests remain pending.

Independent root verification passed: 10 native executables, 27 Python tests,
radio C++/Lua, dashboard syntax/eight SHA vectors, and full pinned ESP32 build.
Production source/build/dashboard/radio files exactly match installed `43b1967`.
Checked the private image size/both hashes and archived telemetry against the
release manifest/postflight; current document links and whitespace pass. Evidence
is under [integration-2026-09-20](../../evidence/integration-2026-09-20/README.md).
No device requests, reflash, arming or motion occurred in this integration task.

The radio branch also advanced with Lua v3 fix `9c5b88d` and verified SD-install
record `c72d67b`; preserve/publish those commits on their own branch without
changing this exact firmware-source snapshot. Push the root integration under
Austin's existing publication authorization and verify remote hashes. Next:
operator braking trial, then a separate restrained lowering trial, each followed
by a disarmed wireless log download.

## 2026-09-20 — First combined trial report; lowering diagnosis handoff

The lowering task relayed Austin's first combined physical-test report: ground
and standing driving worked very well, while CH11 lowering failed and appeared
to lean backward rather than forward. This is an operator observation pending
telemetry analysis; no cause is inferred here. That task owns preservation and
diagnosis of the latest onboard run in its isolated lowering worktree. Updated
CURRENT and the active lowering/test guides to hold further lowering trials.
No new source change, device request, log download or flash was performed by
the root integration task; the deployed source remains `43b1967`.

## 2026-09-20 — Queue forward-fall/catch lowering v2; no deployment

The lowering owner published physical-trial archive `f011ce6` and candidate
`13d4ee2` on `codex/balance-lower`, based on deployed release `eb2bb23`. Austin
confirmed v1 tipped backward on its own and clarified forward fall followed by
an arm catch. The validated 2,371-row CSV/wire and historical C++ replay match
all recorded phases: reaching never qualified support, canceled, then exceeded
the motion limit. [Trial diagnosis](https://github.com/amcchord/Hopscotch/blob/13d4ee2aa51afe7e5ab7aebf32b711da403a1fb1/evidence/balance-lower/trial-20260920/README.md).

The new candidate deliberately transfers out of upright control, initiates
forward rotation, holds each arm at first load and requires two-arm support plus
measured falling/deceleration before supported descent. Ten native executables,
27 Python tests and full pinned build passed in the owner's worktree. The 229
coupled model cases include 216 variants (180 complete, 36 preparation faults),
11 failures with no false success, nominal and high-inertia cases. Nominal is
19.90 seconds; completed variants can catch at 55.15°/s, so physical grace and
impact safety remain unverified. [Candidate evidence](https://github.com/amcchord/Hopscotch/blob/13d4ee2aa51afe7e5ab7aebf32b711da403a1fb1/evidence/balance-lower/forward-catch-v2/README.md).

Independently compared all 11 listed production-preservation files with deployed
`eb2bb23`; they are byte-identical. Root firmware remains exactly installed
`43b1967`, app1; candidate flags 8191 do not describe the installed 4095 image.
Updated CURRENT and the trial notices to identify the queued candidate and hold
retries of v1. The release owner has the integration handoff; no unrequested OTA
is part of this task. Root changes are documentation only, with no device
requests, settings writes or motion. Additional radio fix `605c66d` is preserved
and published on its existing branch, separate from the firmware snapshot.

## 2026-09-20 — Record verified, packaged lowering candidate; still not installed

The release owner published integration record `e9e1c65` on `codex/drive-braking`.
Trial archive `f011ce6` was imported as `7a98037` and candidate `13d4ee2` as
`dd74154aa3f534f95506b5c0dd66e1c6d901e091`. All 10 native executables, 27 Python
tests, syntax/whitespace, pinned build, radio and dashboard checks passed. The
existing event-core macro redefinition warning remains. The owner's independent
229-case simulation and 2,371-row physical replay reproduce the archived results
exactly, with unchanged model limitations and possible sharp catches.
[Verification](https://github.com/amcchord/Hopscotch/blob/e9e1c65996c5f6539a704447516834f2ce12527c/evidence/lowering-v2-integration/verification.json).

A separate private credentialed package is queued at
`worktrees/drive-braking/artifacts/lowering-v2/candidate/`: 1,196,112 bytes, full
SHA-256 `71ca02be6f0471a63f92605f6ae0204acd45a58539a5eec7da0e2328a9542d72`,
ESP digest `7668a0215df34b7e5c0030a23705ba8016343260d1e6bab7aacfe202897c7190`.
The [manifest](https://github.com/amcchord/Hopscotch/blob/e9e1c65996c5f6539a704447516834f2ce12527c/evidence/lowering-v2-integration/candidate-manifest.json) explicitly marks it
`queued_not_installed`, schema 4/240 bytes/features 8191. Root independently
checked the candidate size/digests, preservation of the installed package, and
that only the three reported lowering files differ from deployed `eb2bb23`,
with those files identical to the upstream handoff.

Updated CURRENT/JOURNAL only. Root firmware remains byte-identical to installed
`43b1967` and its verified app1 identity; no candidate source was merged here.
No device requests, restart, OTA, configuration, arming or movement occurred.
Did not repeat the unchanged build/model checks for this records-only update.
Next step remains separately authorized deployment with fresh preflight followed
by a restrained operator trial. Hold further trials of installed lowering v1.

## 2026-09-20 — Integrate verified v2 deployment, faster OTA workflow and failed trial

Merged lowering handoff `d3eb777` into root `codex/wifi-ota` as `721f2ce`,
then integrated trial evidence `d58c917`. Production source/build/dashboard/radio
files match installed `dd74154` exactly. The deployment owner installed the
already validated private package in app0, confirmed its ESP digest and powered
disarmed health, and preserved the earlier 2,371-row CSV/wire unchanged. The
immutable package manifest's queued status is preparation history; the separate
[deployment record](../../evidence/ota-lowering-v2/README.md) establishes the
installation. Root performed no device requests, upload, restart or movement.

Two transmitter-on paced uploads disconnected without changing the running
image or rebooting. The same pacing succeeded with the transmitter off in
62.759 seconds. The underlying cause remains unconfirmed. The host helper now
uses one command for manifest/preflight checks, validated run backup, paced
upload, image/health verification and preserved-run comparison. Lost upload
acknowledgments trigger discovery; already-installed bytes are verified without
another flash. Ten focused host tests pass, including localhost multipart
transport. No distinct hardware trial of the refactored CLI is claimed.

README, architecture, test/lowering/OTA guides and shared current state now use
the installed v2 identity and frozen-package workflow. Reused completed firmware
and model validation from release record `e9e1c65`; did not rebuild unchanged
firmware or repeat those checks. Source identity, local image hashes, new helper
checks, archived-log integrity, documentation links and whitespace provide the
integration checks. Exact wire bytes remain unmodified.

Austin reported v2 arms too far forward, an apparent bounce and backward fall.
The [new trial archive](../../evidence/balance-lower/trial-v2-20260920/README.md)
contains 1,984 rows/39.760 seconds, ending `lower_wrong_direction`. Arm motion
was still about 1.3 rad/s at inferred contact; body rate reversed sharply before
support qualified. The log ends at the fault, before the reported later fall.
Post-commitment replay matches; full preparation replay does not, and that
limitation is preserved. Hold further lowering attempts. The lowering owner
retains device/log access and is revising contact modeling and departure timing.

The new `codex/ota-reliability` worktree owns transmitter-on transport fixes;
root did not edit network source or compete for device access. Radio checkpoint
`05e25ab` records completed Lua v3.1 SD installation; boot confirmation remains
pending. Publish these completed records and coordinated WIP branch checkpoints
under Austin's existing all-work GitHub authorization, excluding private
configuration, credentialed binaries and local caches. Next: focused candidate
handoffs from the lowering and OTA owners, then a coordinated release.

## 2026-09-20 — Complete all-agent publication snapshot

Published root integration `9477a87` and radio install record `05e25ab`.
The OTA owner published source `67f3746` with evidence/package record `50dfdf2`;
the lowering owner explicitly checkpointed unfinished model work as `b7d52fd`.
Both remain separate from the installed-source root. All nine local project
branches have matching GitHub checkpoints, preserving past work and coordinated
in-progress snapshots. All six working checkouts were clean at the checkpoint
inventory; owners may continue after this snapshot. No forced history updates,
worktree removal, private configuration or credentialed-binary publication.

The root helper's 10 host checks, 278 relative documentation links, image/source
identity and both archived CSV/wire pairs passed; see
[handoff checks](../../evidence/integration-2026-09-20/ota-v2-handoff.json).
Publication scans found no local credentials or private keys in the reviewed
outgoing/index blobs. No robot operations occurred in this publication task.

## 2026-09-20 — Install v3 first-contact yielding, OTA transport 2 and long-log export fix

Austin reported v2 contacted with its arms too far forward, rebounded and fell
backward. Forked the transmitter-on OTA work into its own contained
`worktrees/ota-reliability`; kept motion work in `worktrees/balance-lower`.
Integrated completed OTA checkpoints `67f3746`/`50dfdf2` and root history
`045e956` without editing their checkouts. Their tasks were subsequently
archived; this task retains coordinated device ownership.

V3 source `86f921e` stops both advancing targets on first contact, yields a
bounded amount on measured deceleration and makes small preparation/launch
adjustments. Moving-arm contact damping was added to the paired simulation.
304 cases improve from 231 to 248 completions, with four retained regressions;
asymmetry/impact speed remain limits. Combined source `d8613e6` passed native,
Python, radio/dashboard, transport and pinned build checks. Source preservation
confirms ordinary ground/standing drive, startup, radio and dashboard unchanged.

The first normal updater's preflight GET log timed out and the old v2 image
rebooted with task-watchdog reset reason 6. No upload occurred. A newer long run
had replaced the archived rebound run. Source `8449ddb` makes checksum/CSV work
yield every 10 ms, without disabling/feeding watchdogs, and extends eligible
log-download timeouts. Native maximum-row/rollover/checksum tests and updater
host tests passed, followed by a final configured pinned build; unchanged motion
models and other checks were reused.

A documented one-time application-only recovery installed `8449ddb` into app1
in 62.842 seconds with the transmitter off. Exact image/slot and powered disarmed
health passed. The latest run could not be backed up through the old exporter;
filesystem preservation allowed the new exporter to recover its valid 4,789
samples / 96.010 seconds immediately afterward without another reset. Austin
also explicitly allowed discarding the log, but no deletion was necessary.
This later run ends `bailout_angle_error` without entering forward commitment
or catching. The earlier rebound trace remains the v3 design input.

[Release evidence](../../evidence/ota-lowering-v3/README.md) records both the
failed preflight and successful recovery, exact package/digests, old rollback
packages, tests and limits. Documentation now points to installed v3 and retains
a single-command, frozen-package fast update path. No autonomous motion,
calibration/settings write, filesystem upload, secret copying or changes to
other agents' checkouts. Next: transmitter-on disarmed gap/reinstall acceptance,
then a restrained operator v3 trial with telemetry.
