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

## 2026-09-20 — Install continuous arm return and integrate CH6 fast tip-up

Austin's v3 trial stopped upright supported by the arms; one CH12 press then
brought it flat. Retrieved/validated 1,908 rows, 38.235 s from installed `8449ddb`.
The recorded end was lower_wrong_direction, not successful completion. At first
contact, outward arm velocity persisted into the next 20 ms frame despite a small
target reversal, while body rate rebounded to +45.776 deg/s. Seeded pinned replay
matches the fault; the manual CH12 action is outside the saved trace.

Lowering checkpoint f4fafc7 changes the catch to continuous return toward Forward
after two independent contacts, retaining a small one-arm retreat and sampled
motion/health/deadline limits. Brief loaded rebound may settle while returning;
measured flat/Forward remains mandatory for completion. Revised simulation uses
actual policy motor-speed limits plus delay/acceleration sensitivities.329 paired
cases:257→260 complete,7 improve/4 regress, all 11 injected failures reject
completion. Stiff delayed impacts still fail and post-fault motion is not modeled.
Native regressions cover the recorded input sequence and continued return.

Coordinated the other active task's CH6 fast tip-up in its own checkout. Reviewed
and merged `3814f1f` into combined e55cecb, resolving only feature-mask/adjacent
metadata conflicts to 65535. Its 162 tracking cases reach quiet capture within
2.74 s; this is not physical contact-dynamics validation. CH6 LOW preserves slow
standing. Ground/standing drive, radio/dashboard/network and old log metadata
are preserved. No other owner's checkout was edited.

Combined 11-native/37-Python, syntax/whitespace, configured pinned build,
radio/Lua, dashboard and OTA harness checks passed once. Frozen configured app
is 1,203,520 bytes, whole-file SHA-256 6b648fbb0c584146954b7f02d9d34d2e5f24cd06be7965169fcf5f2d078ff808,
ESP digest 62e38da80f1a6e61bb8ed1162c5bf4f83b280d0001d82469f500131dc96f7f42.
Retained exact prior v3/v2/v5 packages for recovery and reused unchanged models.

Installed into app0 and verified 2026-09-20 19:31:57UTC using normal updater
checks, with transmitter on and an injected 4 s sender gap after 256 KiB. HTTP200
arrived after 249.699 s;67 successful RC observations stayed fresh/disarmed,
5 read-only monitor requests timed out. Exact digest/slot, powered disabled
motor health, fresh IMU and released maintenance passed; 1,908-row CSV and wire
were identical across OTA. No autonomous motion or settings/filesystem writes.
The slower monitored throughput is recorded, without attributing a precise RF
cause; normal deployment omits the gap/monitoring. Docs now permit transmitter-on
updates while disarmed and retain transmitter-off as an observed faster option.

[Release record](../../evidence/lowering-v4-fast-integration/README.md) contains
manifest, source audit, validation and full deployment evidence. Notified the
fast tip-up owner of inclusion and installation. Next: a restrained v4 lowering
trial with CH6 LOW and no CH12 assist, then separate fast-standing test; archive
telemetry before each subsequent run. Both physical motion acceptances remain
pending despite successful OTA.

## 2026-09-20 — Install earlier forward departure and fast startup feedback fix

Austin's latest v4 tests showed a backward lean during arm lowering and a flip
at contact. Archived the latest 1,237-row / 24.745-second run directly from the
robot. During 6.2 seconds of preparation the scheduled target moved +4.465°
backward and the body followed +4.463°. Contact rebounded to +105.041°/s and
ended lower_motion_limit. This is the latest retained run, not all operator
attempts. The v4 CSV/wire and reproducible analysis are preserved.

Lowering v5 caps the preparation balance target at its starting measured tilt,
permits forward departure while the arms deploy and hands off without a quiet
upright dwell. Catch initially parks, probes only after measured forward drop,
and reverses both arms on qualified contact. Independent two-arm support then
allows continuous return to measured flat/Forward. Bounded arm lead, sampled
body limit, freshness/load/deadline checks remain; persistent opposite wheel
motion now aborts. Lowering-only wheel handoff permits the measured velocity
needed during preparation without abrupt clipping. Ordinary ground and standing
drive remain unchanged.

Paired production-policy model: 261 → 253 completions across 329 plants, with
43 improvements and 51 regressions; 43 regressions use the widest unmeasured
pivot. All 72 trial-informed contact cases complete. Delayed contacts improve
8 → 15/24, and all 11 injected failures reject completion. Geometry, contact
and servo dynamics are not identified; sampled guards cannot bound all
between-tick peaks, and post-fault falling is not simulated. This remains an
experimental manual-test build, not physical acceptance.

Integrated the fast tip-up owner's 9771bee as 3f3bc37 without editing its
checkout. Its post-setup fresh-feedback wait, real bounded motor requests and
visible start status address the reported CH6 HIGH refusal; trajectory and slow
standing are preserved. Combined checks passed: 11 native executables, 37 Python
tests, syntax/whitespace, dashboard and pinned configured build. Existing radio
and transport acceptance was reused because those paths did not change.

Intermediate source 6e8c486 installed into app1 in 63.65 seconds and retained
the saved run. Before handoff, review found that two volatile writes briefly
published the uncapped target. Final source 17c499c publishes only the final
capped value; its targeted configured rebuild passed, with unchanged policy
checks and model evidence reused. Final application is 1,206,320 bytes,
whole-file SHA-256 44a3cd6c1c1e71f845f1ba8ae9a26c9ebc3f44bc9c6d8c6a20a8cb2aa2a297d7,
ESP digest 8373d0b5be1ba254b5b9e831849e122989f66523c0ea35cf7dfcd1795a2ed837.

One final-image attempt stopped in saved-log backup before uploading; the next
stopped after 148,480 sent bytes at 84.074 seconds with poor Wi-Fi near −83 dBm.
Power-on resets were observed and the intermediate app1 digest remained active.
Neither attempt was accepted as success. Austin repositioned the robot and
confirmed steady power. Signal recovered near −54 dBm; fresh identity and idle
eligibility were checked before retrying the same frozen package.

Final normal OTA completed in 187.997 seconds with the transmitter on, verified
2026-09-20 20:35:49 UTC in app0. All six powered motors returned online, error-free
and disabled; fresh IMU, disarmed state and released maintenance passed. The
1,237-row v4 CSV and wire exports were byte-identical. No autonomous motion,
settings/calibration change, filesystem upload, credential copying or unrelated
checkout edits occurred. The [release record](../../evidence/lowering-v5-integration/README.md)
retains all attempts, manifest, validation, hashes and exact v4 recovery package.

Updated the current state and single-command frozen-package OTA guide to use
the integration checkout's current helper and read the private header in place.
Recorded steady power/strong signal recovery without attributing a unique RF
cause. Notified the fast owner of installation; it independently confirmed its
owned source matches and retained no device actions. Next: CH6 LOW ordinary
stand-up, one CH11 lowering trial, disarm and archive before another run. Test
CH6 HIGH separately afterward; both revised motions await physical acceptance.

## 2026-09-20 — Diagnose v5 supported stop and prepare earlier handoff

Austin reports that v5 reliably leans forward but stops supported on the arms;
fast standing separately rolls forward. Preserved the newest slow/lowering run
(1,219 rows / 24.365 s, source17c499c) after Wi-Fi became reachable. The run ends
lower_prepare_disturbed before commitment: right rear wheel6.094rad/s exceeds
the6rad/s bound while the last arm reaches preparation tolerance. Forward body
movement2.766° and the effective setpoint ceiling are confirmed; no actual
catch/return samples exist. The reported support follows the recorded abort.

V6 adds a moving, unloaded two-arm handoff at1.6rad plus measured forward
motion, preserving all prior limits and return logic. Native replay reproduces
the old fault and reaches commitment39ms earlier with wheels1.695/4.490rad/s.
Replay stops at divergence. The paired329-case model gives253→255 completions,
9 improvements/7 regressions; all72 trial-informed contact cases complete and
all11 injected faults reject completion. Wider1.35rad departure and a version
without load qualification were rejected. Native contact continuation and
blocked-handoff regressions passed. Combined validation/build is pending the
other owner's fast correction; no v6 OTA has occurred.

Requested one separate fast trial and immediately archived236 rows/4.750s,
fast bit128 set, ending bailout_angle_error. Supplied read-only paths to the
fast owner without copying logs between worktrees. Its initial analysis finds
quiet capture, then a support-biased equilibrium shift during arm return.
The owner is investigating a bounded fast-only correction; this task retains
all robot operations, combined schema6 metadata and final release ownership.

## 2026-09-20 — Install coordinated lowering v6 and fast support release

Integrated fast owner f4d2bb7 as882ba98; its quiet supported capture remains a
transient offset rather than a permanent−2.86° equilibrium correction. Initial
measured arm return releases that offset; independent wheel capture gates were
also added. Slow capture arithmetic, trajectory, return speed and shared driving
gains remain unchanged. Fast model24/45 without a modeled fall before/after,10 improvements
and10 regressions; saved-trim uncertainty remains documented.

Combined source a772ecc passed11native/37Python,syntax/whitespace and pinned
configured build once. Existing radio/dashboard/OTA acceptance was reused.
Schema6 identifies both new policies while retaining original schema5 export
metadata. Frozen application1,207,616bytes; whole-file SHA-256
be068a3c2e02ac4343d6c15795984577d2067cceebf0c069e97be9a91b511360, ESP digest
fdf7af6a96d2b6e3c3c908301ae765fcfa4c9110df27f8d88693dd80e8aeb2c2. Exact prior v5/v4 packages retained.

Verified installed in app1 at2026-09-20T21:05:08.644323+00:00; normal OTA transfer342.569s. Exact image,
powered disabled motor feedback, fresh IMU, disarmed state and released
maintenance passed. The236-row fast-trial CSV and wire were byte-identical.
No autonomous motion, settings/calibration/filesystem writes or other checkout
edits. [Release record](../../evidence/lowering-v6-integration/README.md).
Next: separate manual lowering withCH6LOW and fast-standing trials, archiving
each run after disarming. Physical acceptance remains pending.

## 2026-09-20 — Preserve successful fast stand-up; install lowering v7 and OTA progress

Archived the 1,519-row physical v6 run. Fast v2 succeeded through arm return and
recovery; its owner retained production unchanged and supplied success evidence
1a2e30b (integrated `93f997f`). Later CH11 waited 12.525 s in the neutral driving latch,
then contacted both arms but never confirmed rocking support; lower_wrong_direction
ended the run. Earlier departure now works, so preparation timing was preserved.

Lowering c2dd50c adds explicit zero-reference handoff to stationary PD, bounded
wheel braking after both contacts, and recent per-arm support qualification.
An initial native single-impulse test exposed confirmation after only 60 ms; fixed
with real 80 ms elapsed from BOTH first contacts and verified independently by
the fast owner. All global motion/support-loss guards remain. Model 255 → 258/329,
3 improvements / 0 regressions; 72/72 contact cases and 11 injected faults retain outcomes.
Fixed-input replay later rejects old support loss; new braking changes that
trajectory, so hardware acceptance remains outstanding.

OTA owner f58f0d9 integrated as `660634a`: screen progress plus control-owner CRSF
suspension before flash grant, retaining transport2 and cooperative export.
Resolved web method signatures/diagnostics and adapted actual-code lifecycle
and transport harnesses. No other checkout modified. Final source `c442e12` passed
12 native / 38 Python tests, configured pinned build, radio, dashboard and OTA/TCP checks.
Frozen application 1,210,736 bytes, whole SHA
aaef0c5f2c768ff36cd51f3d41ec7ff08a55bb682c2995be958c55901635b59a, ESP digest
5affbef55fc259591b2737cf1f17226b5878f5ac2ce847c7694919d755c975a4. No production source changed after validation.

Verified installed in app0 at 2026-09-20T21:36:50.532600+00:00; transfer 421.475 s. Exact image/slot, fresh
IMU, all six powered motors online/error-free/disabled, disarmed state and
released maintenance verified. Original schema6 CSV/wire remained byte-identical.
No motion or settings/filesystem writes. New OTA screen/UART pause only applies
to subsequent uploads and remains hardware-unobserved. Exact previous v6 package
retained; no automatic rollback. [Release evidence](../../evidence/lowering-v7-integration/README.md).
Next: operator CH6 LOW stand-up and one CH11 trial, disarm and archive immediately.

## 2026-09-20 — Successful v7 trial and faster supported return v8

Austin reported both fast standing and laydown worked. Archived 1,759 rows /
35.195 s from c442e12, schema7, ending lower_complete. Fast capture 2.890 s,
return complete 5.131 s, recovery settled 6.871 s. Lowering took 14.109 s;
supported descent took 11.370 s with at most 0.021/0.011 rad target error.
Final tilt −2.818°, rate −0.455°/s, Forward error −0.027/+0.007 rad.
The former stop gate completed in 0.479 s and contact support in 0.240 s.

Source 47eb19a changes one motion parameter: supported target speed 0.16 to
0.24 rad/s. Catch, all guards, motor cap and final landing remain unchanged.
Final retraction already takes 0.740 s and approaches its body-rate guard;
retained that speed. The same 329-case model retains 258 completions and all
outcomes, with median 2.72 s saved among common successes. 12 native/38 Python,
syntax and pinned build pass; final metadata indentation rebuild also passed.
No other agent checkout edited. Fast and OTA owners informed; no added changes.

Verified app-only OTA in app1 at 2026-09-20T22:12:58.795503+00:00, transfer 475.235 s. Application
1,211,120 bytes, whole SHA 5dfd2201ac0a0d986325dfa913980467cdba47c7d53db7797db24149b188cb80,
ESP digest 6b0b637e58e45c5f0b5c54521acdd81ab8d3f9ca56cd7d54311e56b8cbcc50d1. Exact image/slot, powered disabled health,
fresh IMU, disarmed/released-maintenance state and identical successful log
exports verified. One in-flight snapshot confirmed active OTA, maintenance, disabled motors and cleared RC/link input. The transmitter was linked before and after reboot. This is consistent with the installed UART suspension; the physical progress screen was not independently observed.
No motion/settings/filesystem writes. Exact successful v7 rollback retained.
[Release](../../evidence/lowering-v8-integration/README.md). Next: operator trial
of faster supported return, disarm and archive before another attempt.

## 2026-09-20 — Diagnose catch stop and install bounded confirmation v9

Austin reported another excellent fast tip-up, but lowering stopped on its arms.
Archived 662 rows / 13.290 s from v8 source 47eb19a; fast captures at 2.890 s.
Both arms contact at 12.911 s. A less-than-one-degree loaded rebound peaks near
+17 degrees/s, repeatedly resetting the +12 confirmation gate. A second rebound
outside the 300 ms grace ends lower_wrong_direction at 13.290 s. The faster
supported return is never reached, so this run does not test its speed.

Source `b9763c2f5e2d66dcc28da4b1bc568dac29751b17` adds a bounded qualification exception: at most +20 degrees/s,
1.5 degrees above the impact minimum, within 300 ms, retaining both recent
loads, measured arm-return velocities and full 80 ms dwell. All motion limits,
support-loss/progress/time guards and 0.24-rad/s return remain. Fast, ordinary
driving and OTA code unchanged; other checkouts untouched. Native regression
covers the recorded six frames and rejects invalid support/rate/rise/age/arm
motion. Combined 12 native/38 Python, syntax and pinned build pass once.
Model 258/329 before/after with no outcome changes; recorded replay confirms
support at 13.011 s but later old inputs still reject support loss after commands
diverge. This is not hardware proof. Previous successful replay is unchanged.

Frozen application 1,211,568 bytes; whole SHA
6ca34f3ab58994b60c03e4db73b85540d5755909ba96fcb9aae3b1426d9cf3f8, ESP digest
361386bf17396fa11b7086ef272e813b49ee7a195c4d94bc1e38d19596ce98e8. Installed in app0 at 2026-09-20T22:38:58.351613+00:00; transfer 392.262 s.
The first upload disconnected after 277,504 bytes; read-only recovery verified
the old image, released maintenance and healthy idle state before retrying
the same frozen bytes. Exact identity, six powered healthy disabled motors, fresh IMU, disarmed and
released maintenance verified. All 662 schema8 rows and wire export unchanged.
No autonomous motion, settings/filesystem writes or other task edits.
Known physically successful v7 and exact immediate v8 packages retained.
[Release](../../evidence/lowering-v9-integration/README.md).
Next: operator stand/CH11 attempt, disarm and archive before another run.

## 2026-09-20 — Successful v9 trial and CH6 fast laydown v10

Archived the operator-confirmed successful v9 run: 1,462 rows / 29.750 s,
lower_complete, fast capture 3.026 s. Laydown 12.009 s, supported descent 9.900 s,
final retraction 0.139 s, final tilt -2.368 degrees and Forward errors -0.025/+0.005 rad.
Close tracking and frequent body-rate pauses informed a separate fast schedule.

Source `7917543beedaf4902d1661461f18380959ad100a` latches CH6 at each accepted CH11 lowering request. HIGH
selects fast supported return; LOW/center retain normal v9. After support,
600 ms blend toward 0.60 rad/s target, 0.75 motor cap and 20 degrees/s descent
pause; floor taper 35 to 15 degrees. Initial fall/catch, final landing and all
support-loss/global/time/progress guards remain. Fast standing, driving and
OTA source unchanged; no other checkout edited. Combined 12 native/38 Python,
syntax and pinned build passed once. Model retains all 329 outcomes, 258 complete;
normal outputs exact, common fast successes median 2.04 s quicker. Three normal
recorded replays identical to v9; fast divergence only after support. Physical
fast acceptance pending; model is approximate.

Frozen application 1,212,352 bytes, whole SHA
cf873cee0c52614dd179924c41c2466cc39ba1704fc5b1f449212bc25cb18234, ESP digest
42ed3918122860f6902ae0a1c8915ab771dd353b403de1de6d40fcf2d3cad4fe. App-only OTA 90.96 s, verified app1 at 2026-09-20T22:51:48.370997+00:00.
Six powered healthy disabled motors, fresh IMU, disarmed/released maintenance
and exact successful CSV/wire retention verified. No autonomous motion or
settings/filesystem writes. Successful v9 recovery and older packages retained.
[Release](../../evidence/lowering-v10-integration/README.md). Next: operator
CH6 HIGH / CH11 trial, disarm and archive before another run.

## 2026-09-20 — Queue OTA throughput handoff without changing installed v10

OTA owner supplied local candidate 45c1a94 on codex/ota-throughput, isolated
worktrees/ota-throughput, based on d5f18fa. Reviewed the focused diff and handoff:
all-channel strongest-signal association, AP/channel diagnostics, receiver
write/verify/gap metrics and optional 16 KiB unpaced host sends; paced default,
control-owned maintenance, saved-log/image/health checks and no automatic retry
remain. Owner reports 12 native/42 Python, pinned build, transport, radio and
dashboard validation; no hardware speedup or strongest-AP observation claimed.

Integration checkout is clean at d5f18fa before this record; network source
matches installed 7917543. No OTA is in flight. Candidate is queued, not merged,
built here or installed. No robot request, reconnect or upload performed for
this handoff. Next remains Austin's manual CH6 fast-laydown trial; integrate
network candidate with the next authorized combined release and retain the
proven v10 package until that deployment is verified.

## 2026-09-20 — Hand authorized network deployment to OTA owner

OTA task reports Austin explicitly authorized uploading the strongest-AP /
throughput changes. Confirmed no robot operation or newer motion candidate
is underway here: src/data/build/helper are unchanged from installed v10
source 7917543; a557e62 is a documentation-only queue record. Handed exclusive
release/device ownership to worktrees/ota-throughput, codex/ota-throughput,
for its unchanged 45c1a94 candidate on the same motion baseline. It owns fresh
preflight, latest-run archive, source/image freeze, authorized fast upload,
complete image/health/log verification and its deployment/current-state record.

No robot requests or deployment were performed by this task. Preserve exact
private v10 recovery in place; no secret/log/binary copies between worktrees.
This task waits for the OTA owner to report exact installed identity/outcome
and return ownership before any subsequent device operation. Physical fast
laydown acceptance remains pending unless a newer archived run establishes it.


## 2026-09-20 — Investigate AP choice and prepare faster OTA sender

Austin asked whether the ESP32 connects to the first AP instead of the strongest,
and requested investigation/improvement of approximately 5 kB/s uploads. Created
`worktrees/ota-throughput` / `codex/ota-throughput` from v10 record `d5f18fa`;
root and other task checkouts remain untouched. The lowering task retains robot
and release ownership and received a coordination notice before implementation.

Pinned Arduino 2.0.16 code confirms FAST_SCAN by default. Added explicit
ALL_CHANNEL_SCAN and signal sorting before first association, retained for
maintenance-only reconnects. No roaming during motion/upload. Existing Wi-Fi
sleep is already off. Host 1 KiB/50 ms pacing adds approximately 59 seconds to
current images and caps the sender at 20 KiB/s. Recorded v7/v8/v9 transfers were
2.49–3.02 KiB/s with RC linked; v10 was 13.016 KiB/s unlinked. Conditions differ;
this is not causal RF/AP evidence. Existing Update uses buffered writes/block
erases; no flash or control scheduling change was justified without timings.

Added optional fast profile (16 KiB sends, no artificial sleep) with TCP
backpressure, preserved paced default, manifest/preflight/log/health validation
and no implicit retry. Added AP BSSID/channel, receiver flash-write/verification/
receive-gap timing, and host send/sleep/response timing. Success timing headers
are captured in deployment records before reboot clears device RAM. These
measurements need no extra stress test or in-flight polling.

Consolidated 12 native / 42 Python checks, syntax/whitespace and configured pinned
ESP32 build pass. Production lifecycle harness verifies response timing headers,
metrics and safety ordering; pinned TCP/OTA transport, radio C++/Lua and dashboard
checks pass. Only the established event-core macro warning remains. Local build
configuration reads the existing root private header in place; no secrets,
dependency trees or private binaries were copied/published. Motion/control/RC/
display/dashboard source is unchanged from the v10 baseline.

[Investigation and release procedure](../OTA_THROUGHPUT_2026-09.md),
[evidence and validation](../../evidence/ota-throughput/README.md),
[derived baseline rates](../../evidence/ota-throughput/baseline-rates.json).
No robot requests, uploads, reconnects, restarts, settings changes, motion or
GitHub pushes occurred. Candidate is not installed; speedup and strongest-AP
selection await observation. Next: send focused commit to the integration owner
for the next authorized combined release and opt-in fast-profile comparison.

Candidate source/test/evidence commit `45c1a94` was sent to the lowering/integration
owner with exact checks, opt-in fast command, limitations and no-robot-action
status. Shared operating/current/journal records follow in a separate commit.

## 2026-09-20 — Deploy strongest-AP / OTA throughput update

Austin authorized pushing the prepared firmware while the robot was powered.
The lowering/integration owner confirmed installed v10 source `7917543` and
transferred exclusive device/release ownership for this release. All work stayed
in `worktrees/ota-throughput` / `codex/ota-throughput`; root and other checkouts
were not edited. Reused the exact validated configured candidate at source
`45c1a94`, with 12 native / 42 Python, pinned build, TCP, radio and dashboard
checks. Verified existing private configuration in the binary without exposing
values. Application 1,214,928 bytes, whole SHA
`b3f12485e2b61dcc1f4dd11c120beffbac46a41e18bfe8847bdb77221d948109`, ESP digest
`a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61`.
Private frozen package is `artifacts/ota-throughput/release/`; immediate v10
rollback verified in place in the lowering worktree without copying it.

Fresh preflight verified healthy powered disabled motors, disarmed IDLE, fresh
IMU and RC link up. Each attempt archived the identical 1,462-row saved run.
The first fast-profile transfer queued the image locally but the 120-second
HTTP response wait timed out; the old helper retried the response wait then
closed. Postflight timed out while the old app was still receiving. Read-only
recovery observed 1,071,102 bytes and active motor/RC suppression; Austin
confirmed the physical display percentage was increasing. The device later
aborted with inactivity_timeout at 1,082,590 bytes. V10/app1, released maintenance,
RC return and all six healthy disabled motors were verified before retry.

Host-only fix `243bff2` allows 900 seconds for the receiver's final response,
retains 120-second connect/send timeouts, and avoids reusing a timed-out HTTP
reader. All 16 focused helper regressions passed, including actual delayed
localhost acknowledgment, early HTTP rejection and one-close failure handling.
Firmware was neither changed nor rebuilt. A deliberate retry of identical bytes
received HTTP 200 after 305.815 seconds, 3.880 KiB/s; send blocking 275.442 s,
max send 15.213 s, response wait 30.363 s, zero pacing sleep. This is not a
controlled comparison and still used the old firmware/AP connection.

Exact target digest verified in app0 at 2026-09-21T00:57:10.360018+00:00. All six
powered motors healthy and disabled, fresh IMU, disarmed groups, released
maintenance and RC link restored. Saved CSV/wire byte-identical before/after.
The new image reports all-channel strongest-signal selection and metrics v1.
Its initial association was about -70 dBm/channel 6; one maintenance-only
reconnect after fresh safety checks selected a different BSSID on channel 1,
improving -69 to -55 dBm. Image/health/RC reverified at
2026-09-21T00:57:59.334995+00:00. No repeated scans or motion were initiated.

[Deployment evidence](../../evidence/ota-throughput-deployment/README.md)
retains both attempts, safe recovery, AP comparison, screen observation and
source preservation. No settings/filesystem writes or GitHub push requested.
Motion/RC/display production source remains v10-identical. Next: hand exact
identity, helper fix and records back to the lowering/device owner for source
integration before any later firmware build. Observe speed/receiver timings
on the next normal authorized OTA rather than adding a benchmark reflash.

Completed release records committed as `a2c783b`; exact installed identity,
recovery/verification results, AP comparison, host fix and source-integration
requirement were sent to the lowering task. Exclusive device/release ownership
has returned to that task. No further robot requests are planned here.

## 2026-09-20 — Integrate verified networking release and resume device ownership

Merged codex/ota-throughput through 804fa5d into codex/balance-lower, including
installed firmware 45c1a94, host response-timeout fix 243bff2 and deployment/AP
records. Only CURRENT/JOURNAL conflicted; preserved both task histories and
reconciled current installed identity and ownership. Device ownership has
returned here. Firmware src/data/build inputs exactly match the installed
source, and scripts/tests exactly match the reviewed OTA branch. Existing
candidate validation and 16 focused host-fix tests are reused; no firmware
rebuild, benchmark reflash or robot request was performed for integration.

Installed app0 ESP digest a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61,
verified by the OTA owner at 2026-09-21T00:57:10.360018+00:00. Six healthy powered
disabled motors, fresh IMU, released maintenance, RC return and exact 1,462-row
CSV/wire retention passed. A safe reconnect selected channel 1 at -55 dBm after
channel 6 at -69 dBm; post-reconnect health passed at 00:57:59.334995 UTC. No
throughput improvement on the selected AP is claimed yet. Private package
remains in ota-throughput; no package, private header or raw export was copied.

Next: operator CH6 HIGH / CH11 fast-laydown trial, disarm and archive. Future
ordinary OTA can collect new receiver timings without an extra benchmark run.

## 2026-09-20 — Archive failed fast stand-up for fast-tip owner

Fast-tip task relayed Austin's request to download a new fast stand-up fall.
Verified installed 45c1a94/app0, fresh IDLE/disarmed, saving finished, powered
healthy disabled motors and maintenance eligibility. Archived 291 samples /
5.906 s, schema 10, bailout_angle_error; CSV/wire validated and
preserved byte-for-byte. Post-export fresh idle/health/maintenance-release
verification passed. [Evidence](../../evidence/balance-lower/fast-tip-fall-20260921T010806Z/README.md).

Sent exact read-in-place paths, hashes and image identity to fast-tip task,
which owns analysis. No raw log copies between worktrees, no autonomous motion,
settings change or OTA. This task retains device ownership and holds updates
until the trace diagnosis is reviewed. No cause inferred from exit reason alone.

## 2026-09-20 — Integrate failed fast-stand analysis without tuning

Cherry-picked fast owner's evidence/script-only 8ffece2 as f191d9c. Quiet capture
at 2.981 s precedes the failure. At 5.506 s rear-left command/measured speed is
-21.276/-7.307 rad/s, right -18.276/-17.951, both feedback ages 1 ms. Left response
fits -9.607 rad/s² (R² 0.9983); the successful v9 comparison tracks symmetrically.
Austin confirms twisting and cannot fully exclude an obstacle. No contemporaneous
setup serial was found. Unapplied acceleration setup, physical resistance or
another motor-side issue remain hypotheses; fitted response is not register
readback. No timing stall or networking cause is established.

[Derived evidence](../../evidence/fast-tip-up/fall-20260921/README.md) and plot
reviewed, input hashes and representative sample checked against original files,
script syntax checked, production inputs still exactly match installed 45c1a94.
No raw exports copied between worktrees, no code tuning, firmware build, motor
probe, run or OTA performed. Existing firmware tests need no repeat for evidence
integration. Device ownership remains here; investigate the left response before
choosing a correction, retaining valid current-limit checks and the write-only
ACC_RAD constraint. The download/review request is complete.

## 2026-09-20 — Preserve successful fast laydown; hand stand-up drift to owner

Fast-tip task relayed another successful test and Austin's request to reduce
stand-up drift. Archived 1,325 schema 10 samples / 26.660 s, lower_complete, with
fresh disarmed/saving-finished checks and verified post-export idle/health.
Source 45c1a94/app0 remains installed. CSV/wire hashes and exact read-in-place
paths sent to fast-tip task; no raw log copies between worktrees.

CH6 fast-lowering flag is set throughout the successful lower. CH11 starts
18.421 s; supported descent 19.721–25.891 s; completion 26.660 s. Total 8.239 s,
supported 6.170 s, compared with normal v9's 12.009/9.900 s. Final tilt -1.814 degrees,
rate -0.182 degrees/s, Forward errors -0.028/+0.006 rad. This first physical fast
success is 3.770 s shorter overall, not a controlled comparison or reliability
estimate. [Archive/result](../../evidence/balance-lower/fast-tip-drift-20260921T011755Z/README.md).

Fast-tip task owns stand-up capture/equilibrium/drift analysis and any source
candidate, starting from exact installed code. Austin's suspected angle drift
is a hypothesis to test against the recordings. This task retains device and
integration ownership; no overlapping motor/stand-up changes, motion, settings
writes or OTA. Current download/record work is complete; await the owner's
justified candidate before release work.

## 2026-09-20 — Review and queue fast stand-up drift experiment, no OTA

Reviewed fast-tip owner's e08184d/cf7db90/32c4b68 against installed 45c1a94.
Only runtime gain change is the latched fast stand-up's existing maximum 800 ms
recovery boost, 1.0 to 0.5. Slow and ordinary learning, freshness/bounds/recoil,
capture/trim, motion trajectories, lowering and networking remain unchanged.
Schema 11 identifies the policy with the existing 240-byte layout. Found and
reproduced the old pilot test rejecting schema 11; owner corrected it before
handoff. No files in the owner's checkout were edited here.

The successful recorded run accumulated +2.8529 degrees before a larger backward
recoil with healthy symmetric wheel tracking. That supports a focused gain trial,
not an accelerometer-drift diagnosis. The unvalidated planar screen has 57 fall
improvements and 20 regressions (116→153 no-fall of 385); many no-fall cases do
not settle. Regressions include case 223 at A23/B8/lag .035/arm delta -3.5/
Forward equilibrium 88.7/release .99, so they are not exclusively extreme motor
lag. Review recommendation: queue an explicit operator-test experiment, with
no demonstrated general robustness or physical drift improvement claimed.

Cherry-picked only the focused commits as 2c90263/a0243d9/db437fa, excluding the
peer's baseline-alignment commit df5ed26. Production src/data/platformio inputs
match the peer candidate exactly. One combined validation at db437fa passes all
12 native suites, 44 Python tests, script syntax and whitespace checks, and
the configured pinned ESP32 build (application section 1,214,905 bytes,
RAM 53,600 bytes). Local output/fast-tip-v3-integration-check.log retains the
console record. Existing Arduino macro-redefinition warnings remain.

No raw logs, secrets or firmware packages copied between worktrees. No robot
request, OTA, settings write or autonomous motion. Installed source remains
45c1a94/app0 with successful v10 fast laydown. Candidate is queued and unflashed;
return actual status to the fast-tip task for Austin's next-test decision.

## 2026-09-20 — Preserve perfect run; prepare faster supported return only

Austin reported the latest run was perfect and requested roughly three times
faster laydown after contact. Archived 1,192 schema10 samples on verified
45c1a94/app0, fresh disarmed/healthy pre/post checks. Supported descent5.910s,
total8.508s, finaltilt-1.783 andForwarderrors-.027/+.009rad. Fast v2 stand-up
needed no early recovery. Reverted unflashed fast-v3 runtime as6106e77; preserve
its historical evidence, not its gains in this lowering-only release.

V12 raises fast supported target .6→1.8rad/s andcap.75→2.25, preserves600msblend,
smooths rate response to50degrees/s, tightens fast targetlead to.06rad and pauses
on either unloaded arm above15degrees. Floor taper moves35–15→20–5degrees.
Normal mode, precontact/catch, global65/wheel/time/support guards, finaldwell,
retraction and successful stand-up remain unchanged. Schema12 recordsv12lower
andv2standup; reserved11 remains readable with its historical unflashed policy.

Naive tripling and easing alone introduced simulated support losses. The added
proactive load pause preserves all258completions in329cases and rejects all71
baseline failures. One injectedcontactloss abort reason changes supportloss→
globalmotionlimit. Normal results identical; five normal recorded command
replays identical, fast divergence only aftersupport. Median modeled supported
speed ratio1.996, range1.169–2.648; nominal4.44→2.30s. Physical ratio unmeasured.
Peer fast-tip owner independently reviewed source/results with no blocker.
Continuous model peaks above65 on some successful cases are unchanged initial
impact peaks, before the modified phase; sampledguard is not a continuous bound.

Next: sourcefreeze, singlecombinedvalidation, configuredpackage and authorized
OTA under deviceowner. No motion/settings writes or deployment performed yet.

## 2026-09-20 — Deploy lowering v12 with successful stand-up preserved

Frozen source 09d2e01 passed 12 native suites, 44 Python tests, configured pinned
ESP32 build, radio C++/Lua and dashboard checks. Private package stays in
balance-lower/artifacts/lowering-v12-fast/candidate. Image 1,215,936 bytes,
SHA256 702508c46121c3a01bbd49fad6655ae2888b23950a5772c6793cb43acd60f4e9.
No firmware rebuild after freeze; documentation-only updates need none.

Authorized OTA completed HTTP 200 in 438.820 s with transmitter linked, disarmed
powered motors and no pacing sleeps. Receiver writes totaled 5.215 s, largest
receive gap 13.487 s, response wait 33.047 s. Installed app1 ESP digest
3b5c5f5601ffdac322e0c9bd6ddf06934821f71d9b90f6549c12a172844772f3 verified
at 2026-09-21T02:00:40.530835+00:00. Postflight: fresh idle/disarmed, all six
powered motors healthy/disabled, fresh IMU, released maintenance and returned
RC link (0 ms age). Wi-Fi channel 1/-56 dBm. Latest 1,192-row CSV/wire remained
byte-identical before/after. No autonomous motion, settings or filesystem write.

[Release evidence](../../evidence/lowering-v12-integration/README.md) retains
manifest, checks summary, timings, identities and preservation. Preceding
45c1a94 recovery package stays in the OTA task's checkout; no private package
or secret header copied between worktrees. Other tasks remain independent.
This lowering-only update is installed and ready for Austin's CH6 HIGH / CH11
manual trial. Physical speed/robustness still require the resulting telemetry.

## 2026-09-20 — Diagnose v12 contact deadlock and prepare narrow v13 correction

Archived Austin's failed run on verified 09d2e01/app1: 1,140 schema-12 samples,
lower_descent_timeout. Fresh powered healthy/disarmed checks and archive passed.
At confirmed support (19.790 s/82.825 degrees), targets freeze at 2.071/5.370 rad
until timeout at22.790 s. No Descending sample has both torque magnitudes>=0.2;
132/150 samples satisfy the existing normal rate bounds. The v12 load veto is
the immediate stall cause; static holding torque is not a contact measurement.

V13 replaces the zero target advance under weak torque with the proven normal
.24-rad/s return while forward rate<=12; the existing+4 backward pause remains.
Fast falling with weak support still pauses. Only confirmed fast Descending
changes; stand-up, normal mode, precontact/catch, bounds/timeouts and landing stay.
Native tests cover the physical stall and weak load at full pending target lead.
Schema13 identifies the policy while preserving historical exports.

All329 normal model results and six normal command replays remain identical;
fast retains258 completions and identical71 rejection outcomes. New feedback-only
post-support torque stresses complete40/40 versus21/40 on v12. Failed recorded
replay moves the target20 ms after support; fixed sensors still yield timeout,
so no physical completion inferred. Replay harness was corrected to extend one
stationary frame across the strict internal timeout and distinguish generated
.06-rad lead from held-target error. The latter reaches.061 when measured arm
moves1mrad during a backward pause; this is recorded rather than hidden.

Peer independently confirms trace diagnosis, runtime scope and held-target
explanation. Next: freeze/check/build once, preserve the run and deploy corrective
v13 through existing authorized OTA scope. No autonomous motion/settings writes.

## 2026-09-20 — Deploy verified v13 contact-stop correction

Frozen source7c32bd7 passed12 native suites,44 Python tests, configured pinned
ESP32 build, radio C++/Lua and dashboard checks once. Independent peer source,
trace and model review passed. All329 normal cases and six normal replays remain
identical; fast258/329 completion classification and fault reasons unchanged;
40/40 new low-torque stresses complete versus21/40 onv12. Physical retry pending.

Authorized OTA installs app0, source7c32bd7a6e48e641345a7fbc067a11d46a0643bf,
ESP digest2cba5a84594a703bb377b6697791fca4da790e8b6c8d4fe5ddf0f2181461cdae,
verified2026-09-21T02:22:24.302135+00:00. Frozenapplication1,216,640bytes,
SHA2563f6346766a7f8e2e1a3856e03e9b8daa884c22682ea7b8ffe92a4e29a0e6a7df.
HTTP200 in431.250s with transmitter linked; receiver writes5.699s,
maxreceivegap9.669s,responsewait50.121s,no host pacing sleeps. No rebuild after
freeze and no autonomous motion/settings/filesystem write.

Postflight exactimage/slot, allsix powered motors healthy/disabled, freshIMU,
IDLE/disarmed, savingfinished, releasedmaintenance andRCreturn passed. Saved
1,140-row failedrun CSV/wire byte-identical. [Release record](../../evidence/lowering-v13-integration/README.md).
Private package stays inbalance-lower/artifacts/lowering-v13-contact/candidate;
lastphysicallysuccessful45c1a94 recovery stays inOTA task checkout. No private
copies acrossworktrees. Docs identify currentpackage/procedure and diagnosis.

User can retry CH6 HIGH/CH11, then disarm and archive. Source correction is
installed; physical maneuver success remains for the operator to establish.
