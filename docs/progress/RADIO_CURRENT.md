# Radio telemetry — September 20, 2026

## Crash fix installed — Lua v3.1, September 20 at 14:05 EDT

Austin's GX12 photo shows `attempt to index a nil value (field 'table')`.
Reproduced on v3 at `event()` when the first fresh FM value arrives. EdgeTX
2.11 registers the `table` library only for color-LCD builds; the earlier host
tests incorrectly exposed the desktop library to this monochrome script.

- Removed all library dependencies on `table`: bounded four-entry event shift
  and CSV construction now use core Lua indexing/concatenation. Ordinary Lua
  tables are supported. Retained the page/freshness fixes and opt-in logger.
- Both existing test suites now load production code into a restricted GX12
  environment: no table/os/package/debug/coroutine, explicit supported APIs,
  read-only math/string libraries and EdgeTX-style I/O. Harness helpers cannot
  leak desktop libraries into the script. Old source failed before the fix;
  corrected display and 600-record logging/error suites pass.
- Additionally built official Lua 5.3.6 with 32-bit integers/floats, matching
  EdgeTX 2.11's vendored version/numeric types; all radio tests pass there.
  Accounted for binary32 versus binary64 rounding at the -0.45 display boundary
  in the host assertion. Native fixture/transport, CSV integrity, syntax and
  rendered-layout checks pass. This is not a full hardware VM/scheduler test.
- Installed `hop.lua` from source `605c66d`: 17,524 bytes, SHA-256
  `d2674ebd99d1609c62ff9de6cf657e739c6eaa4653f8fcf490b659a9cd7bc995`.
  Package/evidence: project-root `artifacts/radio-lua-v3.1-2026-09-20/`.
- Austin reattached NO NAME and authorized deployment. Verified the volume UUID,
  all six frozen-package manifest entries and source equality. Backed up v3,
  its bytecode and current configuration to project-root
  `artifacts/radio-lua-v3.1-2026-09-20/install-20260920T180506Z/` before replacing
  only `SCRIPTS/TELEMETRY/hop.lua` and removing generated `hop.luac`.
- Card readback matched the tested source after sync; Lua 5.3.6/32-bit syntax
  check passed directly from the card. All four MODELS/RADIO files and two
  unrelated telemetry files stayed byte-identical. Model00 still selects hop;
  LOGS exists and logging remains OFF by default. Safe eject succeeded.
- Firmware, mappings and other worktrees are untouched. Next: reboot the radio,
  open telemetry screen 2 and verify the display on the handset. Diagnostics
  page 6 should show LUA v3.1. Hardware boot/runtime verification remains pending.

References: [EdgeTX library availability](https://luadoc.edgetx.org/overview/version-libraries),
[2.11 library registration](https://github.com/EdgeTX/edgetx/blob/v2.11.0/radio/src/thirdparty/Lua/src/linit.c).

## Prior installation — Lua v3, September 19 at 23:58 EDT (crashes on GX12)

Austin reported residual blanking and repeated Basic page content, and asked
for an update prepared before reconnecting the radio. The candidate was prepared
in the radio worktree, then installed after Austin reattached the radio and
explicitly requested copying it to the SD card. No robot firmware was changed.

- Fixed the Basic fallback routing: overview, standard pose/power, unavailable
  motor details, observed events/run report, radio link and diagnostics each
  have their own content. Motor states are never inferred from FM text.
- Reproduced lost updates with EdgeTX 2.11's 160–320 ms freshness window and the
  old 200 ms sampler. Poll at 50 ms; hold individual values five seconds, mark
  cached readings with `*`, expire them independently. Robot arming/motion still
  becomes unknown after three seconds; explicit state/fault updates are immediate.
- Added opt-in diagnostics on page 6: long ENTER toggles 1 Hz CSV under LOGS,
  default OFF, 600 rows per script load, append/close each row, stop on I/O error.
  Captures ages, freshness, sample gaps, packet counts and status flags.
- Native radio tests, actual C++→Lua fixtures, six Basic pages, phase-aligned
  missed-update regression, expiry/recovery, unknown/schema/duplicate handling,
  logger lifecycle/error/limit tests, CSV parser and rendered layouts passed.
  Radio runtime/storage timing and the physical RF path remain unverified.
- Installed `hop.lua` from source `9c5b88d`: 17,299 bytes, SHA-256
  `db6ca509d52780358728b18b4a59186e862c4e1195aed939415d566269e1da7f`.
  Prepared package: project-root `artifacts/radio-lua-v3-2026-09-19/`.
- Verified the frozen package, source hash and NO NAME volume identity before
  replacing `SCRIPTS/TELEMETRY/hop.lua`. Backed up the prior Lua, bytecode and
  current configuration in `artifacts/radio-lua-v3-2026-09-19/install-20260920T035847Z/`
  at the project root. Removed old `hop.luac`, verified LOGS already exists and
  model00 still selects hop. Card syntax/readback passed after sync; all four
  MODELS/RADIO files and two unrelated telemetry files were unchanged. Safe
  eject succeeded. Logging remains OFF until enabled on Diagnostics.
- Worktree remains `codex/radio-telemetry`; no edits to other branches, firmware,
  model configuration or channel mappings. Root integration belongs to its
  current task owner. Next: reboot the radio, open telemetry screen 2, verify
  distinct pages and steady values. Page 6 shows LUA v3; hold ENTER there to
  opt into a diagnostic CSV if needed. Hardware runtime/RF checks remain pending.

## Previous installed Lua update — 22:31 EDT

Austin reported flickering readings and requested a Lua-only fix. Installed
three-second holds for each standard sensor and numeric robot reading; fresh
values replace the cache, invalid/stale values and unrelated packets cannot
renew it. Status shows HOLD after 1.5 seconds, UNKNOWN after three; fresh fault,
arming and IMU flags remain immediate. Existing loss alerts retain their timing.

- **Source:** `codex/radio-telemetry` worktree. Root checkout is owned by the
  Wi-Fi/OTA task and was not edited or merged into during this Lua fix.
- **Installed:** `SCRIPTS/TELEMETRY/hop.lua`, 11,724 bytes, SHA-256
  `49f7ea30dcafc6118c27ba626fa7d66d18a346fa70369d3b6ecfd30f62b0b7e7`.
  Removed the old radio-generated `hop.luac` so EdgeTX rebuilds it at reload.
- **Verified:** native encoder/transport and Lua regression tests, per-value
  expiry, alternating updates, invalid values, zero values, recovery, clock
  wrap, duplicate/detail rejection, immediate flags, syntax and rendered
  layouts. Readback matched source; all four MODELS/RADIO files were unchanged.
  Synced and safely ejected the card. Hardware screen check follows reboot.
- **Backup:** `artifacts/radio-lua-hold-2026-09-19/20260920T023100Z/` in the
  project root contains previous Lua/bytecode, installed Lua and installation
  hashes. Restore the prior Lua and remove generated bytecode to undo this fix.
- **Firmware:** no robot access or firmware changes during this fix. The separate
  Wi-Fi/OTA task has since installed combined firmware; consult its integration
  checkout's current state for that release. The earlier flash attempt in this
  task stopped before writing firmware. Its old OpenOCD full-read artifact
  failed image validation and must not be used as a restoration image.
- **Next:** reboot the radio and check telemetry screen 2. Integrate this Lua
  commit into the current firmware branch when its owner is ready.

## Original audit and installation record

- **Objective:** audit and back up the GX12 configuration; build a robot-state
  Lua display and extensible firmware telemetry without changing mappings.
- **Work area:** `worktrees/radio-telemetry`, branch `codex/radio-telemetry`.
  Implementation commit `899d4a3aa65a1cc5a7dfc424cf7c003e006cac64`, based on
  latest saved balance-v4 work `782bcef`. No integration into the control
  checkout, remote push, PR, robot command, or flash by this task. The Lua script
  and its telemetry-screen selection are now installed on the radio.
- **Backup:** `artifacts/radio-backup-2026-09-19/`: 1,072 files / 36,959,971
  bytes; verified per-file SHA-256. Current MODELS/RADIO files rechecked byte
  identical at completion. RF-module settings are not on the USB filesystem.
- **Result:** five receive-only Lua pages; truthful separate drive/arm state,
  motor feedback, filtered pose/error, power freshness, run-stop reason,
  limited event history, stale/unknown status and haptics. Versioned status,
  capability, motion ID, phase, label, and optional progress fields. No new
  command path. Control code, gains, channel settings, and mapping constants
  are unchanged relative to the balance-v4 base.
- **Audit / roadmap / protocol / install:** [RADIO_TELEMETRY.md](../RADIO_TELEMETRY.md).
- **Validation:** seven existing native executables, 25 Python tests, additional
  radio native/C++→Lua fixture suite, Lua behavior/syntax, rendered layout,
  whitespace/shell/Python syntax checks and PlatformIO build passed after rebase.
  Hardware RF transport and EdgeTX runtime/memory have not been tested.
- **Build:** flash usage 1,147,237 bytes; static RAM 50,888 bytes. Candidate
  application 1,147,600 bytes, SHA-256
  `b2985bce4a877e15c83c79743482b4fcf30c5592b7d014af31ea51af3ce97348`.
  Lua source SHA-256 `be2c995801804c7c354cc9d7cb7c5e12177ae60f9bf8b4231e924236aca93899`.
- **Reproducibility:** pinned dependencies to the versions in the control
  checkout. M5GFX registry no longer resolved 0.2.19, so pinned its official
  tag commit `53a7184601f3667b030ba141c58b87ce2acfaa2a`. Initial fresh dependency
  resolution was discarded; final validation uses the original versions.
- **Artifacts:** `artifacts/radio-telemetry-2026-09-19/` contains installable Lua
  zip, review preview, candidate application, report, validation log, hashes.
- **SD installed:** authorized by Austin, September 19 at 20:55 EDT. Verified
  `SCRIPTS/TELEMETRY/hop.lua` against the tested SHA-256 and passed syntax check
  from the card. Added only telemetry screen 2 / index 1, `SCRIPT hop`, to
  `MODELS/model00.yml`; every original model byte outside that insertion and
  all `RADIO/radio.yml` bytes remain unchanged. Previous Values screen retained.
  Synced, reread, and safely ejected the volume. Reboot and open telemetry screen 2.
- **Install backup:** `artifacts/radio-telemetry-2026-09-19/install-20260920T005518Z/`
  contains before/after model files, radio backup, and installation hashes.
- **Next:** verify the display on the rebooted radio. Existing robot firmware
  provides Basic telemetry; full structured state still requires integrating
  and separately installing the robot firmware candidate. Inspect actual ELRS
  firmware/settings and conduct a disarmed bench test. Experimental CRSF type 0x7E is project-private and
  needs hardware forwarding verification. Do not upload filesystem data or
  change the current mappings as part of installation.

## Session record

Read the Development agreement and project current state. Root checkout had
ongoing balance changes, so created an isolated worktree at `bc93348`. Copied
and hashed the radio storage, reviewed all model/radio YAML and backup
differences, traced robot telemetry and trigger logic, and checked official
GX12 / EdgeTX / ELRS / CRSF references. Built the Lua and firmware status
contract, tested sender/backpressure and the actual Lua decoder together,
rendered screen layouts, documented extension and rollback plans. The separate
balance task advanced to `782bcef`; rebased this candidate onto it and reran
combined validation. No hardware state was changed by this telemetry task.

## SD installation follow-up

Austin authorized SD installation, then requested a readiness check after an
interrupted turn. Inspection showed no script had yet been copied. Installed the
tested Lua source and added a spare telemetry screen using the verified EdgeTX
2.11 YAML schema. Byte-level comparison proves no original controls/settings
were altered. Source and installed script hashes match; card syntax check,
post-sync file readback, and safe eject succeeded. Robot firmware was not changed.
Rollback is remove the added screen and hop.lua, or restore the saved pre-install
model after checking for any subsequent user changes.
