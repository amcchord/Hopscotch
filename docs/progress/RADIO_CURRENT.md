# Radio telemetry candidate — September 19, 2026

- **Objective:** audit and back up the GX12 configuration; build a robot-state
  Lua display and extensible firmware telemetry without changing mappings.
- **Work area:** `worktrees/radio-telemetry`, branch `codex/radio-telemetry`.
  Implementation commit `899d4a3aa65a1cc5a7dfc424cf7c003e006cac64`, based on
  latest saved balance-v4 work `782bcef`. No integration into the control
  checkout, remote push, PR, radio write, robot command, or flash by this task.
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
- **Next:** obtain authorization for installation, inspect actual ELRS
  firmware/settings, integrate with the current control checkout, and conduct
  a disarmed bench test. Experimental CRSF type 0x7E is project-private and
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
