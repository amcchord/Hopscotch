# Repository consolidation — 2026-09-21

## Result and scope

Consolidates every development branch into `main`, preserving Git ancestry
without rebasing or force-pushing. The source integration checkpoint is
`f5fb997`. Checkpoint `326046f` preserves 16 previously uncommitted repeat-trial
and analysis files. ESP32 `src/`, `data/` and `platformio.ini` remain byte-identical
to that latest balance-lower baseline. Radio runtime/tests incorporate the
previously separate Lua v3.1 fixes from `605c66d` / `05e25ab`.

All branch tips below are ancestors of the combined history. Duplicate
cherry-picked features were resolved against the later implementation, keeping
v13, schema 13, OTA diagnostics, uploader timeout fixes and deferred-experiment
status. New release evidence and both sides' unique journal entries were kept.

## Retained branch tips

Each tip is retained by `archive/2026-09-21/<branch-name>` on GitHub; merged
branch refs are retired after `main` and tags are verified remotely.

| Former branch | Archived commit |
| --- | --- |
| `agent/balance-telemetry-sync` | `e8b1280d241f` |
| `codex/balance-lower` | `326046f4f2da` |
| `codex/balance-review-ready` | `112e7e7c2dbf` |
| `codex/drive-braking` | `e9e1c65996c5` |
| `codex/fast-tip-up` | `89f8a8d3f4d4` |
| `codex/ground-drive` | `1337c1050319` |
| `codex/ota-reliability` | `50dfdf2bc6e9` |
| `codex/ota-throughput` | `960decdc6ea4` |
| `codex/radio-telemetry` | `05e25ab5132b` |
| `codex/wifi-ota` | `ef4960a79cbf` |

## Context and files

- Root integration checkout replaces seven inactive worktrees. About 5 GB of
  duplicate dependency/build-cache storage was reclaimed; about 2.2 GB remains.
- Preserved and SHA-256-verified 2,400 ignored private/support files,
  600,953,830 bytes, in `artifacts/retired-worktrees/`. Local relocation manifests
  map original paths to immutable contents. Root recovery packages/backups remain.
- Read AGENTS, WORKBOOK, CURRENT and the newest journal entry: about 1,200 words
  combined. Current-state text is about 310 words. README is a short landing page.
- Moved 28 dated reports plus old project/radio journals into `docs/archive/`;
  moved the expanded hardware/architecture README into `docs/FIRMWARE.md`.
  Repaired Markdown links and current operating commands after relocation.
- `.ignore` excludes historical reports, raw captures, experiment results and
  third-party reference material from default searches. About 119 files remain
  in default `rg --files`, rather than more than 1,100 tracked project files.
  Raw CSV/wire/serial captures and evidence remain in Git at stable paths.
- `.gitignore` now shares worktree/private-binary/local-environment protections
  across clones. Configured firmware and recovery backups remain private.

## Verification

`./scripts/check_project.sh` passed: 12 native suites, 44 Python tests,
Python/shell syntax, whitespace, pinned ESP32 build, native radio transport,
Lua display/logging, dashboard syntax/eight SHA vectors and actual-source OTA
transport harness. The first combined invocation exposed a non-executable radio
script; its executable bit was corrected and the full command passed.
The additional shell loop checks every script, not only the first argument.

Build application sections: 1,216,269 bytes; static RAM: 53,600 bytes.
Existing Arduino macro-redefinition warnings remain. The retained Lua 5.3.6
32-bit runtime is separately checked. These are host/build checks, not a new
hardware acceptance test. Source and raw-capture preservation, relative document
links, relocated v13/45c1a94 image manifests and outgoing credentials are checked
before publication. Detailed output stays local in `output/project-cleanup/`.

No robot/handset request, flash, restart, settings write, calibration or motion
occurred. The last archived installed identity and unresolved fast stand-up /
lowering failures are in [CURRENT](../../progress/CURRENT.md). The next task
should begin there, on a fresh branch from `main`.

## Existing credential history

The scan found the configured Wi-Fi password in a historical `docs/PROJECT.md`
blob introduced by `e8b1280` and removed by `aa9ae13`. That history was already
reachable from eight GitHub development branches before this cleanup. Current
tracked files and newly published blobs contain no configured credential matches.
This consolidation preserves history; it does not claim to erase that exposure.
Rotate the Wi-Fi credential through a separately coordinated network/device
change. Purging published Git history would require a separately planned rewrite
of references and coordination with other clones. No credential value is recorded
here, and no password or device configuration was changed in this task.
