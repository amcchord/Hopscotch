# Project journal

Older session records are preserved in the
[September archive](../archive/2026-09/JOURNAL.md). Read only the newest entry
for startup context; detailed investigations remain linked from CURRENT.

## 2026-09-21 — Consolidate branches and reduce startup context

Integrated all ten development branch histories on `codex/project-cleanup`,
starting from the latest balance-lower state. Preserved 16 previously uncommitted
trial/analysis files in `326046f`. Conflicting cherry-picked work resolves to the
later v13 implementation; ESP32 source, dashboard and build configuration remain
unchanged from that baseline. Added the radio branch's Lua v3.1 fixes and tests.

Added AGENTS, WORKBOOK, a concise CURRENT, decisions, a docs index and one complete
offline check command. Moved dated reports and previous journals into the archive;
updated Markdown links and live operating commands. `.ignore` keeps bulky
evidence/raw telemetry/history out of default searches while preserving Git
tracking and stable replay paths.

Retired seven clean, integrated worktrees after moving and SHA-256-verifying
2,400 local private/support files (600,953,830 bytes). Local
`artifacts/README.md` and relocation manifests preserve package locations.
Removed duplicate dependencies/build caches; workspace storage fell from about
7 GB to 2.2 GB. Kept root build environments and all frozen recovery packages.

Validation and GitHub publication are recorded in the
[consolidation report](../archive/2026-09/CONSOLIDATION.md).
No robot, handset, settings, firmware deployment or motion operations occurred.
Next: follow CURRENT's bounded arm/wheel recovery investigation from `main`.
