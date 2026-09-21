# Durable decisions

- **One integration baseline.** Root and GitHub `main` contain completed work.
  Short-lived branches/worktrees isolate changes; merge and retire them after
  verification. September 2026 branch tips are retained as archive tags.
- **A short entry point.** AGENTS → WORKBOOK → CURRENT → newest journal entry.
  Detailed reports and raw evidence are opt-in. Keep current installation and
  next action in CURRENT rather than copying them into every guide.
- **Retain experimental evidence at stable paths.** Tests and replay scripts
  depend on telemetry/evidence filenames. Keep raw captures byte-exact and in
  Git; use `.ignore` to exclude bulk data from default searches. Historical
  reports live in `docs/archive/`. Reconsider external data storage only if clone
  size becomes a measured problem; this cleanup does not rewrite Git history.
- **Preserve the control boundary.** Balance/control own motion on core 1;
  networking formats bounded snapshots on core 0. Flash/file maintenance needs
  control-owned disarmed permission. Core separation alone does not make flash
  safe during motion. [Design](../WIFI_OTA.md#control-isolation).
- **Separate simulated, tested and installed.** Models are uncertain screens.
  Freeze and validate combined firmware once, then separately authorize device
  changes and record exact image/slot and preserved telemetry. A successful
  host test or one physical run is not a reliability claim.
- **Private recovery remains local.** Configured binaries contain credentials;
  never publish them. Preserve frozen packages and full-flash/radio backups
  before retiring checkouts. Use the local artifact index for relocated paths.
