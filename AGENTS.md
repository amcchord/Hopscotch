# Hopscotch working agreement

This root checkout is the integration checkout. Read the workspace agreement
at `../AGENTS.md`, then `WORKBOOK.md`, `docs/progress/CURRENT.md`, and only the
newest entry in `docs/progress/JOURNAL.md`. Follow task-specific links from there.
Current state identifies the last verified robot image; Git HEAD is not proof
of what is installed.

## Keep context small

- Start searches in `src/`, `scripts/`, `tests/`, `data/`, or `radio/`.
- `.ignore` excludes retained experiments, raw telemetry, reference projects,
  and historical reports from default ripgrep searches. They are still tracked.
  Use `rg --no-ignore <pattern> evidence/<specific-case>` when evidence is needed.
- Do not recursively read the journal, tuning history, evidence, CSV/wire/serial
  captures, generated JSON, build output, private artifacts, or worktrees.
  Read a selected README/summary first, then bounded sections of relevant data.
- Keep CURRENT to present facts, uncertainties and the next action (about 400
  words maximum). Put session outcomes in the journal, durable choices in
  DECISIONS, and detailed evidence beside the specific experiment.
- Do not duplicate installed identities or active ownership in multiple guides.

## Work and verification

- Inspect status, remotes and worktrees before editing. Preserve unrelated work.
- Use `codex/<task>` branches; any concurrent checkout belongs under `worktrees/`.
  After integration, preserve unique private files and remove the clean worktree.
- Use focused checks while developing. Run `./scripts/check_project.sh` once
  after integration and scope freeze; it performs host tests and a firmware build.
  Test output belongs in ignored `output/`, with a concise result in the journal.
- Keep pinned PlatformIO dependencies and control/network isolation intact.
  Do not refactor timing-sensitive motion code as incidental housekeeping.
- Preserve raw telemetry byte-for-byte, including wire/serial newlines.
  Models screen candidates; they do not establish physical success.

## Robot and private data

Read `docs/WIFI_OTA.md` before device operations and `docs/BALANCE_TESTING.md`
before physical trials. Device writes, firmware uploads, resets, calibration,
trim changes and motion require authorization for that task. Never run `uploadfs`:
LittleFS holds settings, calibration and the last run. Save that run before it
is replaced. There is no automatic boot rollback.

`src/network_secrets.h`, configured firmware/ELF files, flash backups and handset
backups stay local and ignored. Do not print credentials or publish binaries.
`artifacts/README.md` maps retained private recovery packages; archived manifests
may contain historical checkout paths. Validate bytes and identity before reuse.
