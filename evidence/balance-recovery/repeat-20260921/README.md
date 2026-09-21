# Repeated stand-up and slow comparison — 2026-09-21 UTC

Preserved during project consolidation from uncommitted investigation files.
These captures and the analysis are a checkpoint, not an accepted controller
change or a release candidate. Installed source remains `7c32bd7` (v13).

- [Comparison](comparison.json) and [plot](comparison.png): the fast repeat
  has 321 samples / 6.404 seconds and ends `bailout_angle_error`. The slow run
  has 1,372 samples / 27.424 seconds; stand-up settles, but its subsequent
  lowering ends `lower_wrong_direction`. Do not describe that as full-run success.
- [Fast archive](../../balance-lower/standup-repeat-20260921T030048Z/archive.json)
  and [slow archive](../../balance-lower/slow-comparison-20260921T030552Z/archive.json)
  retain download identities and separate pre/post health snapshots.
- [Analysis source](analyze.py), [source hashes](reset-source-hashes.json), and
  [restart observations](restart-observation.json) preserve unfinished work.

The analysis contains model/causality limitations; verify them before changing
recovery or trim. Consolidation performed no robot or settings operations.
