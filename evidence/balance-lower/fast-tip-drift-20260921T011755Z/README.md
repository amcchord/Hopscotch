# Successful fast laydown and stand-up drift report

Austin reported another successful test, with substantial stand-up drift.
Archived at 2026-09-21T01:17:55Z (September 20 EDT) after fresh disarmed IDLE,
finished saving and powered healthy disabled motors were verified. Running
source 45c1a94/app0 has ESP digest
`a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61`.
[Preflight](preflight.json), [archive and hashes](archive.json),
[post-export idle/health](post-archive.json), [lowering summary](summary.json).

[CSV](../../../telemetry_logs/bal_20260921T011755Z_fast_tip_success_drift_wifi.csv)
and [original wire](../../../telemetry_logs/bal_20260921T011755Z_fast_tip_success_drift_wifi.wire)
contain 1,325 samples / 26.660 seconds, schema 10, ending lower_complete.
Fast stand-up flag 128 is set; the independently latched fast-lowering flag 4096
is set throughout lowering. No telemetry or private files were copied between
worktrees; the fast-tip-up task reads the capture here for stand-up analysis.

## First successful fast laydown

CH11 lowering starts 18.421 s, support confirms 19.721 s, flat hold begins 25.891 s,
final retraction 26.496 s and completion 26.660 s. Total 8.239 s; supported return
6.170 s. The previous normal v9 recording took 12.009 s total / 9.900 s supported.
This run is 3.770 s (31.4%) shorter overall; the trials are not a controlled
comparison or a reliability estimate. Final tilt -1.814 degrees, rate -0.182
degrees/s, Forward errors -0.028/+0.006 rad, wheels +0.004/+0.047 rad/s.

## Remaining stand-up work

Austin reports substantial stand-up drift and suspects a run-to-run angle
change; that hypothesis is not established by this archive. The fast-tip-up
task owns comparison of capture, arm return and equilibrium across the saved
runs, and any justified source candidate. The earlier asymmetric wheel-response
[failure analysis](../../fast-tip-up/fall-20260921/README.md) remains relevant.
Balance-lower retains exclusive device/integration ownership, with no concurrent
motion/source/OTA change. The request here only archived the run and recorded
lowering timing. No autonomous motion, settings change or update was performed.
