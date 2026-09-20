# Installed v10 — CH6 fast laydown

Source `7917543beedaf4902d1661461f18380959ad100a`, verified in app1 at 2026-09-20T22:51:48.370997+00:00.
[Manifest](manifest.json), [deployment](deployment.json),
[combined checks](combined-checks.txt), [source scope](source-preservation.json).
Application 1,212,352 bytes; whole-file SHA-256
`cf873cee0c52614dd179924c41c2466cc39ba1704fc5b1f449212bc25cb18234`; ESP digest
`42ed3918122860f6902ae0a1c8915ab771dd353b403de1de6d40fcf2d3cad4fe`. Private package:
`worktrees/balance-lower/artifacts/lowering-v10-fast/candidate/`.
The manifest records preparation; deployment establishes installation.

Austin confirmed [v9 success](../balance-lower/trial-v9-success-20260920/README.md).
Its 1,462 samples end lower_complete: lowering takes 12.009 s, supported return
9.900 s, final retraction only 0.139 s. Final tilt -2.368 degrees and Forward
errors -0.025/+0.005 rad. Fast standing succeeded again.

CH6 HIGH now latches fast lowering at the accepted CH11 request, independently
of stand-up selection. LOW/center retain normal v9. Only supported return
changes: 600 ms blend toward target speed 0.60 rad/s, motor cap 0.75 rad/s and
descent-rate pause 20 degrees/s. Speed tapers to normal from tilt 35 to 15 degrees.
Normal values remain 0.24/0.30/12. Preparation/catch, backward pause, support-loss,
lead, global motion, progress/time guards and final flat/retraction remain.
Fast standing, ordinary driving, OTA and CRSF production are unchanged; other
agent checkouts were not edited. Schema10 retains 240-byte samples and adds
independent fast-lower flag 4096; previous exports remain byte-identical.

12 native programs, 38 Python tests, syntax/whitespace and pinned configured
build pass once after source freeze. Existing radio/dashboard/pinned TCP
acceptance reused for unchanged code; actual OTA lifecycle/helper tests are
included in the combined run. [Model/replay](../balance-lower/fast-return-v10/README.md)
retains all 329 outcomes (258 completions), including 72/72 contact cases,
15/24 delayed-contact cases and rejection of 11 injected faults. All common
fast successes are quicker, median 2.04 s, nominal 10.14 to 8.08 s. Normal model
results and three recorded command replays exactly match v9. These are
approximate-model/regression checks, not physical acceptance of fast laydown.

Application-only OTA took 90.96 s. Exact image/slot, fresh IMU, six powered
healthy disabled motors, disarmed groups and released maintenance verified.
The successful 1,462-row schema9 CSV/wire are byte-identical before and after.
No autonomous motion, settings/calibration writes or filesystem upload.
Known successful v9 recovery is retained at
`worktrees/balance-lower/artifacts/lowering-v9-catch/candidate/`, source b9763c2,
ESP digest 361386bf17396fa11b7086ef272e813b49ee7a195c4d94bc1e38d19596ce98e8.
V7 and earlier packages remain. [Single-command OTA guide](../../docs/WIFI_OTA.md).
Next: operator CH6 HIGH / CH11 lowering trial, disarm and archive before another
run. The fast physical result remains pending.
