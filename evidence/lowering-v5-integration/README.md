# Installed lowering v5 + fast startup fix

Installed and verified on September 20, 2026 at 20:35:49 UTC from the integration
owner's `worktrees/balance-lower`, branch `codex/balance-lower`.
[Motion behavior](../../docs/archive/2026-09/BALANCE_LOWER_V5_2026-09.md),
[physical v4 evidence](../balance-lower/trial-v4-20260920/README.md),
[paired model and limits](../balance-lower/forward-preparation-v5/README.md),
[source preservation](../balance-lower/forward-preparation-v5/source-preservation.json).

| Installed application | Value |
| --- | --- |
| Source | `17c499c06b57d8e6d82ffaee3fbf289b722aa8b6` |
| Running slot | app0 |
| Application bytes | 1,206,320 |
| Whole-file SHA-256 | `44a3cd6c1c1e71f845f1ba8ae9a26c9ebc3f44bc9c6d8c6a20a8cb2aa2a297d7` |
| ESP image digest | `8373d0b5be1ba254b5b9e831849e122989f66523c0ea35cf7dfcd1795a2ed837` |
| Private frozen package | `worktrees/balance-lower/artifacts/lowering-v5-final/candidate/` from project root |
| Saved capture format | Schema 5, 240-byte samples, features 65535 |

The [manifest](manifest.json) records frozen preparation; the
[deployment record](deployment.json) establishes installation. Normal OTA
transferred the application in 187.997 seconds, HTTP 200. The transmitter was
linked before and after the update, with both arming switches LOW. All six
powered motors returned online with no errors and remained disabled. Final
telemetry reported fresh IMU, both groups disarmed, maintenance released and
fresh voltage/current. The short post-maintenance timing window recorded no
intervals above 7.5 ms; this is not a powered motion test.

The 1,237-row saved v4 log is unchanged across OTA, including its original schema
and metadata: CSV SHA-256
`083d7d6b8d4926de8b5e7cdecc4886531ab5b569fb6206a2b40c72693c7d8741`, wire SHA-256
`2fe0a88f4beb3ebd7adf7eed40c9aaed03811762c867561794a7ad54c4bf91a3`.

## Integration and verification

Fast startup owner `9771bee` is integrated as `3f3bc37`. Ground/standing driving
policies remain unchanged; the setpoint cap is active only in CH11 preparation.
The other task's owned source files match its checkpoint, with combined startup
and lowering changes in the controller. No other checkout was edited.

[Combined checks](checks.txt): 11 native executables, 37 Python tests,
syntax/whitespace and pinned configured build passed. [Dashboard checks](dashboard-checks.txt)
passed. The final publication patch passed its [targeted build](publication-build.txt);
the policy/model arithmetic was unchanged, so those checks were reused.
The 329-case model screen has 253 completions versus 261 for the paired v4
baseline, with 43 improvements and 51 regressions. All 11 injected failures
reject completion; all 72 trial-informed contact cases complete, and delayed
contact improves from 8 to 15 of 24. Most regressions use the widest unmeasured
pivot geometry. This is an experimental operator-test build, with no physical
v5 acceptance yet; see the linked model record for geometry/contact limits.

## Attempts retained

1. Intermediate source `6e8c486` installed successfully into app1 in 63.65 seconds;
   [initial manifest](initial-manifest.json) and [deployment](initial-deployment.json).
   Before handoff, review found that an uncapped target was briefly published
   before a second volatile write applied the cap. Source `17c499c` publishes only
   the final capped value, preventing the faster balance task from reading the
   intermediate backward target. Both builds have the same displayed build time;
   the ESP digest, rather than that timestamp, distinguishes them.
2. [Interrupted backup](interrupted-backup.json): saved-run retrieval timed out
   before any upload; a subsequent device snapshot showed a power-on reset.
3. [Partial transfer](partial-transfer.json): 148,480 bytes sent before a broken
   connection at 84.074 seconds, with approximately −83 dBm Wi-Fi. Verification
   failed; a later snapshot still identified the intermediate app1 image after a
   power-on reset. No success was inferred from the incomplete transfer.
4. The operator repositioned the robot and confirmed steady power. Fresh
   eligibility and the old running digest were checked before the final retry.
   Wi-Fi recovered to approximately −54 dBm; the final updater verified the new
   app0 digest, powered disarmed health and byte-identical saved log. Signal and
   power changed together, so the evidence does not isolate one failure cause.

## Recovery and next trial

Keep the exact prior v4 package at
`worktrees/balance-lower/artifacts/lowering-v4-fast/candidate/`, source `e55cecb`,
ESP digest `62e38da80f1a6e61bb8ed1162c5bf4f83b280d0001d82469f500131dc96f7f42`.
Use the [normal application-only updater](../../docs/WIFI_OTA.md) after archiving
any newer log. Do not rebuild recovery bytes or upload the filesystem. Automatic
boot rollback is not enabled.

No autonomous motion, settings/calibration write, filesystem upload or secret
copying occurred. Next is a supervised CH11 lowering trial using CH6 LOW for
standing, followed separately by the fast startup trial. Download the log after
disarming and before another run replaces it.
