# Installed lowering v6 + fast support release

Verified 2026-09-20T21:05:08.644323+00:00, source `a772ecc833c6100f76cdb67173bdaf4f4d99cd78`, running app1.
[Manifest](manifest.json), [deployment](deployment.json), [combined checks](checks.txt),
[source preservation](source-preservation.json).

Application: 1,207,616 bytes. Whole-file SHA-256
`be068a3c2e02ac4343d6c15795984577d2067cceebf0c069e97be9a91b511360`. ESP image digest
`fdf7af6a96d2b6e3c3c908301ae765fcfa4c9110df27f8d88693dd80e8aeb2c2`. Private frozen package:
`worktrees/balance-lower/artifacts/lowering-v6-fast/candidate/` from project root.
The frozen manifest records preparation; the deployment record proves installation.

The [lowering trial](../balance-lower/trial-v5-20260920/README.md) ended before
catch: a rear wheel exceeded 6 rad/s while upright control waited for the final
arm travel. [V6](../../docs/archive/2026-09/BALANCE_LOWER_V6_2026-09.md) hands off earlier with
measured forward departure and two moving, unloaded arms, retaining all limits.
The existing contact/continuous-return sequence remains responsible for the
flat/Forward finish. Recorded replay reaches handoff 39 ms before the old fault;
it stops at divergence rather than assuming the later physical outcome.

The [fast owner’s diagnosis and screen](../fast-tip-up/roll-away-review/README.md)
accompany checkpoint `f4d2bb7`, which is integrated as `882ba98`. The separate fast trial
captured quietly while supported; treating that angle as equilibrium applied a
permanent −2.86° correction before the arms released. Fast capture now preserves
stored trim and releases its temporary offset over the initial measured arm
return, with the existing base slew limit. Both rear wheels must be within
0.75 rad/s throughout capture. Trajectory, slow standing, return speed and shared
gains remain unchanged. New metadata is schema 6 with the same 240-byte samples;
schema 5 exports keep their literal original metadata.

## Verification and limits

11 native executables, 37 Python tests, syntax/whitespace and the pinned configured
build passed once. New checks exercise recorded lowering handoff, guarded early
release and continued supported arm return; fast checks cover recorded capture
offset, monotonic release, slow arithmetic and independent wheel capture gates.
Unchanged radio, dashboard and OTA transport acceptance was reused.

The [329-case lowering screen](../balance-lower/early-handoff-v6/README.md) has
253 → 255 completions, 9 improvements and 7 regressions. All 72 trial-informed
contact cases complete and all 11 injected failures reject completion. The fast
release model retains 24/45 cases without a modeled fall, with 10 improvements and 10
regressions, especially when saved trim overestimates actual equilibrium.
These models omit or approximate contact/geometry and cannot establish stable
hardware behavior. Neither revised motion has physical acceptance yet.

## Device outcome and recovery

Normal application-only OTA completed in 342.569 seconds with the transmitter
linked before and after; no concurrent telemetry stress monitoring was used. Exact image and slot
matched; all six powered motors returned online, error-free and disabled. IMU was
fresh, both motor groups disarmed and maintenance released. The saved fast trial
retained 236 rows and identical exports: CSV SHA-256
`aefd28b02ea7082f3578527589c7cf4238339a774e7703d576416ef9e434c658`, wire SHA-256
`159742998bbbab37f20793fd451dd8e269b8d0dfbbaa45906fdf2372d85564a8`.

No autonomous motion, settings/calibration change, filesystem upload, secret
copying or other task’s checkout edits occurred. Owner source preservation and
combined checks are linked above. Keep exact previous v5 recovery bytes at
`worktrees/balance-lower/artifacts/lowering-v5-final/candidate/`, source `17c499c`,
ESP digest `8373d0b5be1ba254b5b9e831849e122989f66523c0ea35cf7dfcd1795a2ed837`.
Use the [same verified OTA procedure](../../docs/WIFI_OTA.md) after archiving newer
logs; automatic boot rollback is not enabled.

Next: separate operator lowering and fast-standing trials, archiving the latest
log after each. Use CH6 LOW when evaluating CH11 lowering.
