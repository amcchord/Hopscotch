# Installed v8 — faster supported arm return

Verified 2026-09-20T22:12:58.795503+00:00: source `47eb19a2ce57f51eeea368c4c6469edd1cc51a96`, running app1.
[Manifest](manifest.json), [deployment](deployment.json), [combined checks](combined-checks.txt),
[final build](final-build.txt), [source preservation](source-preservation.json).
Application 1,211,120 bytes, whole-file SHA-256
`5dfd2201ac0a0d986325dfa913980467cdba47c7d53db7797db24149b188cb80`, ESP digest
`6b0b637e58e45c5f0b5c54521acdd81ab8d3f9ca56cd7d54311e56b8cbcc50d1`. Private package:
`worktrees/balance-lower/artifacts/lowering-v8-return/candidate/`.
The frozen manifest records preparation; the deployment proves installation.

## Physical baseline and change

Austin confirmed successful fast standing and lowering on c442e12. The
[saved 1,759-row run](../balance-lower/trial-v7-success-20260920/README.md) ends
lower_complete: tilt −2.818°, rate −0.455°/s and Forward errors −0.027/+0.007 rad.
Lowering takes 14.109 s, of which supported descent occupies 11.370 s.
Maximum supported tracking errors are 0.021/0.011 rad.

The only motion change is supported target speed 0.16 → 0.24 rad/s, a 50% increase.
The 0.30-rad/s motor cap, body-rate pause outside −12..+4°/s, catch/contact policy,
wheel braking, final retraction, flat dwell and every global guard remain.
Final retraction already takes 0.740 s and approaches the ground-motion rate
guard, so its speed remains unchanged. Fast standing, ordinary driving and OTA
production are byte-identical to the successful baseline. New schema8 metadata
retains 240-byte samples and preserves prior schema7 exports.

## Checks and limits

12 native executables, 38 Python tests, syntax/whitespace and configured pinned
build passed. A final build after metadata indentation cleanup also passed.
Existing radio/dashboard/pinned TCP checks were reused because all corresponding
source is unchanged; consolidated tests include actual OTA lifecycle/helper code.
The [same 329-case model](../balance-lower/faster-return-v8/README.md) has 258
completions before and after, no changed outcomes. Every common success is
faster: median saving 2.72 s, nominal 13.08 → 10.14 s. All 72 contact cases
complete, 15/24 delayed-contact cases complete and all 11 injected failures reject.
Unmeasured plant/contact assumptions remain; v8 needs a physical trial.

## Deployment and recovery

Application-only upload took 475.235 s. Exact image/slot, all six powered
motors online/error-free/disabled, fresh IMU, disarmed groups and released
maintenance verified. The successful schema7 CSV and wire retained all 1,759
rows byte-for-byte. No autonomous motion or settings/filesystem writes occurred.

The [single in-flight snapshot](during-ota.json) confirmed active OTA, maintenance,
disabled motors and cleared RC/link input. The transmitter was linked before
and after reboot. This is consistent with UART suspension; the physical
progress screen was not independently observed.
The observed transfer time is not a controlled throughput comparison.

Exact successful v7 recovery remains at
`worktrees/balance-lower/artifacts/lowering-v7-ota/candidate/`, source `c442e12`,
ESP digest `5affbef55fc259591b2737cf1f17226b5878f5ac2ce847c7694919d755c975a4`.
Use the [single-command OTA procedure](../../docs/WIFI_OTA.md); preserve newer
logs before downgrade. No automatic boot rollback. No other agent's checkout
was edited. Next: one operator stand/lower trial, then disarm and archive.
