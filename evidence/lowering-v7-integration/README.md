# Installed lowering v7 + OTA progress, successful fast v2 preserved

Verified 2026-09-20T21:36:50.532600+00:00, source `c442e1271eaf9f38cfbeab27c2e435fe6b8e3eb3`, running app0.
[Manifest](manifest.json), [deployment](deployment.json), [combined checks](checks.txt),
[radio](radio-checks.txt), [dashboard](dashboard-checks.txt),
[OTA transport](ota-transport-checks.txt), [source preservation](source-preservation.json).

Application: 1,210,736 bytes. Whole-file SHA-256
`aaef0c5f2c768ff36cd51f3d41ec7ff08a55bb682c2995be958c55901635b59a`. ESP digest
`5affbef55fc259591b2737cf1f17226b5878f5ac2ce847c7694919d755c975a4`. Private frozen bytes remain at
`worktrees/balance-lower/artifacts/lowering-v7-ota/candidate/` from project root.
The manifest records preparation; the deployment record proves installation.

## Changes and evidence

The [1,519-row v6 run](../balance-lower/trial-v6-20260920/README.md) shows a successful
fast stand-up followed by a 12.525-second lower stop gate and rocking contact
that never qualified support. [V7](../../docs/archive/2026-09/BALANCE_LOWER_V7_2026-09.md) completes
the existing drive-reference ramps, then hands explicit CH11 requests back to
stationary PD through the existing slew. The independent 500 ms calm gate remains.
After both arms contact, existing bounded wheel braking begins. Qualification
allows per-arm loads within 60 ms and body rate −12..+12°/s, requiring both an
80 ms dwell and real elapsed 80 ms after both contacts. Single or delayed
one-off impacts cannot qualify. Supported descent retains continuous arm return
and measured flat/Forward completion; global guards remain unchanged.

Lowering source `c2dd50c`; successful fast production `f4d2bb7` remains unchanged.
Fast-success evidence `1a2e30b` is integrated as `93f997f`. OTA progress source
`f58f0d9` is integrated as `660634a`, preserving transport2 diagnostics/timeouts,
cached start-status JSON, cooperative export and fast feedback scheduling.
It suspends CRSF and sends motor stops on the control owner before granting
OTA flash access, presents queued progress, and resets RC freshness/rearming
after abort. The installation itself runs under old firmware; this release
does not establish the new screen/UART behavior or throughput on hardware.

## Validation and limits

12 native executables, 38 Python tests, syntax/whitespace, configured pinned
build, radio C++/Lua, dashboard and actual pinned TCP/OTA transport checks pass.
The old transport harness required only signature/progress-stub adaptation;
the separate lifecycle test runs actual upload/progress/interlock code.
No production source changed after the consolidated build. The existing
ARDUINO_EVENT_RUNNING_CORE redefinition warning remains. No new network
load/stall test was run on the powered robot.

The [paired lowering screen](../balance-lower/supported-return-v7/README.md) improves
255 → 258/329, three improvements/no regressions; all 72 contact cases complete,
15/24 delayed-contact cases complete, and all 11 injected failures reject.
It omits standing-drive handoff and unidentified physical geometry/contact.
Fixed recorded-input replay qualifies support at 30.110 s but faults on the old
trace's later support loss. Commands diverge from that trace at 30.030 s, so this
is not a prediction of physical success or failure. V7 requires an operator trial.
Fast v2 has one [operator-confirmed successful run](../fast-tip-up/trial-v2-20260920/README.md),
with catch/recoil travel retained for later investigation.

## Installation and recovery

Application-only transfer completed in 421.475 s. Exact image/slot, fresh IMU,
all six powered motors online/error-free/disabled, disarmed groups and released
maintenance verified. Saved schema6 exports retained all 1,519 rows identically:
CSV `eb4455fcb12291c111761d5bfe15daac550bb44014af3b84f588ea78e44e4e9e`;
wire `0f856d5d8d3a0e7341de203ac4d1d840823741674c7463611a0cc96e8cd5f245`. New captures use schema7, still 240 bytes/sample.

No autonomous motion, settings/calibration/filesystem writes, credential copying
or edits in another owner's checkout occurred. Exact previous v6 recovery:
`worktrees/balance-lower/artifacts/lowering-v6-fast/candidate/`, source `a772ecc`,
ESP digest `fdf7af6a96d2b6e3c3c908301ae765fcfa4c9110df27f8d88693dd80e8aeb2c2`.
Use the [same verified OTA command](../../docs/WIFI_OTA.md), preserving newer logs
first; no automatic boot rollback. Next: a separate CH6 LOW/CH11 lowering trial,
then disarm and archive before another run.
