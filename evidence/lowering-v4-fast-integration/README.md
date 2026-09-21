# Combined lowering v4 / CH6 fast tip-up release

Source `e55cecb235728f874ea88cb81a50f7ad4628964d` merges lowering `f4fafc7` and
the independently owned fast tip-up checkpoint `3814f1f`. No other checkout was
edited. Telemetry feature allocation is combined to 65535: 32768 lowering v4,
16384 fast tip-up, plus the historical bits; sample schema/size remain 4/240.

The frozen configured application is 1,203,520 bytes, whole-file SHA-256
`6b648fbb0c584146954b7f02d9d34d2e5f24cd06be7965169fcf5f2d078ff808`,
ESP digest `62e38da80f1a6e61bb8ed1162c5bf4f83b280d0001d82469f500131dc96f7f42`.
Private binary/ELF/partitions are retained at
`worktrees/balance-lower/artifacts/lowering-v4-fast/candidate/` under the project;
see [manifest](candidate-manifest.json). The exact previous v3 package remains
`worktrees/balance-lower/artifacts/lowering-v3-export/candidate/`. No secrets,
configured binaries or caches are copied between worktrees or published.

**Installed and verified** at 2026-09-20 19:31:57 UTC, app0. The
[deployment record](deployment.json) verifies exact digest/slot, six healthy
powered disabled motors, fresh IMU and released maintenance. The 1,908-row
v3 trial CSV and wire are byte-identical before/after. No motion was initiated.

The transmitter stayed on. The sender paused four seconds after 256 KiB;
HTTP 200 arrived after 249.699 seconds, followed by the expected reboot.
67 successful read-only observations all showed RC link up (age at most 1 ms)
and both groups disarmed; five monitor HTTP reads timed out. Postflight RC was
fresh and disarmed. This establishes one successful transmitter-on/gap update,
not universal RF reliability or speed. The earlier transmitter-off transfer
took about 63 seconds. The ordinary updater does not inject the gap or add
this monitoring workload; the cause of slower throughput is not isolated.

## Behavior and validation

[Lowering v4](../../docs/archive/2026-09/BALANCE_LOWER_V4_2026-09.md) continues the return toward
Forward once both arms contact, rather than stopping upright after the v3
capped retreat/fault. It confirms measured level body and Forward arms before
completion. [CH6 fast tip-up](../../docs/archive/2026-09/FAST_TIP_UP_2026-09.md) is a separate
experimental option: high selects fast, center/low preserves the existing slow
trajectory. First test lowering with CH6 low; test fast standing separately.
Neither new maneuver has physical acceptance yet.

Combined validation passed once after source freeze: 11 native executables,
37 Python tests, Python/shell syntax, whitespace, pinned configured ESP build,
radio C++/Lua fixtures, dashboard/SHA checks and actual-method OTA transport
harness. Build uses existing private configuration through an include path;
all four configured values are present in the private image without disclosure.
RAM 53,448 bytes; flash 1,203,157 bytes. Existing event-core macro warning remains.
The [source audit](source-preservation.json) confirms ground/standing drive,
network, radio and dashboard sources unchanged, both owners' policies retained,
and historical v1/v2/v3 log-export metadata byte-identical.

Reused [329-case paired lowering screen](../balance-lower/continuous-return-v4/README.md)
and [162-case fast trajectory screen](../fast-tip-up/README.md); the modelled
policies are byte-identical after integration. Lowering improves 257→260
completions with four regressions; stiff delayed impacts still fault. Fast
capture is within 2.74 seconds in tracking fixtures, not a whole-robot contact
dynamics guarantee. These limits require restrained manual trials.

## Deployment procedure

The [archived execution script](deployment-script.py) was run from
`output/deploy_v4_with_transmitter_check.py`; its relative paths describe that
original location. It wraps the production updater without bypassing any
checks: validated saved-run backup, exact manifest/application, fresh disarmed
preflight, paced inactive-slot upload, digest/slot/health verification and an
identical post-update export. The wrapper only adds a deliberate four-second
sender pause after 256 KiB plus read-only RC/arming observations every two
seconds. This tests transmitter-on OTA during the same update, without a second
flash or model/build rerun. No motion, settings/calibration change or filesystem
upload is requested. The final deployment record establishes installed status.
