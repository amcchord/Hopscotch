# V13 contact correction installed and verified

Source `7c32bd7a6e48e641345a7fbc067a11d46a0643bf` is installed in app0,
verified at `2026-09-21T02:22:24.302135+00:00`. ESP digest
`2cba5a84594a703bb377b6697791fca4da790e8b6c8d4fe5ddf0f2181461cdae`.
[Prepared manifest](manifest.json), [verified deployment](deployment.json).
The manifest describes preparation; the deployment establishes installation.

The 1,216,640-byte application has whole-file SHA256
`3f6346766a7f8e2e1a3856e03e9b8daa884c22682ea7b8ffe92a4e29a0e6a7df`.
Configured binary/ELF/checks remain private in this checkout at
`artifacts/lowering-v13-contact/candidate/`; existing credentials were read in
place and verified in the binary without printing or copying the secret header.

## Correction and checks

[The physical v12 failure](../balance-lower/v12-contact-stop-20260921T020520Z/README.md)
held both targets still after contact because the new load veto never passed.
V13 continues the established normal target return under calm weak-torque
readings after confirmed support. Rapid weak-load falls still pause. All other
motion/guard behavior is preserved, including the successful fast v2 stand-up.
[Policy verification](../balance-lower/fast-return-v13/README.md) includes the
recorded stall, pending-target behavior, six identical normal replays,329 paired
model cases and40 post-support low-torque stresses. Physical success is not
inferred from replay or the approximate model.

After source freeze, all12 native suites,44 Python tests, script syntax,
configured pinned ESP32 build, radio C++/Lua and dashboard checks passed.
Application section1,216,269 bytes; RAM53,600. Existing Arduino macro warnings
remain. Independent fast-tip owner reviewed source/trace/model results and
found no blocker. No firmware rebuild after freeze.

## OTA and preservation

The existing user-authorized update scope covers this correction. Fresh
idle/disarmed, saving-finished, powered healthy disabled motors and IMU checks
passed before backup and upload. The transmitter remained connected. The frozen
image transferred with16KiB sends and no pacing sleeps: HTTP200 in431.250s.
Receiver writes5.698873s (max153.864ms), verification147.555ms, maxreceivegap9.669s;
sender blocked381.085s and waited50.121s for the response. These measurements
alone do not isolate the transport bottleneck.

Postflight verifies exact digest/app0, six healthy powered disabled motors,
fresh IMU(age1,640us), idle/disarmed, saving finished, released maintenance and
returned RC(age1ms). Wi-Fi remained channel1/-56dBm. No autonomous motion,
settings change, calibration or filesystem-image write.

The failed1,140-sample run remains byte-identical before/after:
CSV `03971226f22eb9c2a2eb8f789624db02eda5035469c23fead2a42dd2858ee67d`,
wire `ba55265f2bdfde0564d71df3f5d9bf95d4b8a95a354c30d30cf7bbbc02a80822`.
Raw OTA exports remain in `artifacts/lowering-v13-contact/deployment/`.

V12 package remains for diagnosis; the last physically successful45c1a94
recovery package stays in the OTA task's original checkout. No private package
was copied between worktrees. No automatic boot rollback. Follow the
[slot-aware recovery procedure](../../docs/WIFI_OTA.md) if recovery is requested.

Ready for one operator CH6 HIGH/CH11 trial, then disarm and archive. Physical
completion and speed of the corrected fast return still need that trial.
