# Lowering v12 installed and verified

Source `09d2e01b6ef89a357319a8eeea2fa11db8221932` is installed in app1, verified
at `2026-09-21T02:00:40.530835+00:00`. ESP digest
`3b5c5f5601ffdac322e0c9bd6ddf06934821f71d9b90f6549c12a172844772f3`.
[Prepared manifest](manifest.json), [verified deployment](deployment.json).
The manifest records preparation; the deployment establishes installation.

The configured application is 1,215,936 bytes, whole-file SHA256
`702508c46121c3a01bbd49fad6655ae2888b23950a5772c6793cb43acd60f4e9`.
Private binary/ELF/check output remain in this checkout under
`artifacts/lowering-v12-fast/candidate/`. Existing credentials were read in
place and verified in the built application without printing or copying them.

After source freeze, all 12 native suites, 44 Python tests, script syntax,
pinned configured ESP32 build, radio C++/Lua and dashboard checks passed.
Application section is 1,215,569 bytes and RAM 53,600 bytes. Existing Arduino
macro-redefinition warnings remain. [Production-policy model and replay](../balance-lower/fast-return-v12/README.md)
preserve normal mode and initial catch. The fast-tip owner independently
reviewed source and all paired results with no blocker.

## Deployment and preservation

Austin's existing OTA authorization covers this requested lowering-only update.
The latest successful fast-v2 stand-up is unchanged; the unflashed gain
experiment was explicitly deferred. No autonomous motion was initiated.

The helper verified fresh idle/disarmed, finished saving, healthy powered
motors and IMU; backed up the latest 1,192-row run; then sent the frozen image
using 16 KiB chunks without pacing sleeps. The transmitter remained connected.
HTTP 200 in 438.820 s, about 2.706 KiB/s application throughput. Receiver flash
writes took 5.215 s total (max single write 154.555 ms), verification 149.445 ms,
max receive gap 13.487 s. Sender blocking totaled 405.642 s; response wait 33.047 s.
These figures do not establish why transport was slower than the previous OTA.

Post-reboot identity/app1, six healthy powered disabled motors, fresh IMU,
released maintenance, disarmed groups and returned RC link passed. Wi-Fi stayed
on channel 1 at -56 dBm. Both saved-run exports are byte-identical:
CSV `e79cfe64d1bb507455097198edf027db0da21ccbadd282a502958ccc49a274aa`,
wire `fa1e67fadb87cbca8dda08befab780e7099f328af9e154d29b8cc1480f9e8251`.
Original pre/post exports remain under `artifacts/lowering-v12-fast/deployment/`.
No settings, calibration or filesystem image was written.

Preceding verified 45c1a94 recovery package remains in the OTA owner's checkout,
`worktrees/ota-throughput/artifacts/ota-throughput/release/`; no cross-worktree
private copy was made. No automatic boot rollback is enabled. Use the normal
[OTA/recovery guide](../../docs/WIFI_OTA.md) if the operator requests recovery.

Next: one manual CH6 HIGH / CH11 trial, then disarm and archive. Three times
maximum arm target speed gives about twice modeled supported descent speed;
physical speed and repeatability are not yet measured on v12.
