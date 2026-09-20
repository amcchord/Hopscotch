# September 20, 2026 — forward-fall/catch v2 installed

Austin authorized OTA with the robot powered, supported and safe for manual
testing. Other task owners yielded device access. This task used the frozen
configured release in place; no rebuild or repeated firmware/model checks were
needed. No arming or motion was requested. Shared progress records remain owned
by the root coordinator.

- Source: `dd74154aa3f534f95506b5c0dd66e1c6d901e091`; release validation `e9e1c65`.
- Private application: `worktrees/drive-braking/artifacts/lowering-v2/candidate/firmware.bin`
  relative to the project root, 1,196,112 bytes.
- Whole-file SHA-256: `71ca02be6f0471a63f92605f6ae0204acd45a58539a5eec7da0e2328a9542d72`.
- Installed app0 ESP digest: `7668a0215df34b7e5c0030a23705ba8016343260d1e6bab7aacfe202897c7190`.
- Verified completion: `2026-09-20T18:12:21.968203+00:00`.

[deployment.json](deployment.json) records fresh preflight, exact source/package,
transfer, reboot and postflight. The older build-date string remains `Sep 19 2026
23:44:38` because that unchanged translation unit was reused; digest and slot,
not build date, establish this installation. The package manifest is immutable
preparation history and still says `queued_not_installed`; this deployment record
supersedes that status. The embedded manifest in the record likewise describes
preparation, not the final outcome.

Both first attempts used 1 KiB/50 ms pacing, TCP_NODELAY and a 120-second socket
timeout with the transmitter on. They disconnected after 23.986 and 9.944 seconds;
[attempt-1.json](attempt-1.json) and [attempt-2.json](attempt-2.json) preserve the
evidence. Old app1/ESP digest and continued uptime were checked before retrying;
the device did not reboot, and maintenance released. Austin then switched the
transmitter off. The same paced transfer returned HTTP 200 in 62.759 seconds and
the expected image booted in app0. This strengthens a radio/transport contention
hypothesis but does not prove the underlying cause.

After reboot, all six powered motors were online, disabled, fresh and error-free;
both groups disarmed, balance Idle, healthy IMU and no active maintenance. The
final voltage was about 23.35 V. Both switches must be observed low before rearm.
The 2,371-sample prior failed trial was downloaded/validated before and after:

- CSV SHA: `3083a00737288438466b1d0bbd71f337d433d349e85f7d1b2b3266d4a74b2368`.
- Exact wire SHA: `5c14a7f3df11dc7b1a14a242d79eb2d3f8e4838cdb81403d9d3142fe21f09ab6`.
- Both equal the already committed [trial archive](../balance-lower/trial-20260920/README.md).
  Duplicate downloads remain local under `output/ota-lowering-v2/`.

The previous configured app remains at
`worktrees/drive-braking/artifacts/drive-braking-v5/release/firmware.bin`, whole
SHA `a3b1e64c109bfc91eb43842769efebefbc9b32d16baea290de0af2f6b7a8b982`, verified
before deployment. Application-only OTA preserved the saved filesystem log;
calibration was not independently re-read. Automatic boot rollback is unavailable.

The host updater now automates the demonstrated pacing, manifest checks, saved
run backup/comparison and powered-disarmed image/health handoff. Ten focused host
tests cover manifest/image integrity, unsafe/missing/stale state, powered motor
recovery, lost acknowledgment, already-installed no-op, corrupt export, changed
saved log and exact authenticated multipart transport through a local HTTP server.
See [helper-tests.txt](helper-tests.txt). Those helper checks do not claim a new
hardware trial of the refactored CLI; the live deployment used the equivalent
recorded paced transfer while the helper was being prepared. Firmware sources
were not changed for this deployment. The next work is a separate OTA transport
reliability improvement; physical lowering is ready for its restrained trial.
