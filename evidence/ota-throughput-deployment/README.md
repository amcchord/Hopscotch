# Installed OTA throughput update — September 20, 2026 (EDT)

Austin authorized pushing the prepared changes while the robot was powered.
The lowering/integration owner transferred exclusive device and release
ownership to this task. No other task accessed the robot during deployment.
All changes and evidence are in `worktrees/ota-throughput` on
`codex/ota-throughput`; other checkouts remain untouched.

## Frozen application

- Firmware source: `45c1a94aa82aab952be420e5472c5ae8e377bef1`.
- 1,214,928 bytes; whole-file SHA-256
  `b3f12485e2b61dcc1f4dd11c120beffbac46a41e18bfe8847bdb77221d948109`.
- ESP image digest:
  `a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61`.
- [Preparation manifest](manifest.json). Its `prepared_not_installed` status is
  the immutable preparation state; completed deployment evidence supersedes it.
- Private package: `artifacts/ota-throughput/release/` in this worktree.
  Reused the configured, validated build after verifying unchanged source and
  expected private configuration in the binary without exposing values.
- Recovery: exact v10 application at
  `worktrees/balance-lower/artifacts/lowering-v10-fast/candidate/`, verified in
  place against its original manifest. No private binary/header was copied
  between worktrees. The app has no automatic boot rollback; boot-broken
  recovery requires slot-aware USB diagnosis, preserving settings/filesystem.

Firmware adds all-channel strongest-signal AP selection, AP/channel telemetry
and receiver timing diagnostics. It preserves the v10 motion implementation
and existing OTA percentage/byte/rate screen and motor/RC interlock unchanged.
The uploader uses the optional fast profile: 16 KiB sends, no artificial sleeps.
The default remains paced pending a useful hardware throughput comparison.

## Preflight and interrupted first attempt

[Fresh preflight](preflight-check.json) verified the v10 image in app1, six
powered healthy disabled motors, disarmed IDLE state and fresh IMU. The RC link
was up, Wi-Fi around -55 dBm. Each attempt validated and archived the same
1,462-row saved run before uploading. CSV SHA-256:
`2aa0e23d3d25633bedf29f8655c84b9fea765772e8bb22d42d690a1bd1eaaa71`;
wire SHA-256:
`a4a4347329062c70b9b868175363fb078c073619b08b88d6da919820dbaf0e35`.

[Attempt 1](attempt-1/deployment.json) queued the complete multipart body in
about 11 seconds of socket-send time, but the 120-second HTTP response wait
expired. An old fallback repeated the response wait, then closed the socket.
The automatic 60-second image check expired while the old application was
still receiving. This record correctly retains `verification_failed`.

[Read-only recovery](attempt-1/recovery-active.json) later observed active OTA
at 1,071,102 / 1,214,928 bytes, motors disabled, RC suspended and maintenance
held. Austin [confirmed the screen percentage was increasing](screen-observation.json).
This is operator confirmation, not an assistant visual inspection. Subsequent
[recovery state](attempt-1/recovery-observations.jsonl) recorded a safe abort:
`inactivity_timeout`, 1,082,590 bytes received, 15,001 ms idle. V10/app1 remained
installed, maintenance released, RC returned, and all six motors were healthy
and disabled. No blind retransmit or robot reset was performed.

Host-only fix `243bff2` allows 900 seconds for the final response while retaining
120-second connection/send timeouts, and never reuses a timed-out response
reader. [All 16 focused host tests pass](uploader-timeout-tests.txt), including
an actual localhost receiver whose acknowledgment outlasts the send timeout,
an early HTTP rejection, and single-close failure handling. Firmware bytes
were not rebuilt or changed for the retry.

## Retry and final verification

[Attempt 2](attempt-2/deployment.json) succeeded using the same frozen
application, fast profile and fixed helper. HTTP 200 acknowledged verification;
the exact ESP digest was verified in app0 at
2026-09-21T00:57:10.360018+00:00. Transfer: 305.815 seconds, 3.880 KiB/s
(3.973 decimal kB/s); 275.442 seconds inside socket sends, max single send
15.213 seconds, response wait 30.363 seconds and no pacing sleeps. These socket
stalls do not distinguish Wi-Fi delays from receiver flash/application delays.
The old receiver has no new timing headers, so flash timings are unavailable.
The result is not a controlled A/B speed comparison.

Both groups were disarmed, all six powered motors returned healthy disabled
feedback, the IMU was fresh, maintenance released and RC link restored. The
1,462-row saved CSV and wire export are byte-identical before and after, and
match the first attempt's backup. Raw exports are retained locally beside each
attempt's record and gitignored; the validated hashes are committed.

The new image reports `wifi_ap_selection=all_channels_strongest_signal` and
`ota_metrics_version=1`. Its first boot associated at approximately -70 dBm on
channel 6, BSSID `A8:9C:6C:2C:27:DF`, weaker than pre-upload. One explicitly
recorded [maintenance-only reconnect](ap-reconnect.json), after fresh disarmed
health checks, selected BSSID `8C:30:66:7A:2E:DD` on channel 1, improving -69 to
-55 dBm. Exact image, healthy disabled motors, fresh IMU, released maintenance
and RC link were reverified at 2026-09-21T00:57:59.334995+00:00. This confirms a
stronger AP was selected on reconnect, not that every scan always sees all APs.
There is no periodic roaming or scan during motion. The next normal authorized
update can measure throughput on this association and collect receiver timings.

Source preservation is recorded in [source-preservation.json](source-preservation.json).
The firmware's 12 native / 42 Python, pinned build, TCP, radio and dashboard
checks were reused from the exact unchanged candidate. The host-only timeout
fix adds two regressions; all 16 uploader tests passed. No redundant firmware
rebuild or reflash was performed after success.

No motion was initiated, no settings/filesystem writes were requested, and no
GitHub push was performed. Strongest-AP association and receiver timing headers
only take effect after this candidate boots; this upload still uses the old
firmware connection and cannot establish a strongest-AP speedup.
