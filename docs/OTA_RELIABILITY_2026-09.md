# OTA transport reliability — September 20, 2026

The installed networking update follows the [verified lowering-v2 deployment](../evidence/ota-lowering-v2/README.md).
Two transmitter-on uploads disconnected without an ESP reboot; the same paced
upload succeeded with the transmitter off. That pattern suggests contention,
but does not establish RF interference or a particular library timeout as the
physical cause. The lowering task owns the separate motion investigation.

## Targeted change

The pinned ESP Async WebServer 3.0.6 configures every accepted connection with
`setRxTimeout(3)`. Async TCP 3.1.4 closes a connection after that many seconds
without received data. Increasing only the desktop client's 120-second timeout
does not change this server limit. Hopscotch already has a separate 15-second
OTA inactivity watchdog.

The networking update changes only the networking implementation:

- After authentication, maintenance grant and application-header validation,
  an upload receives a 20-second TCP receive timeout, a 15-second ACK timeout
  and TCP_NODELAY. Ordinary requests retain the short library default.
- OTA's 15-second inactivity watchdog remains authoritative and releases
  maintenance on abandonment. A short gap can recover through normal TCP
  retransmission; an actually disconnected upload still aborts safely and must
  restart. This is not a resumable application protocol.
- New WebSocket telemetry offers pause while uploading or awaiting reboot.
  Existing queued frames may drain. GET telemetry remains a fresh RAM snapshot,
  and dashboard offers resume after an abort. Motion-time streaming is unchanged.
- `/api/info` reports `ota_transport_version: 2`, `ota_rx_timeout_s: 20`,
  `ota_ack_timeout_ms: 15000` and `ota_idle_timeout_ms: 15000`.
  Live telemetry adds `ota.active`, `received_bytes`, `expected_bytes`,
  `last_failure`, `failed_bytes` and `failed_idle_ms`. Failure diagnostics last
  until the next valid upload or reboot. They distinguish a disconnect from
  inactivity/write/hash/ESP-verification failure without disclosing credentials.

The control-owner maintenance gate, fresh switch-low rearm, application-only
inactive-slot writes, exact length/full-file SHA verification, ESP verification,
credentials, normal radio/drive/lowering behavior and pinned dependencies are
preserved. Automatic boot rollback remains unavailable.

## Validation and hardware handoff

`python3 scripts/check_ota_transport.py` compiles the actual upload/abort/watchdog
methods with hardware stubs and the installed pinned Async TCP `_poll` method.
It reproduces the default four-second-gap failure, verifies that the candidate
survives the same gap, and checks bounded inactivity, late data after expiry,
disconnect diagnostics, exclusive upload ownership, authentication/eligibility
and final length/hash/ESP-verification guards. Cryptographic and flash primitives
are stubs in this harness; the firmware build and hardware handoff cover those
interfaces. Bootstrap missing dependencies with `pio pkg install --no-save`.

At source freeze, run the consolidated checks once, the focused transport
harness and radio/dashboard checks. Freeze the configured binary and manifest;
do not copy the private header or reuse an example-credential binary. No
lowering-model rerun is needed when motion sources are byte-identical.

Hardware acceptance passed in the [combined v4/fast-tip-up update](../evidence/lowering-v4-fast-integration/README.md):
transmitter on, four-second deliberate receive gap after 256 KiB, HTTP 200,
expected app0 digest, healthy disarmed reboot, and identical 1,908-row saved-run
CSV/wire. All 67 successful RC observations were fresh and disarmed; five
read-only monitor requests timed out. Transfer took 249.699 seconds. This is one
successful trial, not proof against all RF conditions; the monitored workload
and deliberate pause differ from routine deployment. Transmitter-off previously
took about 63 seconds. Ordinary updates reuse the same paced helper without
monitoring/gap injection or a redundant reinstall.

The old long-log exporter separately caused a task-watchdog reset before an
upload. Installed source `8449ddb` fixed this with cooperative checksum/CSV work
and longer eligible download timeouts, preserving the filesystem and recovering
the log. That fix remains in the new combined image.
