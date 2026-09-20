# Wi-Fi telemetry and OTA

Hopscotch joins the configured 2.4 GHz network as `hopscotch.local`. Open
`http://hopscotch.local/` (or the address on the robot display) for live pose,
motor feedback, RC link state, control timing, saved-run download and application
updates. This is a LAN service. The September 19 device is configured for
SvensHaus. No Internet server is required or deployed.

## Control isolation

- The existing 200 Hz IMU/balance task remains on core 1 at priority 18; the
  control task remains on core 1 at priority 12. Balance gains, startup logic,
  driving v4 dynamics, motor setup and CRSF mappings are retained.
- The control owner copies a fixed-size snapshot into a one-element FreeRTOS
  queue at 50 Hz. This does not allocate, format JSON, wait for a socket or
  perform filesystem I/O. The balance task separately publishes timing counters.
- A core 0 task at priority 2 formats snapshots and offers telemetry at 10 Hz.
  Wi-Fi/lwIP, Arduino network events and async_tcp also run on core 0;
  async_tcp stays at priority 3. Arduino events otherwise default to core 1
  at priority 19, so `ARDUINO_EVENT_RUNNING_CORE=0` is explicitly required and
  checked at compile time. `/api/info` reports actual task affinity.
  Four WebSocket clients are allowed, with at most two frames queued per client.
  Each slow client drops telemetry independently. Delivery rate depends on the
  network; the browser marks a stale connection and disables maintenance controls.
- Core isolation alone does **not** make flash operations safe while balancing.
  ESP32 flash access can suspend both cores. OTA, saved-file export and Wi-Fi
  association use a maintenance request that only the control task can grant.
  A cached HTTP snapshot is never sufficient authorization for flash access.
- Maintenance is rejected during balance, drive/arm arming or armed state,
  any individually enabled motor (including partial arming),
  calibration, simulation, motor test, USB download, or pending log/trim save.
  Once granted, the control owner inhibits arming and motion triggers, continues
  receiving RC/CAN, and discards prohibited serial commands. Firmware upload
  holds this state through reboot; failed/abandoned uploads abort and release it.
- Initial Wi-Fi association and later reconnect scans hold this interlock until
  connected or a 12-second timeout cancels association. Lost Wi-Fi does not
  disarm or otherwise modify a moving robot. Reconnection attempts wait until
  disarmed. Normal motion and failsafes continue to use ELRS.
- Boot and maintenance require a fresh observation of both RC arm switches low
  before rearming. An update never requests a motor to arm.

The timing counters measure actual intervals between 200 Hz task iterations.
They exclude boot's first five seconds and maintenance, and reset after
maintenance. They measure scheduler/IMU cadence even while disarmed; they are
not proof of powered PD/CAN stability. Existing run-scoped balance telemetry and
stall forensics remain the record for physical tests.

Espressif's [flash concurrency documentation](https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32s3/api-reference/peripherals/spi_flash/index.html)
explains the cross-core cache constraint. This build retains Arduino 2.0.16 /
PlatformIO espressif32 6.7.0 and the previously pinned motor/IMU dependencies.

`scripts/patch_asynctcp.py` applies two checked fixes to the pinned Async TCP
3.1.4 source during every build: reject invalid accept callbacks, and stop its
error callback from accessing a PCB already freed by lwIP. The original source
SHA-256 must match exactly; dependency changes fail the build pending review.
These fixes address a core-0 panic reproduced by concurrent HTTP connections
and a stalled WebSocket client. `/api/info` also reports the ESP reset reason.

## Local configuration and access

Copy `src/network_secrets.example.h` to `src/network_secrets.h` and fill in
SSID, Wi-Fi password, a random device API token and a distinct recovery AP
password. The actual file is gitignored and mode 0600 on this workstation.
Wi-Fi credentials and the token are present in the device application binary;
keep firmware packages and full-flash backups private. Never commit the local
header or paste its contents into a worklog.

The token is needed for log downloads, disarm, reconnect and firmware upload.
The browser only keeps the entered token in its current page; the CLI reads it
from the local header or `HOPSCOTCH_API_TOKEN`. Read-only status is available on
the trusted LAN. HTTP is not suitable for exposing this service on the Internet.
A future Internet relay must use authenticated TLS and remain outside control.

If the configured network is unavailable, a disarmed retry after 30 seconds
starts `Hopscotch-Recovery` using the local recovery password; its address is
normally `192.168.4.1`. An unavailable network never blocks the normal control
loop indefinitely. The recovery AP stays enabled until reboot once started.

## Operations

```bash
# Live state; add --host http://<device-ip> before the subcommand if needed.
.venv/bin/python scripts/robot_wifi.py status

# Fetch and validate the same checksummed CSV exported over USB.
.venv/bin/python scripts/robot_wifi.py log

# Safe stop request, executed by the control task.
.venv/bin/python scripts/robot_wifi.py disarm

# Reconnect only while disarmed; used for bench recovery checks.
.venv/bin/python scripts/robot_wifi.py reconnect

# Build and install an application, preserving filesystem and calibration.
./scripts/build.sh
.venv/bin/python scripts/robot_wifi.py ota .pio/build/m5stack-atoms3r/firmware.bin
```

The dashboard is embedded in `firmware.bin`, so its updates travel with OTA.
**Do not use `uploadfs`**: the existing LittleFS volume holds settings,
calibration and the saved balance run. The previous web settings/CAN mutation
endpoints return 410; they mutated control state from a networking callback.
Use the existing USB console for tuning/calibration until a transactional
configuration API is implemented. No Wi-Fi arming, steering or balance command
path has been added.

A log export first copies a checksummed CSV into a bounded 4 MiB PSRAM buffer
under maintenance, then releases the interlock and sends that immutable copy.
One download can be in flight; a second gets 429. A slow/down client cannot
cause filesystem reads while the robot is subsequently armed. The CLI validates
row count, schema, sample values, timestamps and transport/device checksums,
and saves both cleaned CSV and original `.wire` data without overwriting files.

OTA requires an application image, exact byte count, SHA-256 and bearer token.
It streams into the inactive app slot, verifies size/hash and the ESP application
before selecting that slot, then reboots. Disconnects and 15-second upload
inactivity abort incomplete updates. There are two existing 3,342,336-byte slots
and a separate 1.5 MiB filesystem; the partition table is unchanged.

**Automatic boot rollback is not enabled in the existing bootloader.** Failed
or interrupted uploads preserve the active image, but a valid image with a boot
bug can still require USB recovery. Do not confuse integrity checking with a
signed firmware trust chain or automatic health rollback.

## Recovery

The complete pre-upgrade 8 MiB device readback is stored privately in
`artifacts/wifi-ota/pre-upgrade-flash.bin`. Its SHA-256 and the installed release
identity are recorded in `evidence/wifi-ota/README.md`. Application packages
are in `artifacts/wifi-ota/`.

After OTA, do **not** assume app0 is running: inspect `/api/info` for
`running_slot` or inspect OTA data over USB. The old
`scripts/flash_prepared_balance.py` intentionally programs only app0; it is not
a general recovery tool for a device booting app1. Do not overwrite partition,
NVS, OTA data or the filesystem just to update firmware. For a full restoration,
preserve any newer run/settings first and use the complete backup only with a
supported, disarmed robot. Download the saved run before any recovery.

## Validation

Run `scripts/check_balance_candidate.sh`, `bash scripts/check_radio_telemetry.sh`
and `node tests/test_network_dashboard.js`. The maintenance test enumerates all
1,024 combinations of blocking conditions and races cancellation against grant.
The dashboard SHA implementation is compared with Node's independent SHA-256,
including block boundaries and a full firmware-sized input.

With motor power **off**, run the hardware load regression:

```bash
.venv/bin/python scripts/check_wifi_load.py --host http://hopscotch.local \
  --seconds 180 --output output/wifi-load.json
# Normal dashboard delivery without connection churn / saturation:
.venv/bin/python scripts/check_wifi_load.py --host http://hopscotch.local \
  --mode normal --seconds 120 --output output/wifi-normal.json
```

This requires Node with built-in WebSocket support (Node 26 was used). It opens
one nonreading WebSocket with a small TCP receive window, three active telemetry
readers and three repeated HTTP workers. It checks continuous uptime, 200 Hz
task progress, no new intervals above 7.5 ms, RC/IMU health, bounded telemetry
backpressure and successful client delivery. Overload permits bounded HTTP
timeouts and requires WebSocket reconnection/recovery; normal mode requires
zero transport errors/reconnects, at least 4 Hz and no gap over two seconds.
It refuses online motors or an
unsafe maintenance state, and preserves existing output evidence.
Use `--ignore-radio` for network-only, motors-off testing when the transmitter
is intentionally off or being adjusted. It still checks motor state and IMU
health, and records RC measurements without treating link loss as a failure.

See `evidence/wifi-ota/README.md` for hardware checks. Initial validation is with
motor power off. A powered stationary check and supervised balance/drive trial
remain separate operator steps.
