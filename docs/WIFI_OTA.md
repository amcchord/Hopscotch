# Wi-Fi telemetry and OTA

Hopscotch joins the configured 2.4 GHz network as `hopscotch.local`. Open
[hopscotch.local](http://hopscotch.local/) (or the address on the robot display) for live pose,
motor feedback, RC link state, control timing, saved-run download and application
updates. This is a LAN service. The last tested DHCP address was [192.168.1.172](http://192.168.1.172/).
No Internet server is required or deployed.

This is the current operating guide for firmware updates and telemetry.
[Current state](progress/CURRENT.md) identifies the installed application;
[latest deployment evidence](../evidence/lowering-v10-integration/README.md) records
the current image and powered disarmed verification. The [initial network
validation](../evidence/wifi-ota/README.md) records the earlier motor-power-off
load and failure tests. Use [BALANCE_TESTING.md](BALANCE_TESTING.md) for
physical trials. Older dated releases and their app0-only USB commands are
historical records, not the update procedure for this firmware.

## Control isolation

- The existing 200 Hz IMU/balance task remains on core 1 at priority 18; the
  control task remains on core 1 at priority 12. Balance gains, startup logic,
  motor setup and CRSF mappings remain under local control. Current driving and
  lowering behavior is described in the [test guide](BALANCE_TESTING.md).
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
  receiving CAN, and discards prohibited serial commands. Ordinary maintenance
  continues receiving RC. With the new OTA progress/interlock firmware, an OTA
  grant first clears pending motion, sends motor stops and closes the CRSF UART
  on the control owner before flash access is permitted. This pauses RC RX/TX
  and UART interrupts throughout upload. Firmware upload holds maintenance
  through reboot; failure releases it and restores an empty UART/parser with
  fresh RC input and switch-low rearming required.
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

The next network candidate adds strongest-signal AP selection and an optional
fast upload profile. See [OTA throughput investigation](OTA_THROUGHPUT_2026-09.md)
for evidence, timing diagnostics and the pending hardware comparison. The
existing paced helper remains the default; no live roam is added during motion.

For a new checkout, copy `src/network_secrets.example.h` to `src/network_secrets.h` and fill in
SSID, Wi-Fi password, a random device API token and a distinct recovery AP
password. The actual file is gitignored and mode 0600 on this workstation.
Wi-Fi credentials and the token are present in the device application binary;
keep firmware packages and full-flash backups private. Never commit the local
header or paste its contents into a worklog. Keep the already configured local
header when updating this robot; do not replace it with the example. Changing
credentials or the token in a build changes what is needed after its reboot.

The token is needed for log downloads, disarm, reconnect and firmware upload.
The browser only keeps the entered token in its current page; the CLI reads it
from the local header or `HOPSCOTCH_API_TOKEN`. Read-only status is available on
the trusted LAN. HTTP is not suitable for exposing this service on the Internet.
A future Internet relay must use authenticated TLS and remain outside control.

If the configured network is unavailable, a disarmed retry after 30 seconds
starts `Hopscotch-Recovery` using the local recovery password; its address is
normally `192.168.4.1`. An unavailable network never blocks the normal control
loop indefinitely. The recovery AP stays enabled until reboot once started.

## Live telemetry and saved runs

Commands below run from the project root using the current integration helper.
It needs only Python 3's standard library. The historical root helper predates
the latest saved-log schema. The default host is `http://hopscotch.local`; put
`--host http://192.168.1.172` before the subcommand to override it. The existing
private header is read in place through `--secrets-file`.

```bash
# Read the latest snapshot; this command is a single read, not a stream.
python3 worktrees/balance-lower/scripts/robot_wifi.py --secrets-file src/network_secrets.h status

# After a run: disarm both groups and wait for log saving to finish.
python3 worktrees/balance-lower/scripts/robot_wifi.py --secrets-file src/network_secrets.h log
```

The browser receives live pose, motor feedback, RC channels, timing and memory
through `/ws`. Offers are 10 Hz; network delivery is best-effort, can skip frames,
and is not a real-time control channel. The dashboard marks stale data. The full
onboard balance capture runs independently at 50 Hz with 200 Hz aggregates;
new captures are schema 8 (240-byte samples, unchanged from v4–v7), up to 6,000 samples/120 seconds. Live JSON schema 1
and saved-log schema 8 are different formats. Existing older saved runs retain
their original schema and metadata when exported by the new firmware.

After supporting the robot, lower both arm switches and release CH11. Wait for
fresh status showing both groups disarmed, `saving_log: false`,
`maintenance_allowed: true`, and `maintenance: false`, then download. Only the
latest run is stored; retrieve it before the next test or firmware downgrade.

A log export first copies a checksummed CSV into a bounded 4 MiB PSRAM buffer
under maintenance, yielding every 10 ms of work so long exports do not starve
the network-core idle task. The authenticated download has a 120-second receive
timeout. It then releases the interlock and sends that immutable copy.
One download can be in flight; a second gets 429. A slow client cannot cause
filesystem reads while the robot is subsequently armed. The CLI validates row
count, schema, sample values, timestamps and transport/device checksums before
saving cleaned CSV and original `.wire` data. It refuses to overwrite files.
Default names are `telemetry_logs/bal_YYYYMMDD_HHMMSS_wifi.csv` and `.wire`;
use `log --output telemetry_logs/<unique-name>.csv` for a custom path.

Keep both files and your operator observations. Run analysis separately using
the downloaded filename:

```bash
python3 worktrees/balance-lower/scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details
# Optional plot requires matplotlib:
python3 worktrees/balance-lower/scripts/analyze_balance_logs.py telemetry_logs/<run>.csv --details --plot
```

The browser's **Download latest run** saves the raw export. For a validated
archive, use the CLI or run `python3 worktrees/balance-lower/scripts/validate_telemetry.py <raw-download>
<new-clean-output.csv>` afterward. Choose a new output path: the standalone
validator can overwrite its output. Failed Wi-Fi downloads do not automatically
create a diagnostic archive; retry before starting another run. The request
timeout is 120 seconds. USB fallback is `./scripts/save_telemetry.sh --label
<run-name>`; that helper saves `.csv`/`.serial` and also runs analysis.

## Disarm and reconnect

```bash
# Request disarm; confirm both groups are disarmed in fresh telemetry afterward.
python3 worktrees/balance-lower/scripts/robot_wifi.py --secrets-file src/network_secrets.h disarm

# Reconnect only while disarmed; the connection drops while rejoining.
python3 worktrees/balance-lower/scripts/robot_wifi.py --secrets-file src/network_secrets.h reconnect
```

HTTP 202 acknowledges the request, not completion. Keep the radio disarm control
available; Wi-Fi is best-effort. Reconnect holds the maintenance interlock while
associating and does not normally reboot. Lower both arm switches after
maintenance or web disarm before attempting to rearm.

## Update the firmware

Use a frozen, already checked package. **Do not rebuild or rerun the full test
suite simply to deploy those same bytes.** One task owns device access; the
other tasks continue in their own worktrees. Keep the previous exact application
package for recovery.

1. Support the robot, disarm both groups, lower both arm switches and release
   CH11. Motor power may stay on when the robot is safely supported and all
   motors are disabled; cycling power is not a routine OTA requirement. The transmitter may remain on with both arm switches LOW and fresh
   disarmed status. A full transmitter-on update, including a four-second receive
   gap, passed with transport version 2. That monitored transfer took 250 seconds
   versus about 63 seconds for the earlier transmitter-off transfer; switching
   it off remains an option when speed matters. The exact throughput cause is
   unconfirmed. Older transport still uses the transmitter-off bootstrap path.
2. Run one command with the frozen application and its manifest. For the
   September 20 lowering-v10/CH6-fast-laydown package, from the project root:

   ```bash
   python3 worktrees/balance-lower/scripts/robot_wifi.py --host http://192.168.1.172 \
     --secrets-file src/network_secrets.h ota \
     worktrees/balance-lower/artifacts/lowering-v10-fast/candidate/firmware.bin \
     --manifest worktrees/balance-lower/artifacts/lowering-v10-fast/candidate/manifest.json
   ```

   Use the current integration worktree helper and its frozen package as shown.
   Other tasks may own different root source; the private header is read in place.

   The helper verifies file size, whole-file hash and ESP digest, checks fresh
   disarmed maintenance eligibility, archives and validates the saved run, then
   transfers the application with 1 KiB/50 ms pacing and a 120-second socket
   timeout. It verifies the new running digest/slot, fresh IMU, disarmed state,
   return of previously online motors without errors, and identical saved-run
   exports. Observed uploads took about 64 seconds with the transmitter off, 188 seconds
   for the earlier unmonitored transmitter-on update, 250 seconds with
   acceptance monitoring/gap injection, 343 seconds for v6, 421.475 seconds to
   install the OTA progress/interlock, and 475.235 seconds for the first update
   under that interlock. V9 took 392.262 seconds on retry after an initial
   disconnect at 277,504 bytes. The old image and healthy idle state were verified
   before retrying the identical file. These are observations, not fixed upload
   deadlines. V10 completed in 90.96 seconds with the transmitter unlinked.
   There is no separate
   manual status/log/download loop to repeat. Gap injection and concurrent status
   monitoring are acceptance tests, not routine deployment steps.
3. Wait for the final verified report. The helper saves state, transfer outcome
   and `.csv`/`.wire` exports in a new `output/ota-<UTC>/` directory, printed at
   startup. `--record-dir <new-directory>` selects a durable evidence location.
   Preserve the record and update shared release state once. If the transmitter was off, turn it back on; observe both arm switches low before any authorized motion test.
   Check [current trial status](progress/CURRENT.md) first; fast v10 lowering is ready
   for a restrained manual trial and is not yet physically validated.

After the progress/interlock firmware is installed, subsequent uploads show a
large percentage, exact received/total application bytes, average KiB/s and
elapsed seconds on the robot. A queue supplies the 10 Hz display without taking
the flash mutex. The display reaches 100% only after SHA/ESP verification; an
abort retains its partial count for five seconds, then returns to status. The
update that installs this feature still runs under the preceding firmware.
The first subsequent update completed in 475.235 s with the transmitter
linked before/after. One in-flight snapshot confirmed active OTA, maintenance,
disabled motors and cleared RC/link input, consistent with UART suspension.
The physical progress screen was not independently observed.
This is one observed transfer, not a controlled performance comparison.

`--host` and `--secrets-file` go before `ota`; other OTA options go after it.
An isolated worktree can use `--secrets-file /absolute/project/src/network_secrets.h`
to read the existing private header in place. Never copy secrets or credentialed
binaries between worktrees. The helper reads the frozen binary at its existing
path. `HOPSCOTCH_API_TOKEN` still takes precedence when set.

If an upload loses its acknowledgment, the helper checks the actual running
image before reporting failure. It never blindly repeats a write. Rerunning the
same command after a successful but unacknowledged installation verifies and
archives the current state without another flash or reboot. A failed command
retains diagnostic evidence; inspect image identity and wait until maintenance
has released before retrying. A validation failure must not be treated as a
successful update. Keep controller power steady. If transfer stalls and Wi-Fi
signal has fallen sharply, reposition while disarmed, check the running image
and maintenance state, then retry the same frozen package. During the earlier v5
release, a transfer stopped at 148 KB with approximately −83 dBm signal; after
repositioning to approximately −54 dBm, the complete update verified. Signal
and power changed together, so this does not isolate a single failure cause.
The robot's control owner still independently grants every flash operation; client checks do not replace that interlock.

### Check once, freeze once, deploy the same package

During development, run the focused checks for changed code. At scope freeze,
the integration owner runs consolidated validation once and records the result:

```bash
./scripts/check_balance_candidate.sh
bash scripts/check_radio_telemetry.sh
node tests/test_network_dashboard.js
```

The consolidated check includes the pinned firmware build. Freeze its configured
`firmware.bin`, ELF, source revision, manifest and build/check output in a private
package. Confirm existing private network configuration; an example-credential
compile is not a deployment artifact. Reuse that validated package across
handoffs. Repeat checks only for new changes, failures or an unresolved concern.
Documentation-only changes do not require a firmware build; host updater changes
use `python3 -m unittest discover -s tests -p test_robot_wifi.py -v`. Hardware
network stress tests belong to network changes/diagnosis, not every deployment.
New motion behavior still needs its supervised physical trial.

`/api/info` reports `image_sha256`, the ESP image's internal content digest.
It differs from the SHA-256 of the complete upload file, which includes the
appended digest. Compare like with like; the CLI handles both checks. Firmware
identity/slot data is cached at boot under maintenance, avoiding flash reads
from a status request during balance. Query it read-only with
`curl --fail --silent --show-error http://hopscotch.local/api/info`.

The dashboard is embedded in `firmware.bin`, so its updates travel with OTA.
**Do not use `uploadfs`**: the existing LittleFS volume holds settings,
calibration and the saved balance run. The previous web settings/CAN mutation
endpoints return 410; they mutated control state from a networking callback.
Use the existing USB console for tuning/calibration until a transactional
configuration API is implemented. No Wi-Fi arming, steering or balance command
path has been added.

OTA requires an application image, exact byte count, SHA-256 and bearer token.
It streams into the inactive app slot, verifies size/hash and the ESP application
before selecting that slot, then reboots. Disconnects and 15-second upload
inactivity abort incomplete updates. There are two existing 3,342,336-byte slots
and a separate 1.5 MiB filesystem; the partition table is unchanged.

**Automatic boot rollback is not enabled in the existing bootloader.** Failed
or interrupted uploads preserve the active image, but a valid image with a boot
bug can still require USB recovery. Do not confuse integrity checking with a
signed firmware trust chain or automatic health rollback.

The [current combined deployment record](../evidence/lowering-v10-integration/README.md)
identifies the installed image and verified routine update, including unchanged
saved-run hashes. The transmitter was unlinked for v10. The [historical v2 record](../evidence/ota-lowering-v2/README.md)
retains the old transport's interrupted attempts and transmitter-off success.
Frozen manifests record preparation; deployment records establish installation.

[OTA transport version 2](OTA_RELIABILITY_2026-09.md) extends the upload socket's
receive timeout and pauses dashboard streaming during uploads. Its first
transmitter-on hardware acceptance passed. The slower measured throughput and
read-only monitor timeouts are retained in the evidence; one test does not
establish reliability in all RF conditions.

## HTTP and WebSocket API

Port 80 on the trusted LAN. Protected requests use `Authorization: Bearer
<device-token>`; use the CLI or dashboard to avoid putting tokens in commands
or shared logs.

| Method / path | Token | Behavior |
| --- | --- | --- |
| `GET /` | No | Embedded dashboard, updated with the application |
| `GET /api/telemetry` | No | Latest RAM snapshot, JSON schema 1 |
| `GET /api/info` | No | Build, running slot, image digest, capacity, memory, reset reason and network task cores |
| WebSocket `/ws` | No | Receive-only live telemetry; up to four clients, two queued frames per client |
| `GET /api/log` | Yes | Checksummed raw saved-run CSV copied under disarmed maintenance |
| `POST /api/disarm` | Yes | 202: request queued for control-task execution |
| `POST /api/wifi/reconnect` | Yes | 202: disarmed association requested |
| `POST /api/ota` | Yes | Multipart application upload; exact `X-Firmware-Size` and full-file `X-Firmware-SHA256` required |
| Any `/api/settings`, `/api/change-can-id`, `/api/reset-settings` | — | 410: disabled, including settings GET/export |

Telemetry includes `sequence`, `uptime_ms`, `age_ms`, `drive_armed`, `arm_armed`,
`arming`, `rearm_required`, the 16 `channels`, RC link/age, `balance`, `motors`,
`power`, `timing`, `wifi`, memory and dropped-frame counts. Maintenance fields
are `maintenance_allowed`, `maintenance`, `saving_log`, `calibration`,
`test_mode` and `simulation`. Power values have separate freshness flags;
`current` is summed motor IQ, not battery input current. Snapshot availability
does not authorize an operation: the control owner independently checks the
maintenance request.

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Host not found | Use the display's current IP via `--host` before the subcommand; verify the same LAN. The last tested IP is not a reservation. |
| Stale dashboard / lost Wi-Fi | Check data freshness. Control remains local; reconnect attempts wait until disarmed. Reduce open clients/traffic, then use disarmed reconnect if needed. |
| 401 | Use the token for the installed firmware. A new build may have different local credentials. |
| 409 / maintenance unavailable | Disarm both groups, release triggers, wait for save/download/association to finish and inspect calibration/test/simulation state. Never bypass the interlock. |
| 429 on log download | Let the existing transfer finish or close its client before retrying. |
| 400 on OTA | Use the correct application file and CLI-computed size/hash; retain the rejected file for diagnosis. |
| Update interrupted | After the aborted upload releases maintenance, check the running image and retry. A complete accepted update may already have rebooted. |
| Upload accepted, reboot unverified | Check display, LAN/IP and image identity. Keep motors off; use recovery if the new app cannot boot. |
| Log validation fails | Retry while disarmed before another run; retain any raw browser/USB capture. Do not accept a partial CSV as a complete trial. |

## Recovery

The complete pre-upgrade 8 MiB device readback is stored privately in
`artifacts/wifi-ota/pre-upgrade-flash.bin`. Its SHA-256 and the installed release
identity are recorded in [release evidence](../evidence/wifi-ota/README.md).
The current frozen application package is
`worktrees/balance-lower/artifacts/lowering-v10-fast/candidate/` from the project
root. The successful previous v9 image is retained at
`worktrees/balance-lower/artifacts/lowering-v9-catch/candidate/`, source `b9763c2`.
The successful earlier v7 image is retained at
`worktrees/balance-lower/artifacts/lowering-v7-ota/candidate/`, source `c442e12`.
The older exact v6 image is retained at
`worktrees/balance-lower/artifacts/lowering-v6-fast/candidate/`, source `a772ecc`.
The older exact v5 image is retained at
`worktrees/balance-lower/artifacts/lowering-v5-final/candidate/`.
The older exact v4 image is retained at
`worktrees/balance-lower/artifacts/lowering-v4-fast/candidate/`.
The earlier combined driving/v1-lowering application remains at
`worktrees/drive-braking/artifacts/drive-braking-v5/release/`; its driving was
reported good, but its lowering failed. The older Wi-Fi application at `artifacts/wifi-ota/release/` is the
known-good rollback package; other directories preserve earlier iterations. The full original backup
contains the pre-Wi-Fi driving-v4 firmware. Restoring it removes Wi-Fi/OTA and
reverts saved data to that backup's state.

After OTA, do **not** assume app0 is running: inspect `/api/info` for
`running_slot` or inspect OTA data over USB. The old
`scripts/flash_prepared_balance.py` intentionally programs only app0; it is not
a general recovery tool for a device booting app1. Do not overwrite partition,
NVS, OTA data or the filesystem just to update firmware. A functioning OTA
endpoint can install a known compatible application through the same verified
OTA procedure; download newer-schema logs before downgrading. If neither the
LAN nor recovery AP is usable, USB is required to diagnose boot and select a
recovery write appropriate to the actual partition/OTA state. Legacy
`scripts/upload.sh` and `scripts/flash_prepared_balance.py` are not the normal
update path. For a full restoration,
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

See [release evidence](../evidence/wifi-ota/README.md) for hardware checks. Initial validation is with
motor power off. A powered stationary check and supervised balance/drive trial
remain separate operator steps.
