# Strongest AP selection and OTA throughput

This feature first shipped as source `45c1a94` on `codex/ota-throughput`, based on v10
record `d5f18fa` (motion source `7917543`). The initial investigation was offline;
Austin subsequently authorized deployment. [Deployment evidence](../evidence/ota-throughput-deployment/README.md)
records exact identity, the host-timeout fix, successful retry and stronger AP
selected during a safe reconnect. Device ownership has returned to the lowering
task, which integrated the changes and preserves them in later motion releases.

## Findings

The pinned Arduino ESP32 2.0.16 `WiFiSTAClass::_scanMethod` defaults to
`WIFI_FAST_SCAN`; `begin()` passes that value into the station configuration.
Although the sort default is signal strength, sorting is only used after an
all-channel scan. The existing firmware never overrides the fast-scan default,
so it can associate with the first matching SSID rather than the strongest.
[Espressif's versioned station configuration reference](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32s3/api-guides/wifi.html#station-basic-configuration)
documents this distinction and fallback when the strongest AP cannot connect.

The host helper sends 1 KiB then sleeps 50 ms. That imposes a 20 KiB/s ceiling
and approximately 59 seconds of scheduled sleeps for current 1.2 MB images,
before socket, network or flash delays. Existing results are much slower:

| Release | Transfer | Application KiB/s | RC linked before/after | Preflight RSSI |
|---|---:|---:|---|---:|
| v7 | 421.475 s | 2.81 | yes/yes | −55 dBm |
| v8 | 475.235 s | 2.49 | yes/yes | −64 dBm |
| v9 retry | 392.262 s | 3.02 | yes/yes | −53 dBm |
| v10 | 90.960 s | 13.02 | no/no | −53 dBm |

These are different firmware uploads and RF conditions, not a controlled A/B
test. They show that host pacing alone cannot explain the slowest transfers.
They do not establish transmitter interference or prove a weak AP caused the
slowdowns. The old records lack BSSID, channel and separate flash timings.
Removing the receiver UART workload does not turn off the transmitter's RF.
The v9 first attempt also disconnected after 277,504 multipart bytes; the cause
remains unconfirmed. [Derived data](../evidence/ota-throughput/baseline-rates.json)
points to the original deployment records.

Wi-Fi power saving is already disabled with `WiFi.setSleep(false)`. Both ends
already use TCP_NODELAY for eligible uploads, WebSocket offers pause during
OTA, and Update already buffers 4 KiB writes and uses aligned 64 KiB erases.
There is no identified need to alter the flash implementation or motion-task
priorities. Changing those without timing evidence would obscure the diagnosis.

## Candidate changes

- Before the first association, explicitly select `WIFI_ALL_CHANNEL_SCAN` and
  `WIFI_CONNECT_AP_BY_SIGNAL`. Arduino retains both policies for later calls to
  `begin()`, including manual and recovery reconnects. The strongest compatible
  AP with the configured SSID is preferred; credentials/security thresholds
  remain unchanged. Association can take longer because it scans all channels.
- Retain control-owned maintenance for every scan/association. No periodic
  roaming or additional scan is started during motion or upload. An already
  connected robot reselects an AP on its next safe reconnect or boot.
- Add BSSID and channel beside RSSI in existing Wi-Fi telemetry, so normal
  pre/postflight records identify the AP without polling during an upload.
- Add optional `--upload-profile fast`: 16 KiB socket sends with no artificial
  sleep, allowing TCP backpressure to regulate delivery. The existing 1 KiB /
  50 ms `paced` profile remains the default until a hardware comparison. Both
  profiles retain image validation, saved-log backup, disarmed preflight,
  verification after ambiguous responses, and exact image/health/log checks.
  There is no automatic retry or automatic fallback upload.
- Measure host time inside socket sends, the longest send, pacing sleep and
  final response wait. A completed `send()` means accepted by the local TCP
  stack, not confirmed written on the ESP; socket buffering limits inference.
  The first live fast attempt exposed a separate timeout problem: the computer
  queued the image in about 11 seconds, timed out waiting for the response and
  closed while the robot was still receiving. The robot eventually aborted at
  1,082,590 bytes and safely retained v10. The helper now allows 900 seconds for
  the final response, while keeping the 120-second connect/send timeout. It
  never reuses a timed-out HTTP response reader or resends automatically.
  Local regression coverage includes a delayed receiver acknowledgment and
  single-close timeout handling. Firmware bytes are unchanged by this fix.
- Measure receiver Update.write call count, cumulative/max write time, longest
  gap between upload callbacks (excluding prior write/hash work), and final
  verification time. Counters remain in live telemetry on failure. Successful
  responses include `X-OTA-*` timing headers, saved as `server_timings` before
  reboot clears RAM. Old firmware/clients can omit/ignore these headers.
  Receive gaps can include parsing/scheduling and transport delays, not only RF.

## Next authorized release

The release owner can select the fast profile for a normal, already-authorized
application update using the same frozen image and manifest:

```bash
python3 scripts/robot_wifi.py ota path/to/firmware.bin \
  --manifest path/to/manifest.json --upload-profile fast
```

If the acknowledgment is lost, let the helper verify the running image before
considering another attempt. A later explicit retry can use `--upload-profile
paced`; the helper does not automatically resend bytes. No extra reflash or
in-flight stress monitoring is needed just to collect diagnostics.

The fast host profile completed a 305.815-second transfer after correcting the
host response timeout. That transfer still ran on the old receiver; it is not
evidence of a new-AP throughput improvement. Strongest-AP selection and receiver
timing headers take effect only after the new firmware boots. Compare AP/channel/RSSI,
transmitter state, transfer duration, host sleeps/socket stalls and receiver
write/verification timing on subsequent normal uploads. Record the actual
speedup, if any, rather than assuming the removed pacing time translates
directly into the same wall-clock reduction.

## Observed deployment outcome

The successful transfer averaged 3.880 KiB/s with RC linked and no host sleeps.
The first post-boot association was approximately -70 dBm on channel 6. A fresh
disarmed reconnect selected a different BSSID on channel 1 and improved -69 to
-55 dBm. A scan can miss an AP or encounter changing conditions, so strongest
signal is a connection preference, not a guarantee of permanent best coverage.
The next normal update can measure this connection and collect receiver timings.
The physical percentage display was confirmed by Austin; no motion was initiated.

## First receiver timings from a later regular update

The lowering owner supplied the next normal authorized deployment, v12 source
`09d2e01b6ef89a357319a8eeea2fa11db8221932`, recorded at commit `5b1b545` in
`worktrees/balance-lower/evidence/lowering-v12-integration/deployment.json`.
That record was read in place; no raw log or private package was copied, and no
robot requests, scans or extra uploads were made for this analysis. The ordinary
update completed with HTTP 200 and verified image, health, RC and log retention.

| Measurement | Observed value |
|---|---:|
| Application / host transfer duration | 1,215,936 bytes / 438.820 s |
| Application throughput | 2.706 KiB/s |
| Receiver elapsed | 438.622 s |
| Cumulative flash write time | 5.214650 s |
| Longest flash write | 0.154555 s |
| Final verification | 0.149445 s |
| Longest gap between upload callbacks | 13.487 s |
| Host time blocked in socket sends | 405.642140 s |
| Longest individual socket send | 28.327570 s |
| Final response wait / host pacing sleeps | 33.046991 s / 0 s |

The same channel-1 AP (`8C:30:66:7A:2E:DD`) was present before and after, at
-57 / -56 dBm; RC was linked before and after. This transfer used the new AP
selection and receiver timing code, unlike the initial installation above.

Measured flash writing plus final verification accounts for only 5.364095 s,
about 1.22% of receiver elapsed time. Approximately 433.258 s lies outside
those operations. Optimizing those flash calls alone therefore cannot explain
or remove most of this delay. The long receive gaps and blocked socket sends
point investigation toward transport delivery and receiver scheduling/parsing;
these measurements do not isolate RF interference, TCP retransmissions, flow
control, or other receiver work. Chunk hashing is outside the flash-write timer.

Stronger association and removal of artificial host sleeps have not resolved
the observed slowdown. This remains an uncontrolled observation; it does not
prove RC interference or that the strongest-AP policy made uploads slower.
Keep the flash implementation and control priorities unchanged on this evidence.
Useful next measurements are TCP retransmission/receive-window behavior during
the next already-authorized update, alongside AP and transmitter conditions.
No diagnostic reflash is needed to collect the existing receiver timings.
