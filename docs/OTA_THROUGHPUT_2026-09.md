# Strongest AP selection and OTA throughput

Candidate on `codex/ota-throughput`, based on verified v10 record `d5f18fa`
(installed source `7917543`). No robot requests, upload, reconnect or radio
configuration changes were made for this investigation. The lowering task
retains device and release ownership.

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

The fast host profile works with the existing receiver, but hardware acceptance
is pending. Strongest-AP selection and receiver timing headers take effect only
after firmware containing this candidate boots. Compare AP/channel/RSSI,
transmitter state, transfer duration, host sleeps/socket stalls and receiver
write/verification timing on subsequent normal uploads. Record the actual
speedup, if any, rather than assuming the removed pacing time translates
directly into the same wall-clock reduction.
