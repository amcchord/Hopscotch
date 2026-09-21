# GX12 and Hopscotch telemetry — September 19, 2026

The recommended relationship is: **the radio requests an action; the robot owns
its execution and reports the result.** Keep steering, throttle, disarm, and the
existing triggers working independently of Lua. Use the screen to make actual
robot state legible, then add a named motion catalogue as the robot grows.

**Current integration:** the installed [Wi-Fi/OTA firmware](progress/CURRENT.md)
includes this structured radio payload along with progressive braking, ground drive and experimental supported lowering. Robot
updates now use [application OTA](WIFI_OTA.md#update-the-firmware), and complete
saved runs use the [Wi-Fi download workflow](WIFI_OTA.md#live-telemetry-and-saved-runs).
The Lua script is installed on the GX12, but actual RF forwarding/screen behavior
remains unverified. The configuration audit below is a September 19 snapshot;
Austin's subsequent radio adjustments may change its saved mappings.

## Download and configuration audit

The complete accessible radio storage was copied from `/Volumes/NO NAME` to
`/Users/austinmcchord/Development/Hopscotch/artifacts/radio-backup-2026-09-19/sd-card/`.
Its adjacent `manifest.json` records every path, size, and SHA-256. All **1,072
files / 36,959,971 bytes** were verified against the mounted source. Only macOS
indexing/journal/trash directories were excluded. Model/radio settings, historical
backups, Lua source/bytecode, firmware file, logs, sounds, and other storage files
are retained locally. The raw configuration is intentionally not in Git.

This is a filesystem backup, not a dump of the radio's running flash or the ELRS
module's private configuration. The firmware `.bin` on the card is only a stored
file; it does not establish the installed build. The GX12 exposes its internal
512 MB storage as this USB volume; its screen is a 128×64 monochrome OLED.
[RadioMaster specifications](https://www.radiomasterrc.com/products/gx12-dual-band-gemini-x-radio-controller).

Reviewed configuration:

- `RADIO/radio.yml`: board `gx12`, saved schema `2.11.0`, current model index 0;
  preserved stick/pot/slider calibration, switch hardware types, power, audio,
  display, and trainer settings. The `edgetx.sdcard.version` value **2.10** is the
  SD-content version, not proof that the running radio is EdgeTX 2.10.
- One model, `MODELS/model00.yml`, named `RM-GX12`, saved schema `2.11.0`.
  Internal Crossfire/ELRS, 16 channels, starting at CH1. Every mixer has 100%
  weight, zero offset, no switch condition, delay, or speed ramp. Four input
  lines use Ail/Ele/Thr/Rud with a zero-valued expo curve. No custom output
  limits, logical switches, special functions, or model Lua mixers are saved.
- 18 discovered telemetry sensors: `1RSS`, `2RSS`, `RQly`, `RSNR`, `ANT`, `RFMD`,
  `TPWR`, `TRSS`, `TQly`, `TSNR`, `Ptch`, `Roll`, `Yaw`, `RxBt`, `Curr`, `Capa`,
  `Bat%`, `FM`. Their logging flags are on; this alone does not start SD logging.
- One Values telemetry screen with transmitter/receiver voltage and pitch/roll.
  No telemetry Lua screen selected. The custom-mixer UI is disabled globally;
  telemetry is enabled. This does not require changing mixer configuration.
- SA/SD are configured as two-position switches; SB/SC/SE/SF as three-position;
  SG/SH as toggles. Six function-button settings and RGB values are saved, but
  those buttons are not mixer sources in the current model.
- Switch startup warnings are saved for SB/SC/SE/SF; no SA/SD startup warning is
  recorded. Keep the existing arming implementation unchanged in this change.
- TX battery range 6.4–8.4 V, warning 6.8 V, voltage correction -14. Speaker
  volume is 2 and `audioMuteEnable` is 1; verify audible alerts on hardware before
  depending on them. The new display uses modest haptic transition alerts.
- The February 11 and 13 model backups are byte-identical; their mixer sources
  match the current model. The radio backup differs in slider calibration,
  voltage correction, sound, timeout, view, and USB settings. Do not restore an
  older backup over the current calibration.
- Existing Lua includes ELRS tools, the model wizard, GPS Plus Code telemetry,
  and Snake; none is a robot-state dashboard.

### Saved radio mapping at the audit

Robot functions below describe the reviewed source/default settings, not a live
download of the robot's persisted channel map. Ground-drive channels and several
triggers can be remapped in robot settings; standing-drive CH1/2 and arm speed /
nudge CH5/4 are fixed in the current firmware. No robot settings were read or
written during this task.

| Channel | Radio source | Current robot use in source |
|---|---|---|
| 1 | Input Ail | Steering, including standing drive |
| 2 | Input Ele | Throttle, including standing drive |
| 3 | Input Thr | Not used by the current control loop |
| 4 | Input Rud | Arm nudge; **not a slider** |
| 5 | SE | Arm speed; **not a slider** |
| 6 | SB | Retained arm-mode/group setting; not used as a motion catalogue today |
| 7 | SC | Balance selection / trigger interpretation |
| 8 | SF | Not used by the current control loop |
| 9 | SA | Arms arm/disarm |
| 10 | SD | Drive arm/disarm |
| 11 | SG | Context-sensitive execute/calibrate/balance trigger |
| 12 | SH | Arm-position cycle; event marker while balancing |
| 13 | P1 | Retained left-arm setting; current sequential controller does not use it |
| 14 | P2 | Retained right-arm setting; current sequential controller does not use it |
| 15 | SL1 | Not used by the current control loop |
| 16 | SL2 | Not used by the current control loop |

The earlier README slider descriptions were inaccurate for this saved model;
the README now reflects this audit. Verify current channels after any radio
adjustment. Neither this audit nor the Lua script changes any mixer, input,
trim, switch, function button, channel, or trigger.

### What remains inside the RF module

Packet rate, telemetry ratio, switch mode, power/model-match configuration, and
actual ELRS version are not present in the SD YAML. ELRS stores RF settings per
receiver number in the module. The saved Crossfire arming mode is 0 with trigger
NONE; check how the installed ELRS build interprets arming (particularly CH5,
which is currently arm speed) before considering any changes. Do not infer full
16-channel resolution or telemetry bandwidth from `channelsCount: 16` alone.
[ELRS model configuration](https://www.expresslrs.org/software/model-config-match/).

## Implemented display and integrated firmware

`radio/SCRIPTS/TELEMETRY/hop.lua` is a receive-only EdgeTX telemetry script with
six pages, navigated using the roller / ENTER:

1. **Overview:** prominent robot status, separate drive/arm ON/OFF/WAIT,
   six-motor schematic, battery voltage, control link quality, current motion.
2. **Pose + power:** balance-filter tilt, error relative to its effective
   setpoint, summed motor IQ, highest fresh motor temperature, IMU/fault status.
3. **Motors:** each chassis role's enabled, disabled, fault, or lost status.
4. **Run + events:** last ended balance run's reason, plus recent observed
   state/arming changes. This is bounded session history, not a persistent log.
5. **Radio:** control/return link quality, RSSI, transmit power, TX battery.
6. **Diagnostics:** script version, accepted robot/total packet counts, status
   and sensor ages, optional SD recording controlled by a long ENTER press.

Without structured status, every Basic page has distinct content: overview
(FM/voltage), standard roll/pitch/power, explicitly unavailable motor details,
received run reports/recent FM changes, radio link, and diagnostics. Basic does
not establish that firmware is old: it means this script has not accepted the
structured packet, which may instead be a transport problem. `HS 0` on page 6
confirms no accepted robot-status packets in this script session.

The pre-integration firmware supplied basic fallback (FM and voltage), but
could not prove separate drive/arm state: its `DISARM` text was based only on
the arms. The installed combined firmware fixes FM to describe the whole robot
and adds structured status.
An active drive with disarmed arms is now `WHEELS READY`/`DRIVING`/`BRAKING`, not
`DISARM`. Motor-manager accepted arming state and individual motor feedback are
shown separately: `ON` is enabled, not proof that a wheel is physically moving.

The Lua never writes model data, emits a CRSF command, arms, or changes channels.
Each numeric reading and standard telemetry sensor keeps its last valid value
for five seconds after the last accepted fresh sample. An asterisk marks a
reading held longer than 0.5 seconds. Missing, stale, invalid,
or unrelated updates do not refresh that value's hold. Standard telemetry uses
EdgeTX's individual current/fresh flags, not cached `getValue()` results.
Sampling runs every 50 ms. The previous 200 ms poll could completely miss the
160–320 ms `isFresh()` window in EdgeTX 2.11; a regression test reproduces that
phase alignment using unchanged sensor values and background callbacks.
[EdgeTX 2.11 freshness implementation](https://github.com/EdgeTX/edgetx/blob/v2.11.0/radio/src/telemetry/telemetry_sensors.h).
After 1.5 seconds without a new valid status sequence the header says **HOLD**;
after three seconds it hides motor / arming / motion values and says
**ROBOT DATA LOST / UNKNOWN**. Duplicate status or run-detail packets cannot
extend this hold. New arming/fault/IMU-stale flags appear immediately, even while
the associated last numeric reading is held. Existing loss alerts stay at 1.5 seconds.
An unsupported schema asks for a script update. Haptic alerts are transition
based and limited to one per five seconds; they do not replace robot failsafes.
[EdgeTX source freshness API](https://luadoc.edgetx.org/lua-api-reference/variables/getsourcevalue).

Do not display battery percentage, remaining runtime, or consumed mAh: existing
firmware sends zero placeholders for capacity and percentage. `Curr` is the sum
of absolute motor IQ measurements, not battery input current. The dashboard
labels it **MOTOR IQ**. Voltage expires after two seconds without a valid VBUS
reply at the sender; the Lua then applies its five-second display hold. Current
requires fresh replies from all six online motors. Tilt is the
balance controller's actual filtered angle, not a presumed chassis orientation.
Basic roll/pitch instead use standard CRSF attitude, converting the sensor's
radian unit to degrees. They are not the controller's filtered balance tilt.

### Optional SD diagnostics

Logging starts OFF on every script load. On page 6, hold ENTER to start/stop.
The script appends one CSV row per second to `/LOGS/hop-<date-time-tick>.csv`,
closing the file each time. It stops at 600 total rows per script load, or on a
reported I/O error. Pause/resume does not reset the cap or truncate existing data.
The LOGS directory must already exist; the update installer checks it.

Rows contain radio uptime, page, received/custom/accepted/duplicate packet
counts, sequence/status age, maximum sampling gap, robot flags/masks, and each
standard sensor's last accepted value, age, current/fresh flags, and whether the
value is still displayed. This diagnoses polling gaps and packet delivery; it
does not replace the robot's high-rate onboard motion logs. Recording works in
the script's background callback while EdgeTX schedules it, and cannot capture
packets the radio never received. Hardware storage latency/runtime remain to be
checked on the GX12. A diagnostic CSV is not proof of control-loop timing.

EdgeTX provides a restricted `io.open/write/close` API; the host tests emulate
its actual calling convention and no-return `close`. EdgeTX's separate built-in
**SD Logs** function can also record configured sensors and radio controls;
this update does not alter that function or any model configuration.
[Lua file I/O](https://luadoc.edgetx.org/overview/version-libraries/io-library),
[SD Logs](https://manual.edgetx.org/bw-radios/model-select/special-functions).

### Wire contract v1

The experimental project-private extended CRSF type is **0x7E**, destination
0xEA (handset), origin 0xC8 (FC), signature `HS`. **This is not an allocated TBS
message type.** Both ends validate the signature/version. Registration or a
standard application envelope should replace it before distribution beyond
this robot. Avoid types reserved for ArduPilot or Rotorflight.

Compatibility inference: ExpressLRS 3.5.5 accepts extended messages originating
at the FC into its generic telemetry slots; EdgeTX 2.11 forwards unrecognized
CRSF types into Lua telemetry queues. This was checked in upstream source,
**not verified on the attached RF hardware**. The installed module versions and
an over-air bench test remain release gates.
[ELRS forwarding](https://github.com/ExpressLRS/ExpressLRS/blob/3.5.5/src/lib/Telemetry/telemetry.cpp),
[EdgeTX parser](https://github.com/EdgeTX/edgetx/blob/v2.11.0/radio/src/telemetry/crossfire.cpp),
[CRSF specification](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md).

All multibyte fields are big-endian. Offsets are zero-based within the Lua data
payload, including destination and origin. CRSF adds sync, length, type, and CRC.

| Bytes | Field |
|---|---|
| 0–1 | Destination EA, origin C8 |
| 2–3 | ASCII HS |
| 4–5 | Schema 1, kind 1 (status) or 2 (run detail) |
| 6–7 | Incrementing packet sequence; duplicates do not refresh status |
| 8–9 | Status flags |
| 10–11 | Mode and phase |
| 12–13 | Stable motion ID |
| 14–16 | Online, fault, enabled motor masks (FR, BR, BL, FL, LA, RA) |
| 17 | Maximum fresh motor temperature, C; 255 unknown |
| 18–21 | Signed tilt and setpoint error, centidegrees; -32768 unknown |
| 22–25 | Voltage and summed motor IQ, tenths; freshness flags required |
| 26–29 | Balance inner fault bits, capability bits |
| 30–31 | Calibration step, progress percent (255 unknown) |
| 32–47 | ASCII motion/status label, up to 16 bytes |

Flags bits 0–11: drive armed, arms armed, drive arming, arms arming, robot RC
linked, arms moving, rearm required, saving log, IMU fresh, simulation/test,
voltage fresh, current fresh. Capabilities 0–2: status, run detail, motion IDs.
Modes: 0 idle/disarmed, 1 ground, 2 balance, 3 calibration. Phases: 0 idle/hold,
1 arm movement, 2 arming, 3 tipping up, 4 balancing, 5 returning arms,
6 calibration in mode 3 or supported lowering in mode 2. Motion IDs: 0 none, 1 forward, 2 center, 3 backward, 4 jump,
0x0100 balance. Unknown modes/IDs display numerically without guessing behavior.

Kind 2 uses the same eight-byte prefix, then a four-byte run-end uptime and
48-byte ASCII reason. It reports the current boot's last logged run end, not a
complete fault history. It does not extend status freshness. Standard telemetry
continues at 5 Hz; one custom packet is attempted each 200 ms: four 52-byte status
frames and one 64-byte detail frame per second (272 added serial bytes/second).
Radio airtime capacity must be measured separately. Fixed buffers and a UART
space check keep transmission from waiting on a full FIFO; whole frames drop
instead of delaying control. Existing control loops and mappings are unchanged.

## Growth plan

1. **Finish the feedback loop first.** Bench-test actual ELRS settings, packet
   delivery, both arming groups, stale-data indication, fault visibility, and
   screen performance while disarmed. Evaluate any warning threshold against
   the battery chemistry and motor limits; no invented battery-% calibration.
2. **Give motions stable identities.** A robot-owned catalogue should provide
   `id`, name, category, supported parameters, preconditions, and capability
   version. Keep IDs stable even when the visible menu order changes. The
   current motion/phase/progress/label fields are the first step, not a completed
   catalogue implementation.
3. **Separate selected, requested, and active.** Display the selection before
   execution; then accepted/rejected, precondition or rejection reason, phase,
   progress if meaningful, completion, cancellation, and last stop reason.
   Add command IDs, acknowledgements, timeout, and deduplication before any future
   bidirectional command protocol. A lost/repeated packet must never repeat a trick.
4. **Use controls consistently later.** Consider the six function keys for
   mode families or favourites; keep fixed execute/cancel and emergency disarm.
   Use the screen/roller for discovery and contextual explanations. Existing
   unused channels are options, not invitations to remap them now. Do not make
   high-rate steering or disarm depend on script scheduling or menu focus.
5. **Add robot-confirmed feedback.** Haptic/voice for accepted mode, rejected
   trick, fault, and link loss; LEDs should represent confirmed states rather
   than only the last local button press. First verify GX12 LED API support and
   audio settings. Preserve complete onboard logs for tuning; send summaries
   and event markers over RF, not 200 Hz debug streams.

## Installation, verification, and rollback

`hop.lua` is installed on the radio and selected in
telemetry screen 2 (index 1). Original Values screen and all mappings are intact.
The SD files passed readback verification and the volume was safely ejected.
The combined robot firmware was subsequently installed and bench-tested for
Wi-Fi/OTA with motor power off. See [current state](progress/CURRENT.md) for
source/image identity. RF display verification remains outstanding.

For future display or robot updates (SD installation and firmware integration
are already complete):

1. Preserve a fresh model/radio backup if either has changed since this audit.
2. Copy `radio/SCRIPTS/TELEMETRY/hop.lua` to `/SCRIPTS/TELEMETRY/hop.lua` on the
   radio. Safely eject USB storage. Do not install host-compiled `.luac` files.
3. On the radio's model Display page, select **Script → hop** in a spare
   telemetry screen. Keep the existing Values screen. Do not edit mixers,
   channels, switch functions, or RF parameters. Long RTN exits telemetry.
4. Update the robot only if its firmware needs changing, using the
   [OTA procedure](WIFI_OTA.md#update-the-firmware). The current release already
   includes full structured status. Robot OTA does not copy Lua files to the
   handset. Do not upload LittleFS or reset settings/calibration.
5. First verify with both motor groups disarmed. Check firmware identity,
   module versions/settings, matching voltage, all six motor roles, live/stale
   transitions, page navigation, script memory/CPU, and no new control timing
   problems. Receive-only link-loss tests must not require moving the robot.
   Actual arm/motion verification belongs to a separately supervised physical
   test, not this preparation task.

Radio rollback: remove the added telemetry-screen selection and `hop.lua` /
radio-generated `hop.luac`; the original Values screen still works. Restore the
backed-up model file only if needed and only after comparing newer user changes.
Robot recovery: follow the [Wi-Fi/OTA recovery guide](WIFI_OTA.md#recovery),
preserving calibration/settings and downloading the saved run first. The active
slot can change after every OTA. Restoring pre-Wi-Fi firmware removes its OTA
service; do not use historical app0-only commands as a generic recovery method.

**GX12 runtime correction (v3.1):** the v3 installation crashed on the first FM
event because `table.insert` is unavailable on monochrome EdgeTX. Its logger
also used unavailable `table.concat`. Both paths now use core Lua operations.
The complete display and logger tests load production Lua in an isolated,
restricted environment rather than inheriting desktop libraries. This also
removes access to os/package/debug/coroutine and limits library/API exposure.
The pre-fix script reproduces the photographed error in this environment.
Both suites pass on desktop Lua and Lua 5.3.6 configured with 32-bit numeric
types, matching EdgeTX 2.11's vendored version/configuration. Full handset
runtime, heap and scheduler verification still requires the physical radio.
Use `HOP_LUA_BIN` and `HOP_LUAC_BIN` to select a compatible local interpreter and
compiler for the check script; never copy the host compiler's bytecode to SD.
[Library availability](https://luadoc.edgetx.org/overview/version-libraries),
[EdgeTX 2.11 Lua configuration](https://github.com/EdgeTX/edgetx/blob/v2.11.0/radio/src/thirdparty/Lua/src/luaconf.h).

Host validation: `bash scripts/check_radio_telemetry.sh`, the existing consolidated
balance checks, and PlatformIO build. The new tests feed actual C++-encoded
payloads to the production Lua, exercise malformed/stale/duplicate/unknown
messages, queue limits, UART pressure, motor-power freshness, navigation, and
haptic transitions. The preview is rendered from production Lua drawing calls
with EdgeTX fonts; it is not a physical-radio screenshot or an EdgeTX simulator
run. Hardware RF compatibility, Lua scheduling/memory, and electrical timing
remain unverified until the bench test.
