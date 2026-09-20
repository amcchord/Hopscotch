# Hopscotch radio display

Copy `SCRIPTS/TELEMETRY/hop.lua` to the matching radio-storage path after reviewing
the installation instructions in [the radio audit and design](../docs/RADIO_TELEMETRY.md).
Select `hop` in a spare model telemetry screen. Existing button/channel mappings
and the original Values screen remain intact.

Target: GX12, 128×64 monochrome, EdgeTX 2.11 API. Roller / ENTER advances pages;
long RTN leaves telemetry. There are no command, mixer, or settings writes.
Haptics can be disabled by setting `HAPTIC = false` near the top of the script.

Readings hold their last valid value for five seconds to bridge intermittent
telemetry. Robot status shows HOLD after 1.5 seconds without a new packet and
UNKNOWN after three seconds. Received fault/arming changes remain immediate.
An asterisk marks a held reading. Sampling every 50 ms avoids missing EdgeTX's
short fresh-value window. All six pages have distinct Basic content; motor
details stay unknown until structured robot status actually arrives.
When updating, remove the radio-generated `hop.luac` so the radio recompiles the
new `hop.lua`; never install bytecode compiled on the computer.

Page 6 is Diagnostics. Logging starts OFF; hold ENTER there to toggle a 1 Hz
CSV recording under the existing `/LOGS` directory. It records sensor freshness,
ages and packet counts, stops after 600 rows per script load, and stops on I/O
errors. Files are appended and closed after each row. Reboot/reload leaves
logging off again. The update needs only this Lua file; no model edits.

Basic telemetry uses standard sensors (FM, attitude and power, arming unknown).
Full pages require receipt of matching structured robot packets. Basic alone
does not identify the firmware version; the RF transport may be the missing
piece. No firmware upload is part of copying this script.

Protocol and future motion-catalogue design: [RADIO_TELEMETRY.md](../docs/RADIO_TELEMETRY.md).
Host tests: `bash scripts/check_radio_telemetry.sh`.
