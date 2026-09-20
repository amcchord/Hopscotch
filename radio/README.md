# Hopscotch radio display

Copy `SCRIPTS/TELEMETRY/hop.lua` to the matching radio-storage path after reviewing
the installation instructions in [the radio audit and design](../docs/RADIO_TELEMETRY.md).
Select `hop` in a spare model telemetry screen. Existing button/channel mappings
and the original Values screen remain intact.

Target: GX12, 128×64 monochrome, EdgeTX 2.11 API. Roller / ENTER advances pages;
long RTN leaves telemetry. There are no command, mixer, or settings writes.
Haptics can be disabled by setting `HAPTIC = false` near the top of the script.

Readings hold their last valid value for three seconds to bridge intermittent
telemetry. Robot status shows HOLD after 1.5 seconds without a new packet and
UNKNOWN after three seconds. Received fault/arming changes remain immediate.
When updating, remove the radio-generated `hop.luac` so the radio recompiles the
new `hop.lua`; never install bytecode compiled on the computer.

Existing robot firmware gives Basic telemetry (FM and voltage, arming unknown).
The matching firmware candidate adds separate arming, motor health, pose,
motion, and last-run status. Firmware changes must be integrated with current
balance work before an authorized flash. No firmware upload is part of copying
this script. Do not distribute a host-compiled Lua bytecode file.

Protocol and future motion-catalogue design: [RADIO_TELEMETRY.md](../docs/RADIO_TELEMETRY.md).
Host tests: `bash scripts/check_radio_telemetry.sh`.
