# Hopscotch radio display

Copy `SCRIPTS/TELEMETRY/hop.lua` to the matching radio-storage path after reviewing
the installation instructions in [the radio audit and design](../docs/RADIO_TELEMETRY.md).
Select `hop` in a spare model telemetry screen. Existing button/channel mappings
and the original Values screen remain intact.

Target: GX12, 128×64 monochrome, EdgeTX 2.11 API. Roller / ENTER advances pages;
long RTN leaves telemetry. There are no command, mixer, or settings writes.
Haptics can be disabled by setting `HAPTIC = false` near the top of the script.

The installed [Wi-Fi/OTA robot release](../docs/progress/CURRENT.md) includes
structured radio telemetry for separate arming, motor health, pose, motion and
last-run status. The Lua script was installed on the GX12; its actual over-air
display and RF compatibility still need a disarmed hardware check. Basic
FM/voltage fallback remains available for older robot firmware.

Update the **robot** with the [OTA procedure](../docs/WIFI_OTA.md#update-the-firmware).
Copying this script updates only the **radio display**, and robot OTA does not
update the radio's storage. Do not distribute host-compiled Lua bytecode.
For live LAN monitoring and complete saved balance logs, use the
[Wi-Fi dashboard and download helper](../docs/WIFI_OTA.md#live-telemetry-and-saved-runs).

Protocol and future motion-catalogue design: [RADIO_TELEMETRY.md](../docs/RADIO_TELEMETRY.md).
Host tests: `bash scripts/check_radio_telemetry.sh`.
