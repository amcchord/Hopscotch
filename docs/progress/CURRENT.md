# Current state — September 21, 2026

**Development baseline:** the root checkout / GitHub `main` consolidates all ten
former development branches. ESP32 source, dashboard and build configuration
match the latest `codex/balance-lower` checkpoint; the GX12 Lua v3.1 fixes are
also integrated. The [journal](JOURNAL.md) records consolidation and validation.

**Last verified robot:** lowering v13 / fast stand-up v2, source
`7c32bd7a6e48e641345a7fbc067a11d46a0643bf`, app0, telemetry schema 13.
[Release identity and checks](../../evidence/lowering-v13-integration/README.md).
This is archived installation evidence, not a live status check. Consolidation
did not contact, flash, reset or move the robot.

**Latest trials:** fast stand-up failed twice (`bailout_angle_error`). The later
slow comparison settled upright but subsequently failed lowering
(`lower_wrong_direction`). [Latest captures and unfinished analysis](../../evidence/balance-recovery/repeat-20260921/README.md)
are now committed. Earlier successes do not establish reliability. V13 fast
lowering completion remains unverified.

**Next engineering action:** review that comparison and the
[arm/wheel recovery investigation](../../evidence/balance-recovery/rollaway-20260921/README.md),
then develop a bounded recovery prototype that accounts for measured arm-return
handoff, wheel headroom, static arm COM shift and uncertain inertial reaction.
Screen against both successful and failed trials. Prior recovery experiments
regressed cases; none is accepted for upload. Do not reset trim based only on
the failed run. New robot operations require task-specific authorization.

**Other open checks:** OTA works with the transmitter linked but remains slow;
[timing analysis](../archive/2026-09/OTA_THROUGHPUT_2026-09.md) does not isolate the
transport cause. Lua v3.1 was verified on GX12 storage; handset boot/runtime and
structured RF display still need confirmation ([installation record](../archive/2026-09/RADIO_HANDOFF.md)).

**Recovery and ownership:** no parallel checkout retains device ownership after
consolidation. Coordinate device access explicitly for the next hardware task.
Use [OTA/recovery](../WIFI_OTA.md); never `uploadfs`. Private packages and backups
are indexed in local `artifacts/README.md`, including installed v13 and the last
physically successful `45c1a94` package. Historical worktree paths were retired;
use the index and verify manifest/image hashes before reuse.

**Security follow-up:** the Wi-Fi password appears in already-published historical
Git documentation (removed from current files). Coordinate credential rotation;
[details](../archive/2026-09/CONSOLIDATION.md#existing-credential-history).
