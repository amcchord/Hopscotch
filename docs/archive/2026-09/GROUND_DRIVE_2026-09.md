# Flat-ground drive — September 19, 2026

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

**Integration update:** this feature is included in installed combined source
`43b1967`. See [current state](../../progress/CURRENT.md) and the [combined release
record](BALANCE_DRIVE_BRAKING_2026-09.md#installed-combined-release). The isolated
build/handoff details below are historical; physical evaluation remains pending.

## Behavior

CH1 steering and CH2 throttle (the existing default ground-drive mapping) can
drive all four wheels while the balance controller is Idle, including when CH7
is selected. Drive arming is still required; arm-motor arming is not required
for ground driving. Existing configurable ground channel mappings, deadband,
speed limits, CSP mixing and motor directions are preserved.

The previous control loop zeroed both ground inputs whenever CH7 was selected,
even before a stand-up was requested. The new gate follows actual controller
ownership instead:

- Pending CH11 single/double-tap detection stops ground input immediately.
- Stand-up and balancing retain exclusive wheel control.
- Arm return inhibits ground input regardless of CH7 position.
- After a canceled start, balance exit/abort, or radio-link loss, center both
  sticks before ground driving can resume. Centering during active balance
  does not satisfy this requirement; neutral must be observed afterward.

Standing-drive PID, pilot shaping, motor setup, arming, maintenance and radio
failsafe paths are unchanged. The gate runs only in the 50 Hz control task.

## Isolated implementation and handoff

Worktree: `worktrees/ground-drive`, branch `codex/ground-drive`, based on
`aa9ae13`. Changes are limited to `main.cpp`, `ground_drive_gate.h`, a native
regression test and its registration in `check_balance_candidate.sh`, plus this
handoff. The standing-drive task owns the next combined release in
`worktrees/drive-braking`. Shared root docs and other agents' checkouts were
not edited. No robot commands, uploads, or physical motion tests were performed.

## Validation

- Nine native test executables and 27 Python tests passed, including existing
  balance/startup/motor/CRSF/network regressions; Python/shell syntax and
  whitespace checks passed.
- The new native regression executes the real ground gate, drive controller,
  motor manager and CAN encoder. It checks four-wheel forward/reverse/steering,
  clipping and left-side reversal, neutral stop, unarmed suppression, pending
  start cancellation, balance ownership, arm return, hard abort, radio loss,
  and neutral handoff before resuming.
- Full PlatformIO build passed with the pinned environment/dependencies:
  53,288 bytes RAM and 1,187,389 bytes flash. The checkout uses example network
  credentials for compile validation only; its firmware image must not be
  deployed. Release integration must build with the release owner's own
  configured credentials and record the actual installed image identity.
- Local command evidence is in `output/validation.log` (native/Python/syntax)
  and `output/build.log` (successful full compile). The first build attempt used
  unsupported Python 3.14; the isolated PlatformIO environment was recreated
  with Python 3.13 and the pinned PlatformIO 6.1.19 before the successful build.

Next action: integrate implementation `b323220` and this validation record with
the standing-drive braking change and documentation commit `a1e24b9`, then run
the combined release checks. Physical ground-drive behavior remains unverified
until Austin tests the combined firmware.
