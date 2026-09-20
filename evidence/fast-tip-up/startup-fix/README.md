# Fast startup refusal fix — September 20, 2026

Austin reports that CH6 HIGH did nothing while CH6 LOW still tipped up. Installed
source was `e55cecb` (release record `081c118`). The lowering owner supplied the
latest 1,237-row slow-start trace and disarmed snapshot for read-only inspection
in `worktrees/balance-lower`; it retains all device access.

The snapshot reports CH6=1792, confirming that HIGH reaches the receiver. The
recorded run has fast-run bit128 unset and begins at −2.319° with arm velocities
0.029/−0.010 rad/s and positions 3.928/3.545 rad, consistent with the usual
stationary Forward start. The snapshot itself is after the later fall, so its
178.8° tilt does not describe the attempted fast starting pose. No serial refusal
message or fast run was retained; the exact rejected predicate in Austin's
attempt cannot be proved from this recording.

## Reproduced defects

1. The fast starting gate ran immediately after two blocking wheel-mode switches.
   Motor feedback is processed by that same control task; synchronous parameter
   reads discard unrelated motion replies. A healthy pre-setup sample can exceed
   the new 100-ms freshness limit by the second gate, refusing before motion.
2. Fast mode required all six samples within 100 ms, but ordinary discovery has
   eight 20-ms slots (about 160 ms per motor). Tip-up bypasses normal front-wheel
   driving updates; rear zero-speed commands refresh only every 250 ms. The
   strict freshness requirement lacked sufficiently frequent explicit feedback.

These defects are consistent with the symptom. The original trajectory fixtures
bypassed motor setup and assumed continuously fresh feedback, missing this boundary.

## Correction

- Park the arms at their measured positions with zero speed and return to the
  normal control loop. Wait up to one second for fresh, stationary inputs,
  continuously qualified for 100 ms. Initialize the trajectory from the new
  measured positions. There is no catch-up jump.
- Request two motion samples per fast control tick, including parked startup.
  Each motor is requested about every 60 ms. The existing non-motion ping adds
  no synchronous receive wait and never fabricates timestamps. A failed request
  aborts; absent replies still fail the freshness requirement.
- Preserve the 100-ms freshness requirement, neutral RC/arm-switch checks, flat/
  Forward pose, 2.6-second trajectory, capture/tracking/motion limits and 4.5-second
  total logged fast-attempt timeout. The deterministic feedback fixture adds
  about 0.18 seconds of startup qualification; physical timing is unverified.
- Add `balance.start_status` to the bounded live snapshot and dashboard. Refusals
  identify feedback, flat pose, Forward arms, movement, RC/arm-switch/stick input,
  wheel setup or pending log storage. Saved schema/features (65535) and radio
  payload remain unchanged. Accepted pending attempts now produce state-1
  fast-run samples with stationary targets until the trajectory starts.

Slow motion arithmetic, normal balancing, ground/standing driving and lowering
policy remain unchanged. Network changes only expose the diagnostic field.

## Validation and handoff

[Validation](validation.json) and [checks](checks.txt) record regression and build
results. Tests reproduce stale setup rejection, wait for real returned samples,
verify quiet dwell/clock rollover, reject missing/wrong/moving inputs, and require
a deliberate retry after failure. The real MotorManager wrapper is checked for
mapped IDs, transmit failure and unchanged timestamps/targets. All 162 trajectory
cases remain passing. Dashboard syntax/SHA and the pinned configured build pass.

Task checkout: `worktrees/fast-tip-up`, branch `codex/fast-tip-up`, baseline
`081c118`. The lowering task owns next-OTA integration and shared CURRENT/JOURNAL.
No device request, arming, movement, settings write, upload, raw-log copying or
private-header copying occurred here. After integration, retry CH6 HIGH from
the normal flat Forward pose with neutral sticks; any refusal reason is visible
in the dashboard before another run.
