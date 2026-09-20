# Installed v9 — bounded loaded catch rebound

Verified 2026-09-20T22:38:58.351613+00:00: source `b9763c2f5e2d66dcc28da4b1bc568dac29751b17`, running app0.
[Manifest](manifest.json), [deployment](deployment.json),
[combined checks](combined-checks.txt), [source preservation](source-preservation.json).
Application 1,211,568 bytes, whole-file SHA-256
`6ca34f3ab58994b60c03e4db73b85540d5755909ba96fcb9aae3b1426d9cf3f8`, ESP digest
`361386bf17396fa11b7086ef272e813b49ee7a195c4d94bc1e38d19596ce98e8`. Private frozen package:
`worktrees/balance-lower/artifacts/lowering-v9-catch/candidate/`.
The manifest records preparation; the deployment proves installation.

## Physical finding and correction

The [662-row v8 run](../balance-lower/trial-v8-stop-20260920/README.md)
contains another user-confirmed successful fast lift. Lowering contacts both
arms but its confirmation repeatedly resets during a small loaded rebound.
It ends `lower_wrong_direction` before reaching the faster supported return.

V9 extends support confirmation through rates up to +20 degrees/s and a rise
of at most 1.5 degrees from the impact minimum, within the existing first-impact
300 ms window. Both recent independent arm loads, measured arm-reversal velocity
bounds, the full 80 ms dwell and elapsed time from both contacts remain required.
Supported target speed remains 0.24 rad/s, with its existing 0.30-rad/s motor cap
and body-rate pause. Fast standing, ordinary driving, wheel braking, initial
fall, final retraction and global/support-loss/progress/time guards are unchanged.
Schema9 adds metadata only; samples remain 240 bytes and old exports are retained.

## Checks and limits

12 native executables, 38 Python tests, syntax/whitespace and the configured
pinned build passed once after source freeze. Existing radio/dashboard/pinned
TCP checks are reused because their source is unchanged; the consolidated
checks include actual OTA lifecycle and helper tests. Native cases exercise the
recorded rebound and reject missing support, excessive rate/rise, late impact
and invalid arm velocities. Previous single-impulse/delayed-contact tests remain.

[Paired model](../balance-lower/bounded-catch-v9/README.md): 258/329 complete
before and after, no changed outcomes; 72/72 contact cases complete, 15/24 delayed
contacts complete, and 11 injected faults reject. The approximate model does
not reproduce the recorded transient. Replay confirms support at 13.011 s;
after commands diverge, unchanged old sensors still trigger support-loss
rejection at 13.210 s. That is not a prediction of the new physical trajectory.
The successful v7 recording yields identical v8/v9 commands. Manual acceptance
of v9 is pending.

## Deployment and recovery

The first transfer disconnected after 277,504 multipart bytes / 110.304 s.
Read-only recovery confirmed the old v8 image in app1, maintenance released
and all six motors healthy/disabled before retrying the identical frozen file.
[Failed attempt](first-attempt-deployment.json), [recovery](first-attempt-recovery-state.json).
The subsequent application-only OTA took 392.262 s. Exact image/slot, six powered motors
online/error-free/disabled, fresh IMU, both groups disarmed and released
maintenance verified. The 662-row schema8 CSV and wire are byte-identical
before and after. No autonomous motion or settings/filesystem writes occurred.
No other agent checkout was edited. OTA source/transport was unchanged; this
was a routine helper deployment, without repeated fault injection or monitoring.

Known physically successful v7 recovery remains at
`worktrees/balance-lower/artifacts/lowering-v7-ota/candidate/`, source `c442e12`,
ESP digest `5affbef55fc259591b2737cf1f17226b5878f5ac2ce847c7694919d755c975a4`.
Exact immediate v8 is also retained at `artifacts/lowering-v8-return/candidate/`
in this checkout. Follow the [OTA procedure](../../docs/WIFI_OTA.md), preserving
new logs before downgrading. No automatic boot rollback. Next: one manual
stand-up/CH11 attempt, disarm and archive before another run.
