# Physical v5 trial — preparation abort before catch

Installed source `17c499c`, downloaded directly from the robot after the operator
reported reliable forward departure followed by a stop supported on the arms.
[Validated archive](archive.json), [device state](preflight.json),
[analysis and replay](analysis.json), [reproduction script](analyze.py).

The retained run has 1,219 samples / 24.365 seconds, schema 5, features 65535.
Fast-standing bit 128 is absent. Its CSV and original wire are retained in
`telemetry_logs/bal_20260920_forward_catch_v5_wifi.*` at this checkout's root.

Preparation starts at 23.746 s with body tilt 87.885°. The effective setpoint
stays at or below that ceiling. The body falls forward to 85.119° over 619 ms,
while the upright wheel controller continues trying to recover that target.
At 24.365 s the right rear wheel measures 6.094 rad/s, crossing the unchanged
6 rad/s preparation limit. The left measures 2.486 rad/s. Both arms finally
reach the old preparation tolerance in that same frame, but the safety check
runs first and aborts with `lower_prepare_disturbed`.

There are no committed, catching or supported-descent samples. The saved log
ends before the operator's observed contact/support; it does not prove that the
catch logic recognized contact and stopped. This is why the existing continuous
arm-return logic never became active in this recording.

Pinned v5 production-policy replay reproduces the exact final reason and time.
The v6 candidate releases upright control at the earlier 24.326 s frame: both
arms have traveled at least 1.6 rad and are still advancing, the body has fallen
2.164°, and rear wheels measure 1.695/4.490 rad/s. It retains their mean command
3.0925 rad/s and proceeds into the existing catch/return sequence. Replay stops
at this divergence; later recorded values cannot validate counterfactual motion.
The [paired physical model](../early-handoff-v6/README.md) is a separate,
approximate screen, not proof of the eventual catch.
