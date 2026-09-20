# Physical v6 trial — waiting drive latch and rocking arm contact

[Archive](archive.json), [fresh installed identity/state](preflight.json),
[analysis and fixed-input replay](analysis.json), [reproduction](analyze.py).
The 1,519-row / 30.370-second trace uses installed `a772ecc`, schema 6, and fast
bit 128. Original CSV/wire remain in this checkout's `telemetry_logs/` as
`bal_20260920_forward_catch_v6_wifi.*`. The first fast lift succeeded; its owner's
separate evidence describes the startup portion.

CH11 is accepted at 16.745 s. Stopping lasts 12.525 s. The pilot's moving latch
keeps the acceleration driving controller active while waiting for its stricter
calm dwell, even though velocity/turn references and sticks are zero. Body sway
and wheel ripple repeatedly break the separate lowering dwell. Before 27.5 s,
no quiet interval exceeds 260 ms, shorter than the required 500 ms. The operator
reports manually settling the robot. Preparation starts at 29.270 s, then the
v6 early handoff works at 29.850 s with body 86.190° and wheels 3.123/3.153 rad/s.

Both arms load at 30.030 s (−0.489/+0.453 Nm), body rate slowing from −38.775 to
−19.615°/s. The targets reverse toward Forward. Contact then rocks: the body
briefly rebounds near +9°/s and either arm's instantaneous torque drops below
0.2 Nm. The old continuous two-arm-load / body-rate≤4°/s confirmation never
qualifies. Wheels continue at the captured 3.138 rad/s and no Descending samples
occur. The return pauses during renewed forward fall; a second rebound to
+20.305°/s ends `lower_wrong_direction` at 30.370 s.

V7 hands the pilot's zero references back through the existing stationary-PD
slew while still requiring 500 ms of independent lower calm. First independently
observed contact on both arms starts the bounded wheel stop. Support qualification
uses each arm's observed load within 60 ms, an 80 ms dwell plus at least 80 ms
of real elapsed time since both contacts, and body rate −12..+12°/s. A single
impulse, even with stationary arms, cannot qualify. Motion/load/freshness limits
and measured flat/Forward completion remain.

The installed policy replay reproduces the original fault. Supplying the same
recorded sensors to v7 enters Descending at 30.110 s, but then faults
`lower_support_lost` at 30.310 s during the old trace's second fall. This is a
**fixed-input regression, not a prediction**: v7 starts braking wheels at
30.030 s, and its earlier drive handoff also changes the physical trajectory.
We do not relax the support-loss limit to make an old sensor trace pass. The
[separate model](../supported-return-v7/README.md) and a new physical trial are
needed to evaluate the changed motion.
