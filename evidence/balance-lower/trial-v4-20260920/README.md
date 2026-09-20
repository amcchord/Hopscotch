# Physical v4 lowering trial — September 20, 2026

Retrieved the latest saved run from installed e55cecb while disarmed. Only the
latest run is stored on the robot; Austin reports several attempts.
[Exact CSV](../../../telemetry_logs/bal_20260920_forward_catch_v4_wifi.csv)
and [wire](../../../telemetry_logs/bal_20260920_forward_catch_v4_wifi.wire)
validate at 1,237 rows / 24.745 seconds. [Hashes](archive.json),
[preflight](preflight.json), [analysis](analysis.json), [plot](impact-rebound.png).

At 17.925 s preparation begins with body 86.093 degrees and base 85.984 degrees.
At 24.125 s departure starts with body 90.556 degrees and base 90.449 degrees:
**+4.463 degrees measured backward lean and +4.465 degrees scheduled target**.
The old 1.25 rad arm sweep took 6.2 seconds under ordinary balance scheduling.

The arms first show strong load at 24.725 s: body 83.577 degrees, rate +35.981 deg/s,
arm reach 1.899/1.889 rad, torque -1.026/+1.011 Nm. Their targets reverse, but the
next 20 ms sample still shows outward arm motion1.493/1.437 rad/s and body rate
**+105.041 deg/s**. The 65 deg/s motion limit ends the run. No supported descent
or successful completion is recorded. Slow standing was used (fast-run bit 128
is clear). The user's backward flip is consistent with this trace.

Larger body tilt means backward. A logged setpoint of 0 during state Lowering is
an inactive-field convention, not a request to instantly pitch flat. Forward
arm references are inferred from the settled targets 3.928/3.545 rad. The plot
and analysis describe measurements; they are not a counterfactual simulation.
