# V3 caught upright, then faulted before returning the arms

Austin reported that CH11 stopped with the robot upright on its arms, and one
CH12 press brought it flat. The [1,908-row CSV](../../../telemetry_logs/bal_20260920_forward_catch_v3_wifi.csv)
and [exact wire](../../../telemetry_logs/bal_20260920_forward_catch_v3_wifi.wire)
cover 38.235 seconds on installed `8449ddb` (flags 16383). Checksums and device
preflight are retained here. The end reason is `lower_wrong_direction`, **not
`lower_complete`**. The successful subsequent CH12 action is outside the log.

CH11 requested lowering at 24.426 s. Preparation began at 31.961 s and forward
commitment at 37.636 s / 91.921° tilt. The catch phase began at 38.155 s. At
38.215 s / 85.665°, first left load exceeded threshold (0.454 Nm), right load
was only 0.189 Nm, and arms still moved outward at 1.272 / 1.328 rad/s. V3
changed both targets toward Forward by 0.01 rad. The next frame still measured
outward speeds 1.232 / 1.295 rad/s, both arm loads about 0.92 Nm, and body rate
reversed from -11.405 to +45.776°/s. The catch faulted at 86.223° upright.

The [plot](impact-rebound.png) shows that a target reversal did not instantly
reverse the physical motors. Seeded at observed commitment, pinned v3 replay
matches subsequent phases and the fault exactly. Full preparation replay has
the recorded sampling limitation; see [analysis](analysis.json). There are no
samples of supported descent, ground hold, final Forward return or completion.

The v4 response is to continue returning after independent two-arm contact,
while allowing a short measured loaded rebound to settle. One-arm-only contact
retains a small retreat bound. All global motion/health/deadline checks and the
measured flat/Forward completion criteria remain. Simulation must include motor
response delay and acceleration limits; a command alone is not measured motion.
