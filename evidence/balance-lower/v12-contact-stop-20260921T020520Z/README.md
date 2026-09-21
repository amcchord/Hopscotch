# V12 contact stop: torque veto deadlock

Austin reported that the arms touched the ground and stopped. Archived 1,140
schema-12 samples on verified source 09d2e01/app1; end reason
`lower_descent_timeout`. [Preflight](preflight.json), [archive](archive.json),
[post-export health](post-archive.json), [analysis](analysis.json).
CSV/wire remain in this checkout under
`telemetry_logs/bal_20260921T020520Z_v12_contact_stop_wifi.{csv,wire}`.

Confirmed support begins at 19.790 s/82.825 degrees. Both targets remain exactly
2.071/5.370 rad for all 150 Descending samples, until the 22.790 s timeout.
Zero samples have both measured torque magnitudes at least 0.2 Nm. Median body
tilt is 82.741 degrees, rate -0.0185 degrees/s and arm torques 0.0455/0.030 Nm.
132/150 samples satisfy the established normal return's [-12,+4] rate bounds.

This establishes the new v12 load veto as the immediate cause of the stationary
targets. Low holding torque does not distinguish mechanically supported arms
from lost floor contact. The original contact model did not reproduce this
low reported torque while supported. V13 therefore preserves normal return
under calm weak-load readings after the existing two-arm catch confirmation;
a rapid unsupported fall still pauses and retains all fault guards.

Fresh pre/post checks passed: idle/disarmed, saving finished, powered motors
healthy/disabled, fresh IMU and released maintenance. No autonomous motion or
settings change was performed during archive/analysis.
