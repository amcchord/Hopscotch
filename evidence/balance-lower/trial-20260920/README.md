# Failed physical CH11 trial — September 20, 2026

Austin reports that ground driving and standing driving both worked very well.
CH11 lowering tipped backward on its own; he did not catch or push the robot.
His clarified maneuver is to initiate a forward fall and catch it with the
arms. The deployed contact-before-release policy does not implement that intent.

Downloaded and validated the latest saved run while both groups were disarmed,
balance idle, saving false and maintenance allowed. Live `/api/info` matched
deployed source `43b1967`, build `Sep 19 2026 23:44:38`, app1, ESP digest
`bc1e158acac24fa08a9fb81b26933b00243baa71f67c106c9696135182f0a9b3`.
No flash, arming, settings change or movement was requested during diagnosis.

- [CSV](../../../telemetry_logs/bal_20260920_lowering_trial_wifi.csv): 2,371 samples, 47.419 seconds; schema 4 / feature flags 4095.
- [Exact wire export](../../../telemetry_logs/bal_20260920_lowering_trial_wifi.wire); transport and device checksum validation passed.
- [Calculated metrics and hashes](analysis.json), [plot](lowering-trial.png).
- Reproduce with `.venv-pio/bin/python evidence/balance-lower/trial-20260920/analyze.py`.

| Run time | Event | Measured tilt | Tilt rate |
| --- | --- | --- | --- |
| 37.020 s | CH11 accepted / stopping | 86.631° | +0.257°/s |
| 37.500 s | Arms begin reaching | 86.601° | +0.308°/s |
| 45.600 s | Reach canceled: wheel feedback exceeds 4 rad/s | 90.146° | +7.447°/s |
| 47.419 s | `lower_motion_limit` | 108.494° | +77.060°/s |

Increasing tilt is backward. During reach, the arm-scheduled base rose from
85.215° to 92.364°. The effective target peaked at 91.233° and measured tilt at
91.763°. The balance controller deliberately compensated the forward arm motion
by leaning backward. It reserved its -3° forward lean for the later Loading
phase, which this run never reached. Reconstructed `setpoint - base_sp -
sp_offset` stays zero to export precision throughout reach.

Neither arm provided the resisted-motion evidence required for contact:
maximum target resistance was 0.017/0.018 rad, below the 0.06-rad threshold;
right-arm torque stayed below 0.4 Nm. These facts establish that the software
never qualified support; they do **not** establish whether an arm physically
touched the floor. There is no dedicated contact sensor or synchronized video.
Recorded phases contain no Loading or Descending; all lowering rows remain
balance state 2.

Both rear feedback velocities exceeded the 4-rad/s reach limit at cancellation.
While the helper slowly retracted the arms, wheel balancing continued. Its
ordinary recovery requested forward arm assistance, but the lowering override
still owned the arm targets. The robot did not recover: wheel command reached
-30 rad/s and the filtered tilt rate crossed the helper's 65°/s limit. This
is not a successful safe cancellation, and no claim is made that a different
arm recovery command alone would have caught it.

The lowered-priority network is not implicated by recorded timing: during the
maneuver maximum inner interval was 5.469 ms, outer sample interval 21 ms,
IMU age 10 ms and rear-feedback age 6 ms. Pilot CH1/CH2 inputs remained neutral.
Normal driving preceding CH11 includes 1,049 samples with the driving controller
active; ordinary flat driving is outside this balance log and is established
by Austin's observation.

The historical replay compiles the **deployed** `43b1967` C++ helper and feeds
recorded measurements through it. It matches every recorded helper phase,
including `lower_reach_disturbed` and `lower_motion_limit`, with zero phase
mismatches. This validates the diagnosis of its decisions, not robot physics.
The earlier contact model assumed stable preparation and could not expose this
coupled backward lean / failed recovery. Its prior 81-case screen must not be
treated as evidence that the deployed maneuver works on the robot.

Next candidate: stage the arms for interception, deliberately transfer out of
upright balancing to a bounded forward fall, infer and verify the catch from
arm load and body motion, then lower under arm support. Model wheel/body/arm
coupling and contact throughout the maneuver, including missed/asymmetric
catches and false load. Preserve v5 driving, ground gate, radio and network
source. Another hardware trial requires a revised candidate and supervised
catch restraint; the current v1 lowering trial has failed.
