# Fast stand-up fall: rear-left wheel tracking mismatch

The latest run was archived by the device owner at 2026-09-21T01:08:06Z
(September 20 EDT): 291 samples over 5.906 seconds, schema 10, ending
`bailout_angle_error`. Installed source is
`45c1a94aa82aab952be420e5472c5ae8e377bef1`, app0, ESP digest
`a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61`.

Raw data remains in the balance-lower checkout under
`telemetry_logs/bal_20260921T010806Z_fast_tip_fall_wifi.csv` and `.wire`.
CSV SHA256: `57e62cd546ad52d8372f676b5e2382dbb99d3940dbbfdd2b291dd756ba4379e4`.
Wire SHA256: `235b1b710b9e817d79aa1d7c6f60d3413a4139f67f437a9c433a4e696cb72161`.
The owner verified fresh disarmed IDLE, finished saving, disabled healthy
powered motors, fresh IMU and released maintenance before/after export.

## What the trace establishes

- Quiet upright capture at 2.981 s: tilt 82.787 degrees, rate -0.028 degrees/s,
  rear speeds +0.047/+0.013 rad/s. The 2.6-second lift trajectory completed.
- Arm return starts at 3.401 s. Fast v2 correctly preserves stored trim and
  fades its temporary supported-capture offset; permanent curve shift stays zero.
- The rear-left wheel increasingly falls behind during backward recovery.
  At 5.506 s, left command/measurement are -21.276/-7.307 rad/s; right are
  -18.276/-17.951. Both feedback ages are 1 ms. Yaw correction is already at
  its 1.5 rad/s limit, commanding the slower left wheel harder.
- Left measured acceleration fits -9.607 rad/s² across 45 samples from
  4.701–5.586 s (R² 0.9983, speed residual RMSE 0.104 rad/s).
  This is measured response, **not** a readback of the motor acceleration setting.
- Angle error reaches 46.433 degrees at the last sample, 5.905 s; the run
  records the existing angle-error bailout. No lowering phase was attempted.
- Austin confirms the robot twisted. He saw no definite obstruction, but
  cannot exclude one.

The previous operator-confirmed v9 success has a comparable initial catch
(peak recovery integral +2.809 degrees versus +2.686 here), while its rear
wheels follow their commands together. Maximum measured wheel-speed difference
in the first three seconds after capture is 0.673 rad/s in that success versus
18.524 here. This comparison does not establish an experimental success rate.

[Comparison figure](wheel-tracking.png) and [derived measurements](analysis.json).
Figures align both runs at upright capture. Times above are measured from the
failed run's logging start. No raw logs were copied between worktrees.

## Interpretation and next step

The strongest new finding is asymmetric rear-wheel response during the catch.
A missed or unapplied acceleration setup is a candidate explanation for the
near-linear left response, alongside physical loading/obstruction or another
motor-side problem. It is not proven. The installed controller requests
`ACC_RAD=100` at Speed-mode setup but can verify only transmission enqueue:
RS05 acceleration is write-only under the repository's documented protocol.
Current-limit setup does require readback. See the existing
[motor setup investigation](../../../docs/archive/2026-09/BALANCE_START_FIX_2026-09.md).
Do not restore the previously rejected acceleration-readback requirement.

The installed `balance_tip_up.h`, `balance_math.h` and `motor_manager.cpp` are
byte-identical to fast v2 production commit `f4d2bb7`. The archived balance
interval has maximum inner tick 5.201 ms and outer update age 24 ms; feedback
is fresh during the decisive mismatch. This trace does not support a control
stall as the immediate explanation, nor does it establish that networking
caused the failure.

Review any existing motor-setup diagnostics and the left rear wheel's physical
condition before selecting a correction. Preserve the fast trajectory and
balance tuning until that discrepancy is understood. A motor-setup robustness
change, if pursued, needs a focused transport test and a separately observed
physical response; replaying telemetry cannot prove an unperformed recovery.
The balance-lower task retains device/integration ownership and received this
diagnosis. No motion, settings write, firmware change or OTA was performed by
the analysis task.

The device owner subsequently searched existing serial/setup captures and found
no contemporaneous setup log for this run. Wi-Fi telemetry records the motion,
not startup serial history. Existing files therefore cannot establish whether
the left motor applied the requested acceleration value.

## Reproduce

Run from `worktrees/fast-tip-up`, with the project Python environment that
contains NumPy and Matplotlib:

```sh
/Users/austinmcchord/Development/Hopscotch/.venv/bin/python scripts/analyze_fast_tip_tracking.py \
  /Users/austinmcchord/Development/Hopscotch/worktrees/balance-lower/telemetry_logs/bal_20260921T010806Z_fast_tip_fall_wifi.csv \
  /Users/austinmcchord/Development/Hopscotch/worktrees/balance-lower/telemetry_logs/bal_20260920_forward_catch_v9_success_wifi.csv \
  --output evidence/fast-tip-up/fall-20260921 \
  --installed-source 45c1a94aa82aab952be420e5472c5ae8e377bef1 --fit-window 4.7 5.6
```

Validation: reproduced derived measurements directly from checksum-matching
CSV, inspected the rendered comparison, checked unchanged relevant production
files against installed source, and ran whitespace/syntax checks. Firmware
tests/build were not repeated because no production code changed.
