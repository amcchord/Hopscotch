# Archived stand-up roll-away on lowering v13

Retrieved372 samples over7.424s; schema13, end `bailout_angle_error`.
Installed source7c32bd7 / ESP digest
`2cba5a84594a703bb377b6697791fca4da790e8b6c8d4fe5ddf0f2181461cdae`, app0.
Preflight and post-archive health confirm IDLE/disarmed, powered healthy motors,
fresh IMU and released maintenance. No robot motion or OTA was initiated.

- CSV: `telemetry_logs/bal_20260921T023411Z_standup_rollaway_wifi.csv`
- Original wire: matching `.wire` file.
- CSV SHA256: `c2ee34b480f8cc5f801b8e3d7d0ac431989aa184202493c8df4548fb886b3bc4`
- Wire SHA256: `48712f4fe2b691cebd3586ba6c8551de94d6a88f5ef8cf7ab651d585d875b09e`

[Joint arm/wheel investigation, plots and rejected offline candidates](../../balance-recovery/rollaway-20260921/README.md).
The two rear wheels track together before the large fall. Startup recovery
overshoots and arm COOLDOWN blocks full assist demand for1.239s after the return
ramp. A wrong equilibrium angle is possible but not established by this trace.
