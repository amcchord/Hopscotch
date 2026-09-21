# CH6 fast laydown v10

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

Installed source `7917543` for operator testing; [current state](../../progress/CURRENT.md) identifies
the installed image. [V9 succeeded](../../../evidence/balance-lower/trial-v9-success-20260920/README.md),
with 9.900 seconds of supported arm return in a 12.009-second laydown.

- **CH6 HIGH before the lowering CH11 pulse:** fast laydown.
- **CH6 LOW or center:** the successful normal v9 laydown.

Selection latches when CH11 lowering is accepted, independently of the stand-up
choice. Changing CH6 afterward does not change an active maneuver. CH11 remains
the trigger, and CH6 retains its existing fast/slow stand-up role.

Fast mode keeps the demonstrated departure and arm catch. After support is
confirmed, speed blends up over 0.6 seconds toward 0.60 rad/s target speed with
a 0.75-rad/s motor cap and a 20-degrees/s descent pause threshold. It tapers
back to normal between 35 and 15 degrees body tilt. Normal values remain
0.24 rad/s, 0.30 rad/s and 12 degrees/s. The backward-rate pause, support-loss,
lead, global motion, time/progress limits, flat dwell and final retraction remain.

[Tests, replay and model](../../../evidence/balance-lower/fast-return-v10/README.md)
preserve normal mode exactly on three recordings and all 329 modeled cases.
Fast simulation retains 258 completions and all fault outcomes, saving a median
2.04 seconds among common successes. This is an approximate model result;
physical speed and reliability need testing. Schema10 retains 240-byte samples
and records the independently latched fast-lowering choice in pilot flag 4096.

For the next [manual trial](../../BALANCE_TESTING.md), stand normally, select CH6 HIGH,
pulse CH11 to lower, then disarm and archive before another run. Verify continued
arm return, a gentle final landing and arms Forward. No autonomous motion.

[Verified release and recovery](../../../evidence/lowering-v10-integration/README.md).

## First physical fast-laydown result

Austin confirmed the next run worked. [Telemetry](../../../evidence/balance-lower/fast-tip-drift-20260921T011755Z/README.md)
records CH6 fast selection and lower_complete in 8.239 seconds, with 6.170 seconds
of supported return. The earlier normal v9 run took 12.009/9.900 seconds respectively.
Final tilt -1.814 degrees, rate -0.182 degrees/s and Forward errors -0.028/+0.006 rad.
These are separate observed runs, not a controlled performance or reliability
study. The faster lowering succeeded; current follow-up concerns stand-up drift.
