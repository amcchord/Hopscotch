# CH11 lowering v8 — faster supported return

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

Historical source `47eb19a`; [current state](../../progress/CURRENT.md) identifies
what is installed. V7 has now completed an operator-confirmed successful
lowering and a second successful fast stand-up. The [saved successful run](../../../evidence/balance-lower/trial-v7-success-20260920/README.md)
provides a baseline worth preserving.

The supported arm-return target now advances at 0.24 rad/s instead of 0.16,
a 50% increase. The existing 0.30-rad/s motor speed cap and pause outside
−12..+4°/s body rate remain. The initial fall/catch timing, contact qualification,
wheel braking, 0.24-rad target lead and all fault limits remain unchanged.
Final retraction stays at 0.30 rad/s: that phase took only 0.74 s and already
produced appreciable body movement. Measured flat/Forward completion is retained.

[Offline comparison](../../../evidence/balance-lower/faster-return-v8/README.md): 258/329
modeled completions before and after, no new failures. Common successes save
a median 2.72 s; nominal completion changes 13.08 → 10.14 s. These are approximate
model results. The faster return still needs a physical trial. Fast standing,
ordinary driving and the installed OTA progress/interlock are unchanged.

After installation, use the [existing test procedure](../../BALANCE_TESTING.md): one
stand-up and CH11 lowering attempt, then disarm and archive before another run.
Compare contact-to-flat duration and body rocking with v7. New telemetry schema8
keeps 240-byte samples and preserves the previous run's exports byte-for-byte.

## Subsequent physical trial

The [v8 run](../../../evidence/balance-lower/trial-v8-stop-20260920/README.md) confirms
another successful fast lift but stops during catch confirmation, before the
faster supported return is used. It does not establish a speed regression.
[V9](BALANCE_LOWER_V9_2026-09.md) retains the faster return and makes support
confirmation tolerant of the measured small loaded rebound.
