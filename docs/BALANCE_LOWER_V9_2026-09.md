# CH11 lowering v9 — bounded contact rebound

Installed source `b9763c2` for operator testing; [current state](progress/CURRENT.md) identifies
what is installed. The [v8 trial](../evidence/balance-lower/trial-v8-stop-20260920/README.md)
confirmed another successful fast stand-up but stopped lowering on its arms.
It never entered the faster supported return: a small loaded rebound reset
contact confirmation, then a later rebound triggered `lower_wrong_direction`.

V9 allows the 80 ms confirmation to continue through a loaded rebound up to
+20 degrees/s and 1.5 degrees above the impact minimum, only within the
existing 300 ms impact window. Both arms must retain recent independent
load observations and satisfy measured reversal-velocity limits. Global
limits, support-loss rejection, timeouts and progress monitoring remain.
Once confirmed, the existing 0.24-rad/s supported return continues toward
Forward with its 0.30-rad/s motor cap and body-rate pause. Fast standing,
ordinary driving, initial fall, wheel braking and OTA behavior are unchanged.

[Replay and model evidence](../evidence/balance-lower/bounded-catch-v9/README.md)
places support confirmation at 13.011 s in the failed trace and preserves
all 329 model outcomes. After commands diverge, the old sensors are not a
physical prediction; support loss still rejects on that old trajectory.
The earlier successful trace produces identical v8/v9 commands. Schema9
adds policy metadata while retaining 240-byte samples and historical exports.

Follow the [existing test procedure](BALANCE_TESTING.md): one stand-up and
CH11 attempt, then disarm and archive before another run. Check that contact
leads to continued supported return and a level body with arms Forward.
Physical acceptance remains pending; exact successful v7 recovery is retained.

[Verified installation and recovery record](../evidence/lowering-v9-integration/README.md).

## Physical acceptance and successor

Austin confirmed [successful v9 lowering and fast standing](../evidence/balance-lower/trial-v9-success-20260920/README.md).
[V10](BALANCE_LOWER_V10_2026-09.md) retains this normal laydown and adds a
separately latched CH6 HIGH fast supported return. Earlier pending statements
above describe the pre-trial state.
