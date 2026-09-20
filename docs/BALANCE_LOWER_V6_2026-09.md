# CH11 lowering v6 — hand off before the preparation abort

Prepared for the next combined update. [Current state](progress/CURRENT.md)
identifies what is installed. The operator reports that v5 now leans forward
reliably but stops supported on its arms.

The [saved v5 run](../evidence/balance-lower/trial-v5-20260920/README.md) identifies
an earlier cause: upright control accelerates a wheel past the 6 rad/s limit
while waiting for the final arm travel. It ends `lower_prepare_disturbed`
before the catch/return phases. The arms contact and support the body after
that abort, outside the saved capture.

V6 permits an earlier handoff once both arms have reached 1.6 rad, both still
advance at least 0.3 rad/s, neither carries 0.4 Nm of load, and the body has
fallen at least 0.5° with forward rate at least 2°/s. The original final-arm-pose
handoff remains. This releases upright control while the catch is being placed;
it does not increase wheel or body limits. The backward setpoint cap remains.

Once the existing contact checks recognize the catch, both arm targets reverse
toward calibrated Forward. Two-arm support permits continued withdrawal and
supported descent. Completion requires measured flat/quiet body and Forward
arms. Catch/return tuning and ordinary standing/ground driving are unchanged.

The recorded-input replay hands off 39 ms before the old abort, with wheels
1.695/4.490 rad/s, retaining their 3.0925 rad/s mean. It stops at the point the
new commands diverge from the recording. Native checks then exercise qualified
contact and continued return; neither test proves the later physical motion.

The [same 329-case model](../evidence/balance-lower/early-handoff-v6/README.md)
completes 255 cases versus v5's 253, with 9 improvements and 7 regressions.
All 72 trial-informed contact cases complete and all 11 injected failures
reject completion. Unmeasured geometry/contact dynamics remain limitations.
New captures use schema 6 with the unchanged 240-byte sample layout;
historical v4/v5 exports retain their original metadata.

## Operator trial after installation

Use the existing [clear-area/spotter procedure](BALANCE_TESTING.md). Start flat
with arms Forward, observe both arming switches LOW, and arm normally. Use
CH6 LOW to stand for this lowering trial; after settling, center CH1/CH2 and
pulse CH11 once. Expect forward departure, contact and continuous arm return to
a level finish. Keep the fast stand-up test separate. Disarm both groups and
archive the log before another attempt replaces it; avoid CH12 assistance in
the recorded trial.
