# Fast v2 — first operator-confirmed successful stand-up

On September 20, Austin reported that the fast tip worked, with substantial
wheel travel to catch balance. The same run then included intentional driving,
manual settling of oscillations, and an unsuccessful lowering attempt.
The lowering task owns those latter fixes and the next combined release.
**Keep the successful fast v2 production policy unchanged for this iteration.**

Installed source was `a772ecc833c6100f76cdb67173bdaf4f4d99cd78`, app1,
ESP digest `fdf7af6a96d2b6e3c3c908301ae765fcfa4c9110df27f8d88693dd80e8aeb2c2`.
The integration owner verified identity and archived 1,519 samples / 30.370 s,
schema 6, with fast-run bit128 throughout. The run ends `lower_wrong_direction`.
The raw CSV and wire remain in that owner's `worktrees/balance-lower/telemetry_logs/`
checkout as `bal_20260920_forward_catch_v6_wifi`; this task read them in place.
[Derived metrics, events, source path and checksum](analysis.json).

## Successful fast sequence

| Event | Time from logging start |
| --- | ---: |
| Quiet upright capture | 2.885 s |
| Arms start returning | 3.326 s |
| Arm/base ramp complete | 5.145 s |
| Startup recovery declares settled | 6.926 s |
| Standing drive ready | 8.385 s |
| First intentional drive | 10.185 s |

The temporary supported capture shift is −2.430° and fades away; the permanent
run curve shift stays zero. Saved trim +3.2398° supplies the 87.24° final base,
with about +0.15° learned residual before driving. The quiet measured body is
about 87.4°. The prior failure's supported-angle recalibration no longer occurs.
The operator report plus completed ramp/recovery provide evidence of one
successful physical fast stand-up, not a reliability rate.

[Fast-only timeline before intentional driving](fast-success.png).

## Remaining catch travel

Before intentional driving, mean rear-wheel position reaches +1.864 rad forward,
then −2.635 rad behind the capture origin. Common wheel speed peaks at
+2.9145 and −4.7845 rad/s. The recovery integral peaks at +1.779° and subsequently
unwinds. The larger backward speed and 4.499-rad excursion from the forward
peak to reverse peak show that recoil dominates the remaining catch travel.
Wheel radians are not ground displacement; tire radius and slip are unmeasured.

The fast correction now works with the existing startup recovery. Changing its
learning/unwind timing could affect both catch and recoil. This single success
supports preserving it while fixing the separate lowering failures. Further
catch tuning should compare repeated fast starts and model both initial capture
and recoil, rather than change gains based on this one outcome.

## Later oscillation and lowering context

Intentional driving starts at 10.185 s. The lowering request is accepted at
16.745 s, but its stopping phase persists until 29.270 s. Neutral driving remains
active (`pilot_flags` driving16/moving2), because its continuous calm requirement
never completes. During the unassisted waiting interval, body tilt spans about
0.812° and raw rates reach 8.545°/s. The lower gate's rate≤4°/s and each wheel≤0.65
rad/s conditions stay simultaneously quiet for at most 260 ms, short of 500 ms.
The operator reports manually settling it before departure.

The ending lowering fault is separate from fast capture. The integration owner
is correcting the neutral driving handoff during a lowering request and the
contact/support transition. Fast source, arm trajectory, capture reference,
startup recovery gains and return speed are unchanged in this task.

## Reproduce

```sh
/Users/austinmcchord/Development/Hopscotch/.venv/bin/python scripts/analyze_fast_tip_success.py /Users/austinmcchord/Development/Hopscotch/worktrees/balance-lower/telemetry_logs/bal_20260920_forward_catch_v6_wifi.csv --output evidence/fast-tip-up/trial-v2-20260920
```

The script validates fast-run flags, derives milestones from telemetry states
and flags, excludes all later driving from catch metrics, and writes a reviewed
plot. No new motion code was added; no duplicate firmware build was run. This
task performed no robot calls, raw-log copying, settings writes or deployment.
