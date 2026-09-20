# Forward preparation v5 — offline screen

The [physical v4 run](../trial-v4-20260920/README.md) scheduled +4.465 degrees
of backward lean during 6.2 seconds of arm preparation. The robot followed it,
then rebounded to +105.041 deg/s at contact. V5 caps the preparation target at
its measured starting tilt, deploys promptly, and hands off without waiting
upright for stationary arms. The catch first parks at 1.85 rad, then probes
slowly farther only after six measured degrees forward. The first loaded
body deceleration reverses the arms; measured flat/Forward still gates completion.

## Paired results and limitations

The same 329 modeled plants run through pinned installed e55cecb and the new
production C++ policy, including the actual C++ preparation setpoint cap and
motor speed limits. V4 completes 261; v5 completes 253, with 43 improvements and
51 regressions. This is **not an across-the-board improvement or hardware
acceptance**. All 11 injected failures reject completion, including the new
sustained wheel-direction disagreement check. Nominal completion 13.04 s,
peak 52.141 deg/s; preparation 0.52 s and body 86.630 → 86.184 degrees.

All 72 contact sensitivities with geometry informed by the recorded impacts
complete in both versions. Delayed/acceleration-limited contact cases improve
8 → 15 of 24. The delayed v3 example and stiff rebound example now complete.
43 of 51 regressions have the widest, unmeasured 0.16 m pivot geometry (41 motion
limits,2 missed catches);7 are other missed catches, and1 is asymmetric arm
preparation. These limits remain in the recorded results. Geometry, inertia,
friction and compliant contact are not identified physical measurements.
The model ends at a fault and does not predict the subsequent physical fall.

The previous immediate-release proposal was rejected:128/329 completed versus
261 for the installed policy under that paired model. It let the body outrun
the arms. Increasing free-flight arm speed also recreated moving-arm impacts;
those probes were not promoted or uploaded. The retained design changes the
preparation balance target so forward motion begins while arms deploy.

The plant speed ceiling is4 rad/s to permit the new preparation motor command;
actual per-phase limits still come from each compiled policy. The old policy
therefore retains its1.5 rad/s catch and0.3 rad/s descent cap. The preparation
controller remains an approximation of the firmware's stationary cascade.
Adding the C++ float setpoint interface changes one historical boundary case
(261 completions here versus260 in the previous v4 screen); compare only the
paired files in this folder.

The 6 rad/s wheel bound accommodates measured wheel velocity during this brief
forward preparation, rather than switching to the old 2 rad/s sender limit.
It is about0.33 m/s using the assumed 55mm wheel radius. Wheels stop at3 rad/s²
after support. Sampled body-rate65 deg/s, feedback/owner100 ms, load requirements,
deadlines and measured completion gates remain. Peaks between sampled checks
can exceed their thresholds; the detailed results record this limitation.

Files: [summary](summary.json), [new](simulation.json),
[baseline](baseline-simulation.json), [paired comparison](comparison.json),
[nominal trace](nominal.csv), [delayed impact](delayed-impact.csv).
Reproduce with `python3 scripts/simulate_lowering.py --baseline-ref e55cecb`.
