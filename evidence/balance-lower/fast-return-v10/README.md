# CH6 fast supported return v10

CH6 HIGH (>0.5 normalized) is latched independently at the accepted CH11
lowering request. LOW/center retain normal v9. The [successful v9 recording](../trial-v9-success-20260920/README.md)
spends 9.900 of 12.009 seconds in supported descent with close arm tracking.
Final retraction is only 0.139 seconds, so it remains unchanged.

Fast mode blends over 600 ms after support confirmation toward a 0.60-rad/s
target ramp, 0.75-rad/s motor cap and 20-degrees/s descent pause threshold.
Normal values are 0.24, 0.30 and 12. Speed tapers to normal from body tilt
35 to 15 degrees. The +4-degrees/s backward-rate pause is unchanged, as are
initial departure/catch, support-loss rejection, lead, global motion limits,
progress/time limits and measured flat/Forward completion. No speed choice
is changed in the middle of a maneuver. Fast standing uses its existing policy.

Native tests verify independent request latching, rejected-request/reset
behavior, matching preparation/catch, gradual acceleration, fast and normal
rate pauses, floor taper, unchanged final dwell/retraction and fault rejection.
[Recorded replay](replay.json), reproducible with `python3
 evidence/balance-lower/fast-return-v10/replay.py`, gives identical normal v9/v10
commands on three archived runs. Fast commands first differ only after support.
Old recorded sensors cannot predict physical fast motion after that divergence.

Paired model command: `python3 scripts/simulate_lowering.py --fast
--baseline-ref b9763c2 --output output/fast-lower-v10-screen`. Also run without
`--fast` into `output/normal-lower-v10-screen` to check normal mode.
[Summary](summary.json), [comparison](comparison.json),
[fast simulation](fast-simulation.json), [baseline](baseline-simulation.json),
[nominal](nominal.csv), [rebound stress](rebound-stress.csv).

All 329 outcomes match: 258 complete, 72/72 contact cases, 15/24 delayed-contact
cases and rejection of all 11 injected faults. Normal-mode results exactly
match v9, including times, dynamics and phase transitions. All common fast
successes take less time: median saving 2.04 seconds; nominal 10.14 to 8.08.
This approximate model is not an identified robot/contact plant. Physical
speed, load and robustness still require manual acceptance.
