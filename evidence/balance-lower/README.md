# CH11 lowering evidence

**Current candidate:** [forward fall and arm catch v2](forward-catch-v2/README.md).
The historical v1 screen below was followed by a [failed physical trial](trial-20260920/README.md);
its assumed-stable preparation missed the observed backward lean. These v1
artifacts are retained as history, not current release validation.

This is offline simulation/test evidence. No physical maneuver was performed.
See the [behavior, model limits and first-test procedure](../../docs/BALANCE_LOWER_2026-09.md).

- [Simulation results and all input assumptions](simulation.json): 81 cases;
  72 geometry/compliance/servo/body variants plus nine nominal/fault scenarios.
- [Representative trajectory](nominal.csv), [plot](nominal.png): 26.12 seconds
  to completion, 18.09 deg/s peak body rate. The simulator assumes successful
  wheel balancing during preparation; this remains a physical validation gap.
- [Full native/Python/build validation](validation.log): ten native executables,
  27 Python tests, syntax checks, whitespace and ESP32 build passed.

Historical reproduction requires the `f26ece6` source checkout. Its
`python3 scripts/simulate_lowering.py` command compiles
`scripts/lowering_bridge.cpp` and calls the actual `balance_lower.h` policy.
The resulting CSV/JSON files are written under ignored `output/lowering/`.
Run `scripts/check_balance_candidate.sh` for the consolidated source/build gate.

The screen rejected immediate wheel release at arm contact: contact-seeking
preload could jack the body backward, and being supported exactly at the
unstable equilibrium did not ensure forward descent. The final implementation
removes preload and requires a small measured forward weight transfer while
wheel balancing remains active, before allowing supported lowering.

Final outcomes: nominal and moving-entry cases complete; 52 of 72 parameter
variants complete and 20 cancel without establishing support. Missing floor,
one-arm contact, unreachable geometry and slow servos cancel. Stale feedback
faults. Obstructed arms fail the weight-transfer test and retract under balance.
Injected floor-support loss faults at 41.28 deg/s in this model; that fault is
not evidence that the robot can recover from a slipping support in reality.
No fault injection reports a successful landing. These counts describe a
chosen parameter grid, not a measured reliability percentage.
