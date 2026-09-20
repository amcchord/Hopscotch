# Faster supported return v8

The [successful physical v7 run](../trial-v7-success-20260920/README.md) spent
11.370 of its 14.109 seconds in supported descent, with close arm tracking.
V8 changes that phase's target ramp from 0.16 to 0.24 rad/s (50% faster).
The separate 0.30 rad/s motor speed cap, body-rate pause outside −12..+4°/s,
0.24-rad target lead, contact sequence, wheel braking, final 0.30-rad/s
retraction, flat dwell, completion gates and all failure limits remain unchanged.
Fast stand-up and normal driving source are preserved.

The [paired 329-case model](simulation.json) uses the same cases as
[installed v7](baseline-simulation.json): both complete 258 cases, with
[no success/failure changes](comparison.json). All 258 common successes are
faster, with a median 2.72 seconds saved. Nominal completion changes from
13.08 to 10.14 seconds. All 72 trial-informed contact cases complete; delayed
contact remains 15/24; all 11 injected failures reject completion.
[Summary](summary.json). These times are model outcomes, not physical promises.
The simulator uses production C++ policy but unmeasured plant/contact geometry,
omits lateral dynamics and standing-drive handoff, and stops at policy faults.

Focused native checks exercise the faster one-second supported return with
recorded body-rate interruptions and resume, while keeping the motor cap.
Existing single-impact rejection, stale feedback, lost support, wrong direction,
rate/wheel limits, target-lead bounds, deadline and flat/Forward checks remain.

Reproduce: `python3 scripts/simulate_lowering.py --baseline-ref c442e12 --output output/lowering-v8-screen`.
New captures use metadata schema8 with unchanged 240-byte samples; prior schema7
exports keep their original policy metadata. Hardware acceptance of v8 is pending.
