# Continue from arm contact to level ground

The [physical v3 run](../trial-v3-20260920/README.md) stopped upright at a loaded
rebound; one operator CH12 press then lowered the robot. V4 makes that return
part of CH11 while retaining measured flat/Forward completion.

- First qualifying contact stops outward targets and immediately reverses both
  toward calibrated Forward, with a 0.5 rad/s motor speed limit. One-arm-only
  contact permits at most 0.06 rad retreat; independent contact on both arms
  unlocks the continuous return to Forward, rather than the v3 capped retreat.
- Return speed tapers as forward body rate approaches 12°/s. While support is
  being established, faster forward falling pauses further withdrawal.
- Both independent contacts must show at least 0.2 Nm load, arm velocity must
  be stationary or moving toward Forward (−0.1 to +0.7 rad/s in each mirrored
  direction), and forward body movement must be observed. Body rate must be
  between −12 and +4°/s for 80 ms before supported descent starts. A steady
  supported descent need not exhibit another one-degree/second deceleration.
- A loaded rebound may settle for at most 300 ms after first impact, with
  two-arm load evidence no older than 60 ms and tilt still forward of launch.
  Backward displacement beyond launch+2°, the global 65°/s sampled rate limit,
  3 rad/s wheel limit, health/feedback requirements and 2.5-second catch deadline
  remain. This is not permission for an unsupported backward fall.
- Supported descent continues to the existing 600-ms flat confirmation and
  measured Forward pose. `lower_complete` requires tilt within ±5°, quiet body
  and wheels, both final targets at Forward, and both arms within 0.06 rad.

Native regressions exercise the two recorded v3 impact frames, immediate
reversal, continuation past the old retreat limit, two-arm support during
return, no upright completion, one-arm limits, stale/absent support and
persistent/backward/overspeed rebound faults. Existing completion, false/missed
catch, timing/feedback, mirrored geometry and clock-rollover checks remain.

## Paired model screen

`scripts/simulate_lowering.py --baseline-ref 8449ddb` runs both policies through
the same model. It now respects the actual policy motor-speed limits, and adds
24 delayed/acceleration-limited contact cases plus one recorded-v3-inspired
example. Those are sensitivity assumptions, not identified motor parameters.
The physical trace establishes that outward velocity persisted for at least one
20-ms frame after target reversal; the prior instantaneous response omitted this.

Of 329 cases, v3 completes 257 and v4 completes 260: seven improvements, **four
regressions** (`sweep_27`, `sweep_99`, `sweep_126`, `sweep_162`, all missed catch).
All 11 injected no-floor/contact-loss/health/obstruction/direction cases reject
completion. Nominal simulation reaches level/Forward; exact metrics and cases
are in [summary](summary.json), [candidate](simulation.json),
[baseline](baseline-simulation.json) and [comparison](comparison.json).

Stiff delayed impacts can still hit motion/direction limits. No universal
rebound fix or physically graceful landing is claimed. The simulator terminates
at a fault, so failed-case peaks cover different durations and cannot compare
the later physical motion after each controller stops. Contact force, floor
geometry and joint inertia are unmeasured; lateral roll and structural loads
remain outside the model. The next physical test must be restrained and logged.

Ground/standing drive, normal startup, radio and networking behavior are outside
this change. The separately owned CH6 fast tip-up is integrated only after its
source handoff, then the combined firmware is checked/built once for release.
