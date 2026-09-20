# CH11 forward-fall/catch v2 candidate

This is a software/model candidate, **not an installed or physically validated
maneuver**. It implements Austin's clarified forward fall and arm catch while
preserving the deployed driving controls. See the [operator/design guide](../../../docs/BALANCE_LOWER_2026-09.md)
and [failed physical v1 trial](../trial-20260920/README.md).

- [Measured failure versus nominal candidate model](comparison.png).
- [All model inputs/outcomes](simulation.json), [text output](simulation.txt),
  [nominal trace](nominal.csv), [plot script](render.py).
- [Consolidated native/Python/build checks](validation.log).
- [Preserved deployed source and telemetry checks](source-preservation.json).

## Results

The screen executes the production C++ helper through a native bridge. There
are 229 cases: nominal, one high-arm-inertia sensitivity case, 11 fault
injections and 216 parameter variants. Nominal and high-inertia cases complete.
Of the 216 variants, 180 complete and 36 fault during preparation before the
intentional forward departure. No variant that reaches departure reports a
failed catch in this chosen grid. This does not predict hardware reliability.

Nominal: preparation completes at 5.92 s; catch qualifies at 6.74 s; final flat
hold and arm retraction complete at 19.90 s. Peak body rate is 19.838°/s and rear
speed 1.730 rad/s. Across completed variants, peak body rates range from
11.25 to **55.151°/s**. The upper end is a hard catch, not evidence of grace.
The model reports finite contact forces but does not validate impact loads.

The 11 injected failures are absent floor, one-arm support, stale feedback in
preparation/fall/supported descent, unreachable floor, very slow servos,
support loss, obstructed arms, reversed initiation and false load without a
floor. None reports `lower_complete`. Missing floor/obstruction can still
produce fast uncontrolled motion before detection; reporting a fault does not
mean an unsupported robot safely recovered. A restraint is required for the
first physical trial.

The model evolves body gravity, wheel acceleration/lag, arm servo motion,
arm inertial reaction and unilateral floor contact throughout preparation,
departure and descent. It includes a 200 Hz PD / 50 Hz stationary-PI
approximation, rather than assuming a stable tilt servo during preparation.
Geometry and contact/joint parameters are assumptions, and the wide-angle COM
model is swept between linear and sinusoidal arm dependence. This is not a
full firmware, lateral dynamics, friction or structural model. It was used to
screen candidate designs and is not an independent physical validation.

The prior policy waited for contact while extending the arms under upright
balance. The new policy stages the arms, then explicitly transfers wheel
ownership to a bounded forward initiation. The fast sweep waits for measured
forward body rate. First load stops each arm; a small bounded yield absorbs an
early catch; sustained load and measured forward motion/deceleration qualify
support. Only afterward do arms lower and rear speed return to zero. The
normal controller cannot simultaneously command a backward balance target.

## Reproduction and release handoff

```bash
.venv-pio/bin/python scripts/simulate_lowering.py
.venv-pio/bin/python evidence/balance-lower/trial-20260920/analyze.py
./scripts/check_balance_candidate.sh
.venv-pio/bin/python evidence/balance-lower/forward-catch-v2/render.py
```

The physical replay pins the deployed `43b1967` helper/bridge, matches all phases
exactly and preserves 2,371 CSV rows plus original wire bytes. Eleven selected
production drive/config/radio/network files match `eb2bb23` byte-for-byte.
The candidate changes `balance_lower.h`, its lowering-only controller
integration, helper tests, bridge/model and dedicated docs/evidence. It retains
v5 braking, the ground gate, arming and motor setup.

Compile uses public example credentials; its binary is not an OTA package.
The release owner must build with the existing private configuration and
record verification before an operator trial. No device motion, configuration
change or flash occurred while preparing this candidate. The only device
operation was read-only status/identity and a disarmed, validated saved-log
export. The previous installed image remains the v1 lowering release.
