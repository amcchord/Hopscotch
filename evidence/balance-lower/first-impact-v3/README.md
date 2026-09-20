# Lowering v3: stop both strokes at first impact, then yield

This candidate follows the [physical v2 rebound](../trial-v2-20260920/README.md).
It is not yet a physically validated catch. The decisive measured frame had left
load above threshold, right just below, and continued right-arm extension. The
following frame reversed body rate sharply and faulted.

The new policy stops **both** forward arm targets at the first qualifying load,
retains separate contact latches, and allows 0.06 rad of retreat at 0.5 rad/s once
falling rate has slowed at least 1°/s from its measured peak. This budget is
anchored once, not reset by later/repeated loads. Two-arm load, slow arm motion
and body deceleration still must qualify before supported descent. One contact
does not imply support. Preparation is reduced from 1.30 to 1.25 rad; launch
acceleration from 4 to 2 rad/s²; the fast sweep requires at least 0.1° forward
travel as well as -1°/s rate. Maximum reach/speed and existing fault limits remain.

The recorded first-impact input is a native regression: the right target must
stop/recede with the loaded left, without reporting support. Further regressions
cover the premature 0.049° v2 sweep trigger, one-arm load and bounded retreat.
Existing deadlines, stale feedback, false/missed catch, direction, ground hold,
retraction, command bounds and clock rollover tests remain.

## Paired model screen

Run `python3 scripts/simulate_lowering.py --baseline-ref 13d4ee2 --output
output/lowering-v3-final-screen` to compare the installed v2 policy and candidate
through **the same revised model and cases**. The revised contact model includes
floor-approach velocity from moving arms; the prior model only damped body rate.
It also includes unequal arm speed and floor height sensitivities.

The original 229 cases are retained, plus three trial-informed examples and 72
contact/plant variants, for **304 paired cases**. The added family includes a
contact geometry consistent with measured q≈1.92 rad/body≈82.5°, and gravity
coefficients 18/24/27 versus the old unmeasured 8.82. A noisy single-run departure
fit suggested ~27, but geometry, inertia, stiffness and damping are **not
independently identified**. These are stress assumptions, not measured hardware
constants. An altered family can change the result.

| Outcome | Installed v2 policy | v3 candidate |
| --- | ---: | ---: |
| Model complete | 231 | 248 |
| Preparation disturbance | 36 | 36 |
| Wrong direction | 22 | 6 |
| Missed catch | 6 | 4 |
| Motion limit | 4 | 4 |
| Other faults | 5 | 6 |

Twenty-one cases change from fault to completion; **four previously complete
cases now fault** (one descent timeout, two missed catches, one wrong direction).
These regressions are retained in [summary.json](summary.json), not hidden by the
aggregate improvement. The deliberately asymmetric-servo example still misses
the catch; independent arm contact is a real limitation of this maneuver.
All 11 injected failure cases reject completion. The nominal model completes in
19.90 seconds, peak 19.396°/s. Completed variants peak at **55.226°/s**.

The stiffer trial-informed example changes from backward rebound/fault to
completion, but its peak body rate rises from 41.808 to 44.678°/s. Thus the result
supports testing the first-impact strategy, **not a claim that every catch is
softer or safe**. The 2D model omits lateral roll and realistic impact loads;
reported model torque is not a hardware load rating. Delaying/shortening the
arms aggressively was rejected in exploratory screens because it caused later,
faster impacts or missed catches. The final timing change is deliberately small.

Files: [candidate cases](simulation.json), [same-model v2 cases](baseline-simulation.json),
[paired outcomes](comparison.json), nominal and stress CSV traces. Historical
v1/v2 data and metadata remain unchanged. New captures use schema 4/240 bytes,
feature flags 16383 (new bit 8192). Ground/standing drive and startup are untouched.

The combined integration passed 10 native executables, 37 Python tests, the
pinned configured build, radio/dashboard and OTA transport checks; retained
`validation-*` logs record that source freeze. The initial `d8613e6` package was
not installed: a longer saved-run export exposed a watchdog reset during
preflight. Source `8449ddb` adds cooperative export and was subsequently
[installed and verified](../../ota-lowering-v3/README.md). Motion code and this
model screen are unchanged. The next physical lowering attempt must be a
restrained supervised trial; archive the log before another attempt.
