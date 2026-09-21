# Fast stand-up drift: reduced transient-learning candidate

Austin's successful September 20 evening trial is archived as 1,325 samples /
26.660 seconds, schema 10, `lower_complete`. Device owner retains raw CSV/wire
in `worktrees/balance-lower/telemetry_logs/` with stem
`bal_20260921T011755Z_fast_tip_success_drift_wifi`.
CSV SHA256 `4bc872c7e1b04c75438203abf25b3d66a7eac10412b2776a32ffc4b99a696a9a`;
wire SHA256 `f70a531e1a15bda75350092bbc094e55846de1c4963eb2362f1e8e33a15773be`.
Installed firmware remains `45c1a94aa82aab952be420e5472c5ae8e377bef1`, app0,
ESP digest `a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61`.
This candidate has not been uploaded or physically tested.

## Recorded behavior

[Observed trace](observed.png) and [measurements with four successful stand-up
references](observed.json). All travel measures stop before intentional driving
at 12.241 s; later lowering is excluded.

| Event or measure | Latest successful run |
| --- | ---: |
| Upright capture | 2.996 s / 82.595 degrees |
| Arm return starts | 3.436 s / 82.597 degrees |
| Early recovery starts | 3.756 s |
| Arm/base ramp complete | 5.256 s |
| Recovery settles | 7.361 s |
| Quiet Forward median angle | 87.147 degrees |
| Peak learned correction | +2.8529 degrees |
| Forward/reverse wheel-position extremes | +3.310 / -5.325 rad |
| Forward/reverse common-wheel speed extremes | +4.329 / -7.0235 rad/s |

The two rear wheels track together in this run: maximum speed difference
0.628 rad/s before driving, unlike the preceding failure. This does not
resolve that intermittent motor-response discrepancy.

The measured pre-return angle is a useful continuous initial reference, and
the controller already starts there. It is not the arms-Forward equilibrium:
the successful references show roughly 3.8–4.7 degrees between those poses.
Arm support, center-of-mass movement, floor/contact variation and sensor bias
cannot be separated by these logs alone. They do not establish accelerometer
drift as the cause. Reapplying fast v1's permanent supported-capture calibration
would repeat a previously failed policy; that change is not included.

## Candidate scope

Halve only the fast run's early recovery learning gain from 1.0 to 0.5 during
the existing maximum 800 ms boost. The recorded surge built nearly three
degrees of temporary correction, then the controller had to remove it during
the backward catch. Reducing that accumulation is the targeted experiment.

The actual production helper selects the gain from the **latched fast-run
flag**, not live CH6. Slow starts retain gain 1.0; both modes retain configured
normal learning after boost expiry. Recovery trigger, 6-degree integral bound,
6-degree/s update bound, slew, freshness checks, recoil confirmation and gain,
capture/trim behavior and 2.6-second lift/1.5-rad/s arm return are unchanged.
Standing drive, v10 normal/fast lowering and installed networking are preserved.

Schema 11 identifies fast v3 and its gain without changing the 240-byte row.
Historical saved logs export their historical versioned text. The current
lowering policy still identifies itself as v10.

## Evidence and limits

The actual C++ gain selector, recovery integrator and recoil helper run inside
[the paired sensitivity screen](screen.json) through
`scripts/fast_tip_recovery_bridge.cpp`. The remaining planar/contact model is
approximate. The nominal parameters are rounded exploratory fits to recent
healthy wheel-response runs, not an independently validated plant model.

In the nominal case, modeled peak wheel speed changes 7.267 → 3.089 rad/s;
maximum absolute wheel excursion 6.166 → 2.092 rad; 12-second total wheel path
17.710 → 12.081 rad. [Nominal simulated traces](nominal-traces.json) are retained.
These are model results, not predicted physical percentages.

The broad 385-case screen is mixed: no-fall counts change 116 → 153, with
57 improvements and **20 regressions to a fall**. Among 96 common no-fall
cases, total path improves in 59. All regressions remain listed with inputs.
The model omits asymmetric wheel response, slip and real contact impacts, and
even many no-fall cases do not settle. This candidate is a focused experimental
gain change, not a demonstrated general robustness improvement. Its effect on
physical drift and catches still needs a controlled operator trial.

## Verification and handoff

- Fast-tip native tests: 162 trajectory/tracking cases plus unchanged capture,
  fault/timeout checks, both-sign gain/expiry/unwind/freshness checks and schema
  compatibility through 11 (reject 12).
- Startup/recoil native tests: detectors, rollover, bounds, stale samples,
  hysteresis and 100,000 randomized integrator updates pass.
- Existing pilot compatibility test updated for schema 11 and passes. The
  integration owner caught its initial stale assertion rejecting schema 11;
  that failure is resolved.
- All 16 telemetry Python tests pass; analysis/screen scripts compile, observed
  chart inspected and whitespace checks pass.
- Pinned PlatformIO 6.7.0 configured ESP32 build passes using the existing
  private header in place. Build reports application section size 1,214,889
  bytes and RAM 53,600 bytes. Existing Arduino core macro redefinition warnings
  remain; no secrets or raw logs were copied.

The fast-tip task owns the candidate on `codex/fast-tip-up`. Public build
inputs were aligned to installed 45c1a94 in separate baseline commit `df5ed26`;
that alignment must **not** be cherry-picked back into integration. Hand off
only the subsequent focused candidate/evidence commits. The balance-lower
task owns combined checks, shared progress state and the robot. No OTA,
settings write or autonomous motion was performed here.

Reproduce the screen from the fast-tip checkout:

```sh
/Users/austinmcchord/Development/Hopscotch/.venv/bin/python scripts/screen_fast_tip_drift.py \
  --output evidence/fast-tip-up/drift-v3-20260921
```

`scripts/analyze_fast_tip_drift.py` takes the archived CSV, optional
`--references` CSV paths and `--output`; exact read-in-place paths and their
checksums are retained in `observed.json`.

## Integration review

The device/integration owner reviewed the runtime scope and retained this as
an experimental operator-test candidate. The 20 modeled regressions include
case 223 (A=23, B=8, lag=0.035, arm equilibrium delta=-3.5, Forward equilibrium
88.7, release fraction=0.99), so they cannot be dismissed as only extreme motor
lag. Poor modeled settling limits conclusions in either direction. The observed
transient correction/recoil motivates the trial; no physical improvement or
sensor-drift cause has been demonstrated.

Focused commits e08184d/cf7db90/32c4b68 integrated as
2c90263/a0243d9/db437fa on codex/balance-lower; baseline alignment df5ed26 was
excluded. Production src/data/platformio inputs match the reviewed candidate.
The consolidated check at db437fa passes all 12 native suites, 44 Python tests,
script syntax/whitespace and the pinned configured ESP32 build (application
section 1,214,905 bytes, RAM 53,600 bytes). Existing Arduino macro-redefinition
warnings remain. No OTA, robot request, settings write or autonomous motion
was performed; installed firmware remains 45c1a94. Candidate is queued,
unflashed and awaiting the next operator-test decision.
