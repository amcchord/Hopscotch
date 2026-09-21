# Forward-fall lowering v2: integrated candidate, not installed

September 20, 2026. The release checkout integrated the lowering owner's
physical-trial archive `f011ce6` as `7a98037` and candidate `13d4ee2` as
`dd74154aa3f534f95506b5c0dd66e1c6d901e091`. This checkpoint prepares and validates
the combined application only. **No device requests, flash, restart, settings
change, arming or movement occurred during this integration.**

The installed firmware remains the last verified `43b1967` application, app1,
with ESP digest `bc1e158acac24fa08a9fb81b26933b00243baa71f67c106c9696135182f0a9b3`.
That identity is from the previous deployment and subsequent trial download;
it was not queried again here. Austin reported good ground/standing driving
and failed backward lowering on that image. Hold further v1 lowering attempts.

## Findings and scope

The [2,371-sample failed trial](../balance-lower/trial-20260920/README.md) shows
the old policy leaning backward as it counterbalanced the reaching arms.
It never qualified support. Replaying the deployed C++ helper independently
reproduced the archived phase transitions and calculated metrics exactly.
The committed CSV and original wire bytes are unchanged.

The [v2 candidate](../balance-lower/forward-catch-v2/README.md) stages the arms,
transfers wheel ownership out of upright balance to initiate forward rotation,
holds each arm on first qualifying load, verifies the catch, then lowers under
support. The [design and trial guide](../../docs/BALANCE_LOWER_2026-09.md)
documents the thresholds, failure behavior and limitations.

Only `src/balance_lower.h`, `src/balance_controller.cpp` and
`src/balance_controller.h` differ from deployed release record `eb2bb23` within
production source/build/dashboard/radio files. Those three files match the
lowering handoff byte-for-byte. All other 35 files in those areas match the
deployed baseline. Review confirms ordinary pilot/ground-drive code, braking
v5, startup, motor configuration and network implementation are preserved;
controller changes serve lowering and its telemetry. Existing v1 CSV metadata
descriptors are unchanged. New captures retain schema 4 / 240-byte samples,
with feature flags 8191 identifying the changed lowering policy and rate field.

## Independent checks

- [Combined validation](validation-combined.txt): all 10 native executables,
  27 Python tests, Python/shell syntax, whitespace and pinned ESP32 build passed.
  The existing `ARDUINO_EVENT_RUNNING_CORE` redefinition warning remains.
- [Radio checks](validation-radio.txt) and
  [dashboard checks](validation-dashboard.txt) passed.
- Reran all 229 model cases through the integrated C++ helper. The resulting
  JSON exactly matches the owner's archived simulation: nominal and the extra
  high-inertia case complete; 180 of 216 variants complete, 36 stop with a
  preparation fault; all 11 injected failures reject success.
- Historical replay and source/data preservation checks passed; see
  [verification.json](verification.json). No additional driving simulation was
  needed because its helpers/configuration are unchanged and native driving
  regressions passed in the combined checks.

Model nominal completion is 19.90 seconds with a 19.838 degrees/second peak.
Completed variants reach 55.151 degrees/second. Geometry, contact, inertia and
impact assumptions remain unmeasured; these results do not establish a graceful
or reliable physical catch. Fault detection does not guarantee recovery from
an unsupported fall. A separate restrained operator trial is still required.

The model rerun used:

```sh
/Users/austinmcchord/Development/Hopscotch/.venv/bin/python scripts/simulate_lowering.py --output output/lowering-v2-integration
```

The combined check used this checkout's ignored `output/bin/pio` wrapper first
on PATH, followed by the project Python environment, and
`bash scripts/check_balance_candidate.sh`. The wrapper selects the pinned
Python 3.13 PlatformIO runtime and `output/release.ini`. That configuration
adds only an include path to the existing private project header. Secrets were
neither copied nor printed. The built application was checked for all four
expected private configuration values without revealing them.

## Queued package and next action

[candidate-manifest.json](candidate-manifest.json) records exact source, size,
whole-file hash and ESP digest. The credential-bearing binary/ELF/partitions
remain ignored and private at
`worktrees/drive-braking/artifacts/lowering-v2/candidate/` relative to the
project root. They are **not uploaded**. The manifest explicitly marks
`queued_not_installed`.

The installed v5/v1 package at
`worktrees/drive-braking/artifacts/drive-braking-v5/release/` remains intact;
its application hash was verified against its original manifest. No earlier
release package was overwritten. When deployment is authorized separately,
use fresh disarmed preflight and application-only OTA, retain saved logs and
verify the running image before any supervised test. Root shared operating
records remain owned by the integration coordinator.
