# Current state — September 19, 2026

- **Objective:** Austin authorized implementing/uploading the faster recoil-release experiment and will perform the physical test.
- **Workspace:** `/Users/austinmcchord/Development/Hopscotch`, root checkout, branch `codex/balance-review-ready`, tracking GitHub origin; previous remote head `75318bc`.
- **Candidate:** confirmed post-ramp recoil temporarily doubles ordinary integral release (0.231 → 0.462), with 60 ms direction confirmation, 0.35/0.15 rad/s hysteresis and 120 ms gain blend. Existing initial catch, limits, calibration and 400 ms calm check stay intact. Feature flags 63 add a recoil marker without changing sample size.
- **Validation:** five native suites, 21 Python tests, syntax/whitespace and firmware build passed. Final 324-case approximate screen: early/later failures 79/42 vs baseline 86/60, no new failures, 25 rescues; paired median settling change 0.0 s. No physical benefit measured yet.
- **Candidate image:** SHA-256 `5d3465e0269f9df19065d1583de35d093b46f947fdd27e49ef0de0bbf28d4dd0`, 1,139,280 bytes. Flash usage 1,138,913; static RAM 50,768.
- **Still installed:** two-success baseline source `720f2e31939a249a215b2b7f7c197130f2301310`, application `5531c6287dd5ef8d398d426a7596d8c6d91e6c914825d137bc9a1ef485734e35`, frozen package `artifacts/balance-startup-recovery/`. Both successful logs are safely archived/pushed. Last observed trim 3.51°; calibration retained.
- **Records:** [candidate findings](../BALANCE_RECOIL_RELEASE_2026-09.md), [prior review](../BALANCE_SETTLING_REVIEW_2026-09-19.md), `evidence/balance-recoil-release/`, `telemetry_logs/TUNING_HISTORY.md`.
- **Next:** freeze candidate package, fresh disarm/health/pending-log check, authorized application-only upload at 0x10000, then stationary verification and one operator-triggered comparison. No tool arming or motion; restore proven baseline via its package without `--rollback` if needed.
