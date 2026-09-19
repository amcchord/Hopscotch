# Confirmed recoil release v1

Source `1d80257e5609a173d9bbe104997eab1f9d4c7341` was uploaded application-only and verified. See `device-checks.json` for exact image/health identity and `../../docs/BALANCE_RECOIL_RELEASE_2026-09.md` for behavior, validation, limitations, restoration and physical test.

- `screen.py` / `.json` / `.txt`: frozen baseline versus final mirrored policy, 324 cases each; initial catch output equality asserted.
- `replay.txt`: production C++ phase replay on the two successes; not a prediction of new motion.
- `validation.txt`: five native suites, 21 Python tests, syntax/whitespace and build. Text log trailing spaces trimmed; raw USB `.serial` unchanged.
- `preflash.serial`, `postflash.serial` and summaries: fresh disarm, calibration/trim and device health.
- `flash.txt`: OpenOCD application-only write and successful verification.
- `record_trial.py`: bounded passive observer plus descriptive note; never arms or moves the robot. Its completed `first-trial.serial` captured the actual attempt and save confirmation. Onboard capture does not depend on this host observer.

First physical run reported successful: 1,241 complete samples, 4.600 s settled, reduced recoil versus latest baseline. See `trial-metrics.json`, `trial-analysis.txt`, `trial-comparison.png`, `trial-config-diff.json`, `trial-device-checks.json` and `analyze_trial.py`. Raw transfer is in `../../telemetry_logs/bal_20260919_151011_confirmed-recoil-release-success.serial`. The two-success baseline is preserved in `artifacts/balance-startup-recovery/`; installed application is unchanged after this run.
