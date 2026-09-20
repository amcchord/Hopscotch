# Standing drive response v2

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../docs/progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../docs/WIFI_OTA.md) and the [current test guide](../../docs/BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

See `../../docs/BALANCE_DRIVE_RESPONSE_2026-09.md` for cause, changes, limits and operator procedure.

- `experiment.py` / `experiments.json` / `experiments.txt`: frozen v1 model and production pilot, longer full-forward/stop/reverse and RC interruption, five candidate parameter sets. Rejected 3 rad/s candidates were never flashed.
- `screen.py` / `pilot_bridge.cpp` / `screen.json` / `screen.txt`: selected production C++ helper and gain function, 324 exactly unchanged neutral cases, 54 cases for each shorter drive/input-loss profile. Nominal JSON files are decimated simulated traces, not measurements.
- `validation.txt`: six native suites and 23 Python tests, syntax/whitespace and firmware build.
- Actual first-drive telemetry, health checks, plot and diagnosis are preserved in `../balance-standing-drive/` and `../../telemetry_logs/`.

Model has no yaw/contact/slip and differs from actual stationary behavior. Do not interpret no-new-failure counts as proof of physical handling or reliability. An initial simulator check used C++ float math for neutral P against Python double, then amplified those rounding differences in its unstable cases. The final adapter calls the new moving correction only during movement and retains the original neutral numerical path; the production controller uses unchanged stationary gain/formula.

`preflash`, `flash`, `postflash`, `device-checks.json` confirm application-only verified release, disarmed/healthy state and retained calibration. `compat-download.txt` / `compat-check.json` verify old v1 samples/config are unchanged through the new exporter. `record_trial.py` passively observes the next user test for up to four minutes.
