# Startup recovery evidence

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../docs/progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../docs/WIFI_OTA.md) and the [current test guide](../../docs/BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

The selected implementation is **early wheel-feedback learning**, in production source and `final-*` results. All prior arm/fixed-angle alternatives were unflashed development experiments.

- `final-validation.txt`: consolidated five native suites, 19 Python tests, syntax/diff and firmware build.
- `validate_final_model.py`, `final-model-comparison.*`: final source-derived 324-case comparison, including all per-case results and configurations.
- `wheel-replay.txt`: production C++ detector on the two corrected-unit physical logs; `plot_detection.py` and `detection-replay.png` illustrate the latest run. These are replay, not new physical outcomes.
- `rejected-early-arm.patch`, `rejected-arm-native-tests.cpp`, `arm-screen-simulator.py`, `candidate-config.json`, `model-comparison.*`, `response-screening.*`, `validation.txt`, `replay.txt`: **rejected early-arm prototype**, preserved before replacing it. `candidate-config.json` is that prototype's frozen screening input, not the selected firmware config.
- `wheel-screen*`, `wheel-release*`, `wheel-final*`, `compare_wheel_release.py`: intermediate wheel-response/boost-duration/hold-position screens; frozen simulators preserve the tested variants. Final selected source produces the same 86 early/60 later failures as the 800 ms + hold screen.
- `implement_wheel_*.py`: development transformation records against the restored capture-correction source; not idempotent build steps and not needed to reproduce final firmware.

Definitive physical findings and limits: `docs/BALANCE_STARTUP_RECOVERY_2026-09.md`. Frozen source/image identity: `artifacts/balance-startup-recovery/manifest.json`. New device checks will be added only after an actual flash.

## First reported successful physical trial

`first-success-*` contains the checksummed download report, posttrial status, metrics and inspected comparison figure. `analyze_first_success.py` reproduces the comparison. `tested-firmware-identity.json` records the exact tested source/application hash for the unchanged firmware. The 1,203-row CSV and byte-exact USB transfer are in `telemetry_logs/bal_20260914_233101_early-recovery-first-success.*`.

`first-trial.serial` is only the earlier passive observation window: it expired before the operator started and does not contain the successful attempt. Do not confuse it with the complete onboard CSV/raw download. All serial readers are now closed.
