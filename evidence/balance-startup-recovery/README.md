# Startup recovery evidence

The selected implementation is **early wheel-feedback learning**, in production source and `final-*` results. All prior arm/fixed-angle alternatives were unflashed development experiments.

- `final-validation.txt`: consolidated five native suites, 19 Python tests, syntax/diff and firmware build.
- `validate_final_model.py`, `final-model-comparison.*`: final source-derived 324-case comparison, including all per-case results and configurations.
- `wheel-replay.txt`: production C++ detector on the two corrected-unit physical logs; `plot_detection.py` and `detection-replay.png` illustrate the latest run. These are replay, not new physical outcomes.
- `rejected-early-arm.patch`, `rejected-arm-native-tests.cpp`, `arm-screen-simulator.py`, `candidate-config.json`, `model-comparison.*`, `response-screening.*`, `validation.txt`, `replay.txt`: **rejected early-arm prototype**, preserved before replacing it. `candidate-config.json` is that prototype's frozen screening input, not the selected firmware config.
- `wheel-screen*`, `wheel-release*`, `wheel-final*`, `compare_wheel_release.py`: intermediate wheel-response/boost-duration/hold-position screens; frozen simulators preserve the tested variants. Final selected source produces the same 86 early/60 later failures as the 800 ms + hold screen.
- `implement_wheel_*.py`: development transformation records against the restored capture-correction source; not idempotent build steps and not needed to reproduce final firmware.

Definitive physical findings and limits: `docs/BALANCE_STARTUP_RECOVERY_2026-09.md`. Frozen source/image identity: `artifacts/balance-startup-recovery/manifest.json`. New device checks will be added only after an actual flash.
