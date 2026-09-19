# Acceleration drive / planned arms evidence

The actual physical trial in this directory is **installed response-v2**, not the new candidate. See [findings](../../docs/BALANCE_DRIVE_AGILITY_2026-09.md).

- `attached.serial/.txt`: read-only USB retrieval status; IDLE/disarmed, no live motor/receiver traffic.
- `download.txt`: validated 1,666-sample download; raw transfer/CSV are in `telemetry_logs/`.
- `analyze_v2.py`, `v2-trial-metrics.json`, `v2-delayed-response.png`: reproducible physical-trial analysis.
- `screen.py`, `pilot_bridge.cpp`, `screen.json/.txt`: final comparison with installed source `d1ae97d`; invokes actual C++ helpers, records production source hashes. Model outputs are not physical tests. Run from root with `.venv/bin/python evidence/balance-drive-agility/screen.py`.
- `validation.txt`: consolidated seven native executables, 25 Python tests, syntax/whitespace and firmware build.
- `rejected-*/`: never-flashed prototypes. One/two-sided velocity-reference governors, stronger damping and another reference-controller variant added model falls. `rejected-slow-braking` added none but stopped too slowly. Acceleration prototypes with full speed error driving recovery arms, or a very small combined balance-acceleration cap, were rejected.
- `accel-without-planned-arms/`, `acceleration-experiment.json/.txt`, `acceleration_experiment.py`: earlier Python exploratory acceleration-controller trials. The script is tied to `rejected-reference-v3-source/balance_sim.py`; it does not execute final production helpers or final recovery-priority coordination. Older variant outputs do not all retain a runnable per-variant script. Use final `screen.py` for release evidence.

The preceding `balance-drive-response/first-trial.serial` observer is preserved but contains idle status only; it expired before the downloaded physical run. Raw serial files are byte-exact and excluded from Git newline normalization.

Upload evidence: `preflash.*`, `reconnected-preflash.*`, `upload.txt`, `postflash.*` and `device-checks.json` identify the programmed source/image and powered disarmed health. USB was briefly absent during power preparation; no flash started until a new preflight passed. `compat-download.txt` and `compat-check.json` establish historical schema-3 compatibility. `record_trial.py` is a four-minute passive observer with a descriptive note; it never commands motors. Its future capture is not part of the completed v2 physical trial.
