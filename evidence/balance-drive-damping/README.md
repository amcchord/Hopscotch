# Driving damping evidence

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../docs/progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../docs/WIFI_OTA.md) and the [current test guide](../../docs/BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

See [findings and firmware changes](../../docs/BALANCE_DRIVE_DAMPING_2026-09.md).

- `identify.py`, `observed-frequency-ratios.json`: windowed ratios from the archived physical v3 trial, excluding the terminal contact bout.
- `model.py`, `pilot_bridge.cpp`: actual C++ pilot, acceleration controller, fast filter and graded/emergency arm helpers coupled to the optional uncertain planar plant in `scripts/balance_sim.py`.
- `tune.py`, `tuning-screen.json/.txt`: exploratory feedback candidates. These are screening results, not physical tests or causal plant identification.
- `braking.py`, `braking-tradeoff.json`: nominal stronger speed/braking alternatives that were not selected.
- `screen.py`, `screen.json/.txt`: final source hashes, frozen `5a07eba` production helper comparison, 324 exactly matching neutral cases and 432 revised-plant profiles. Three cases per profile never unlock driving.
- `plot.py`, `model-comparison.png`, `nominal-response.json`: selected nominal traces; quiet stationary ripple differs from actual hardware.
- `validation.txt`: consolidated native/Python/syntax/build gate.
- `attached.serial/.txt`: initial read-only status, both groups disarmed; motors no longer communicating.
- `preflash-download.txt`: preservation download of the existing physical v3 trial before programming.

Run Python scripts with `.venv/bin/python` from the repository root. Run simulation scripts sequentially because they compile the same bridge library under ignored `output/drive-damping/`. They never open USB or command motors. Figures are offline model comparisons, not predicted physical stopping distances. Raw `.serial` captures are retained byte-for-byte.
- `preflash.serial/.txt`, `postflash.serial/.txt`, `upload.txt`, `device-checks.json`: exact application-only upload/readback proof, retained disarmed/calibration/IMU state, and explicit limitation that powered motor health remains unverified (zero replies).
