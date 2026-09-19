# Current state — September 19, 2026

- **Objective:** Austin explicitly authorized CH1 steering / CH2 forward-back while balanced and deployment for a physical test.
- **Workspace:** root `/Users/austinmcchord/Development/Hopscotch`, branch `codex/balance-review-ready`, clean baseline 39720e9 before this task.
- **Implemented:** 400 ms centered/calm unlock after startup, ±1 rad/s forward request through the velocity loop plus inner feedforward; ±0.5 rad/s differential steering; gradual speed changes; center to stop/hold new position and heading; loss/large-error pause requires centered reacquisition. CH7 HIGH suppresses ground driving before tip-up. Existing balance/catch/calibration preserved; trim learning requires rest.
- **Telemetry:** schema 3, 236-byte samples / 1,416,000-byte PSRAM buffer, 120 seconds retained. Pilot intent/state added; old v2 prefix/layout remains readable with unknown pilot fields.
- **Validation:** six native suites, 23 Python tests, syntax/whitespace and firmware build passed. All 324 neutral model trajectories identical to installed source; gentle movement/input-loss screens add no new model falls. Model cannot establish real handling/turning/braking, and some cases never unlock. Physical test pending.
- **Candidate image:** 1,141,584 bytes, SHA-256 `83d50ac277799094e58395cab3a9296e66c27718226da847a0442eea5a4c7c7d`; flash use 1,141,221, static RAM 50,800.
- **Still installed / restoration:** successful recoil-release source `1d80257e5609a173d9bbe104997eab1f9d4c7341`, SHA `5d3465e0269f9df19065d1583de35d093b46f947fdd27e49ef0de0bbf28d4dd0`, `artifacts/balance-recoil-release/`. Latest 1,241-row successful log safely archived/pushed, learned trim 3.57°, last observed disarmed.
- **Records:** [standing-drive findings](../BALANCE_STANDING_DRIVE_2026-09.md), `evidence/balance-standing-drive/`, `telemetry_logs/TUNING_HISTORY.md`.
- **Next:** fresh disarm/health/pending-save check, application-only authorized flash, stationary health and old-log compatibility download, then passive observation for a short user-driven test. Never tool-arm or move the robot. Preserve baseline for restoration without `--rollback`.
