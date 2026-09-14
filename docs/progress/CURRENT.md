# Current state — September 13, 2026

- **Objective:** prepare a documented balance firmware candidate and reliable telemetry workflow, then test after Austin attaches the robot.
- **Workspace:** `/Users/austinmcchord/Development/Hopscotch`, root checkout, branch `codex/balance-review-ready`; baseline `e8b1280`. Local changes are committed; nothing pushed.
- **Result:** firmware compiles; control-math/USB integrity tests pass. Candidate and rebuilt baseline rollback are in `artifacts/balance-candidate/`, with exact commit, binary hashes and build versions in `manifest.json`.
- **Evidence:** all 112 historical logs plus five photos and sampled frames across seven videos reviewed. [Findings and changes](../BALANCE_REVIEW_2026-09.md), [test procedure](../BALANCE_TESTING.md), [validation output](../../evidence/balance-review/validation.txt). The established append-only journal is `telemetry_logs/TUNING_HISTORY.md`.
- **Known uncertainty:** post-ramp surge remains unresolved. Approximate simulation does not show better stand-up reliability. Fast-task IMU changes, arm arrival gates and full v2 device telemetry need hardware validation. Same hardware/calibration; mounting movement can shift sensor angle.
- **Physical state:** no firmware flashed, no calibration changed, no device connected/operated for this preparation.
- **Next action:** Austin attaches USB with both drive/arm switches low. Back up the device's log, settings and calibration before firmware-only upload; verify disarmed sensor/motor state before the first 30-second stand-up capture.
