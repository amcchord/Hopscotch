# Current state — September 19, 2026

- **Objective:** CH1 steering / CH2 forward-back added and physically tested. Austin reports all directions work but forward/back has a long delay; explicitly requested faster, more responsive driving.
- **Workspace:** root `/Users/austinmcchord/Development/Hopscotch`, branch `codex/balance-review-ready`.
- **Installed before follow-up flash:** first-drive source `90f07e83e3472cebbc5ca2646e585bfbce20f2d7`, app `83d50ac277799094e58395cab3a9296e66c27718226da847a0442eea5a4c7c7d`, package `artifacts/balance-standing-drive/`.
- **Archived first test:** 2,881 rows/57.601 s, schema3/features127, binary `0x90FA4B79` / USB `0xED14696F`. All directions work; full forward/reverse delay to 0.25 rad/s 6.42/6.84 s, followed by roughly 3.1 rad/s peaks against ±1 requests. Receiver/unlock stayed responsive. Disarmed, saved, trim 3.89°, six motors healthy, no IMU/CAN fault.
- **Response v2 ready:** moving low-error P 1.0 vs0.462 to overcome near cancellation with cruising feedforward; maximum requested speed2, turn1.5, acceleration1.5, braking2, turn slew3 (wheel rad/s units). Stationary/stand-up gains and safety gates unchanged. Schema3/features255 preserves old-log metadata.
- **Validation:** six native suites, 23 Python tests, syntax/whitespace/build; 324 neutral model trajectories unchanged. Shorter and longer drive/input-loss screens add no new failures at chosen limits; 3 rad/s variants rejected. Model differs from actual motion and cannot prove physical benefit.
- **Candidate app:** SHA `b28793cdd8d2b45cdfbcb344cc594ba85ba79fadcdb2e373bca321a1d86b172b`, 1,141,840 bytes, flash1,141,469/staticRAM50,800.
- **Records:** [response findings](../BALANCE_DRIVE_RESPONSE_2026-09.md), [first-drive findings](../BALANCE_STANDING_DRIVE_2026-09.md), both evidence directories, `telemetry_logs/TUNING_HISTORY.md`.
- **Next:** freeze source/package, fresh disarmed preflight, authorized application-only upload, stationary health/old-v1 export verification, short operator test. Never tool-arm or move. Exact v1 and recoil-release packages retained; do not use `--rollback`.
