# Standing drive candidate evidence

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../docs/progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../docs/WIFI_OTA.md) and the [current test guide](../../docs/BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

See `../../docs/BALANCE_STANDING_DRIVE_2026-09.md` for the operator procedure, exact control changes, verification and limitations. Source starts from 39720e9 (installed recoil-release source 1d80257). The application was flashed and physically tested; Austin requested faster forward/back response after all directions worked. See the findings document and trial-metrics.json.

`screen.py` compiles the actual production `BalancePilot` class via `pilot_bridge.cpp`, then compares neutral commands to the frozen installed model and exercises forward/stop/reverse and input loss. `screen.json` retains all movement cases; `screen.txt` is the summary. The nominal traces are model outputs, not observations of the robot. No yaw/contact/slip model; no claimed braking distance or reliability probability.

Rejected variants were never flashed:
- `rejected-no-feedforward`: velocity limit 2, acceleration 1, deceleration 1.5, turn 0.75/slew 1; no cruising feedforward; glide boost disabled while moving.
- `rejected-feedforward-no-glide`: same limits, cruising feedforward enabled, glide boost disabled while moving.
- `rejected-feedforward-glide`: same limits and feedforward, glide boost left enabled while moving.
- Final source: velocity limit 1, acceleration 0.5, deceleration 0.75, turn 0.5/slew 0.75; cruising feedforward enabled, glide boost disabled while moving/braking. Other controller gains unchanged.

`validation.txt`: six native suites and 23 Python checks, syntax/whitespace, firmware build. New telemetry layout is tested through its real C++ header; existing v2 files remain readable. Raw USB `.serial` evidence will stay byte-exact; human-readable command output has trailing spaces trimmed.

`preflash`, `flash`, `postflash`, `first-trial` and `posttrial` retain the release/run. Postflash was operator-armed; posttrial is verified disarmed. `download.txt` verifies 2,881 rows; the raw transfer and CSV are in telemetry_logs. `device-checks.json` records actual release state.
