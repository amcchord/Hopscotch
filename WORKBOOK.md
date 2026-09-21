# Hopscotch project map

ESP32-S3 firmware for a six-motor robot: four drive wheels, two arms, GX12/ELRS
control, experimental balancing, Wi-Fi telemetry and application OTA.

Start with [current state](docs/progress/CURRENT.md). The root checkout and
GitHub `main` are the integration baseline. Read only the records relevant to
the task; [the docs index](docs/README.md) routes deeper investigation.

| Work | Entry points |
| --- | --- |
| Stand-up, balance, lowering | `src/balance_controller.*`, `balance_tip_up.h`, `balance_lower.h`, `balance_math.h`, `balance_pilot.h` |
| Drive, motors, RC | `src/drive_controller.*`, `motor_manager.*`, `robstride.*`, `crsf.*`, `main.cpp` |
| Wi-Fi/OTA and dashboard | `src/web_server.*`, `network_safety.h`, `ota_progress.h`, `data/network.html`, `scripts/robot_wifi.py` |
| Radio display | `radio/SCRIPTS/TELEMETRY/hop.lua`, [radio guide](radio/README.md) |
| Architecture and hardware | [Firmware reference](docs/FIRMWARE.md), [protocol detail](docs/PROJECT.md) |

The 200 Hz balance task and control owner run on core 1; networking runs on
core 0. Only the control owner grants maintenance. [Durable decisions](docs/progress/DECISIONS.md)
explain the constraints. Hardware truth comes from versioned release/trial
evidence, not a branch name or a simulator result.

```sh
./scripts/check_project.sh       # complete offline checks + pinned firmware build
./scripts/build.sh               # build only; never uploads
.venv/bin/python -m unittest discover -s tests -v
./scripts/check_radio_telemetry.sh
```

The existing `.venv` supplies analysis packages; `.venv-pio` supplies PlatformIO.
See [development setup](docs/DEVELOPMENT.md) for a fresh checkout and checks.
Device commands and recovery live only in [the OTA guide](docs/WIFI_OTA.md).

`src/`, `data/`, `radio/`, `scripts/`, and `tests/` are active development.
`evidence/` and `telemetry_logs/` retain research data at stable paths because
analysis and tests refer to them. `docs/archive/` holds dated reports and old
journals. These stay in Git but are excluded from default searches.
`output/` is reproducible scratch output; `artifacts/` holds private retained
packages. Neither belongs on GitHub. Use the local artifact index for recovery.
