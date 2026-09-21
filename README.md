# Hopscotch

Firmware for an ESP32-S3 robot with four drive wheels, two arms, RadioMaster
GX12/ELRS control, experimental balancing, Wi-Fi telemetry and application OTA.

**Start development:** [project map](WORKBOOK.md) → [current state](docs/progress/CURRENT.md).
The current-state record distinguishes source code from the last verified robot
installation and identifies unresolved physical behavior.

```sh
./scripts/check_project.sh   # offline tests and pinned firmware build
./scripts/build.sh           # build only
```

[Development setup](docs/DEVELOPMENT.md) · [Hardware and architecture](docs/FIRMWARE.md) ·
[Wi-Fi/OTA and recovery](docs/WIFI_OTA.md) · [Physical test procedure](docs/BALANCE_TESTING.md) ·
[Radio display](radio/README.md) · [Documentation index](docs/README.md)

Source, dashboard, radio, scripts and tests are the active project. Historical
reports, experiment evidence and telemetry remain versioned but are excluded
from default searches; read specific records through the documentation index.
Credentials, configured binaries, private backups and generated output stay local.

MIT licensed. See [LICENSE](LICENSE).
