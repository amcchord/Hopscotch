# Development setup and validation

The root checkout is the integration workspace. Read [AGENTS](../AGENTS.md)
and [CURRENT](progress/CURRENT.md) before changes.

On this workstation `.venv` and `.venv-pio` are already configured. A fresh
checkout needs Python 3, a C++17-capable `clang++`, Node.js and Lua/luac. Create
local environments without copying dependencies or credentials from a worktree:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
python3 -m venv .venv-pio
.venv-pio/bin/python -m pip install platformio==6.1.19
```

The recorded analysis environment uses Python 3.14. Dependencies below match
the consolidation workstation; embedded dependencies are pinned in
`platformio.ini`. Configure your own ignored `src/network_secrets.h` using the
[configuration guide](WIFI_OTA.md#local-configuration-and-access). Keep an
existing configured header intact. Configured builds contain credentials.

Run `./scripts/check_project.sh` from a clean source checkout for integration:

- 12 native motion, motor, RC, network-safety and OTA-progress executables;
- Python test discovery, Python/shell syntax and diff whitespace checks;
- the pinned PlatformIO ESP32 build, including checked Async TCP patches;
- radio native fixtures, Lua syntax, display and logging tests;
- dashboard syntax and eight SHA-256 vectors;
- actual OTA methods and pinned TCP polling in the host transport harness.

This script makes no robot requests. The compiler emits existing Arduino macro
redefinition warnings; inspect other warnings/failures rather than hiding output.
Capture a full log under `output/` and record only its result in the journal.

For focused changes, use the relevant existing test or
`./scripts/check_radio_telemetry.sh`. `HOP_LUA_BIN` / `HOP_LUAC_BIN` can select
the retained Lua 5.3/32-bit runtime for radio checks. Firmware or model work may
also need a specific recorded replay; do not indiscriminately rerun all historical
experiments. Deferred candidate scripts require the historical source revision
specified in their evidence README.

Use `rg --no-ignore <pattern> evidence/<case>` to deliberately search archived
data. Default `rg` skips archives and raw captures via `.ignore`; this does not
affect Git tracking or direct file reads.
