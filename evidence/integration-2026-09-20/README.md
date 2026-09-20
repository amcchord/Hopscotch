# Root integration verification — September 20, 2026

The control checkout merged deployed `codex/drive-braking` release record
`eb2bb23` into the published root documentation/history. Production `src/`,
`scripts/`, `tests/`, `platformio.ini`, `data/` and `radio/` match installed source
`43b1967` exactly. Only the shared progress files conflicted; the root's complete
append-only journal was retained, and CURRENT was rewritten from the verified
[deployment manifest and postflight](../balance-drive-braking/README.md).

- `validation.txt`: 10 native executables, 27 Python tests, syntax/whitespace and
  full ESP32 build passed in the root's pinned Python 3.13 PlatformIO environment.
- `radio.txt`: radio C++/Lua fixtures and behavior checks passed.
- `dashboard.txt`: syntax and eight independent SHA-256 checks passed.
- Private release file size, whole-file hash and ESP embedded digest match the
  release manifest and recorded running image. No credentials or binary copied.
- The retained 2,981-row raw transfer validates, reconstructs the exact archived
  CSV, and both file hashes match postflight evidence.

The integration task did not access, reflash or move the device. No additional
model rerun was needed: the production/model source matches the release owner's
completed checks. This build is verification, not a new installed image.
The latest handset Lua v3 work remains separately published on
`codex/radio-telemetry`; it was not merged into this firmware-source snapshot.
