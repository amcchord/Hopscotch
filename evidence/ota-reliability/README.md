# OTA reliability candidate — validated locally, not installed

Source `67f37468ca615995ed5a37298e11b3d99ce379ff` on `codex/ota-reliability`,
based on installed lowering-v2 handoff `d3eb777`. The user explicitly separated
OTA reliability from the concurrent physical-lowering investigation. This task
owns only `worktrees/ota-reliability`; root shared records and the other
worktrees remain with their owners. **No device request, upload, restart,
configuration change or motion occurred during this work.**

The [change and hardware plan](../../docs/archive/2026-09/OTA_RELIABILITY_2026-09.md) address a
concrete mismatch: pinned HTTP connections expire after three seconds without
data, whereas OTA permits 15 seconds. Authenticated eligible uploads now have
a 20-second receive timeout, 15-second ACK timeout and TCP_NODELAY. OTA's own
15-second inactivity abort is unchanged. New dashboard WS offers pause during
upload; cached GET telemetry remains available with bounded failure diagnostics.
Real disconnects still abort the partial application. Transmitter-on failures
were TCP disconnects with continued ESP uptime, not controller reboots.

Only `src/web_server.cpp` and `src/web_server.h` differ in production files
from `d3eb777`. All motion, lowering, drive, configuration, build, dashboard and
radio sources remain byte-identical. The existing maintenance interlock, token,
full-file hash/length/ESP verification, inactive-slot writes and switch-low
rearm are retained. The Async TCP null-accept/UAF patches are unchanged.

## Completed checks

- [Combined checks](validation-combined.txt): 10 native executables, 37 Python
  tests, Python/shell syntax, whitespace and pinned configured ESP32 build pass.
  The existing event-core macro redefinition warning remains.
- [Radio](validation-radio.txt) and [dashboard](validation-dashboard.txt) pass.
- [Focused transport harness](ota-transport-test.txt): compiles the actual
  upload/abort/watchdog methods and pinned Async TCP poll method. Reproduces
  default failure after a four-second receive gap, verifies the candidate stays
  open through the same gap, and checks 15-second abandonment, late data after
  expiry, disconnect diagnostics, upload ownership, authentication/eligibility
  and final verification guards. Hardware flash and SHA functions are stubbed;
  the harness does not establish radio behavior or flash integrity on the robot.
- Source-preservation comparison passes. No motion-model rerun was needed for
  these networking-only changes. No repeated build/test cycle was run solely
  to freeze the identical validated artifact.

The fresh checkout bootstrapped its own dependencies via the pinned PlatformIO
runtime; no dependency trees, private headers or logs were copied between
worktrees. Its local `output/release.ini` adds only the existing root private
header's include directory to this checkout's normal configuration. All four
private configuration values were checked in the built image without exposing
them. The public [manifest](candidate-manifest.json) identifies the artifact:

- Application: 1,196,992 bytes.
- Whole SHA-256: `56b9517f3a1f7ee7e82f0898b9d4a6e95f533d317c3a6574e1a8b60acf3d8062`.
- ESP digest: `08e46a1d6188d853c576d6f86ad2aee4a1205d9ec97b1d26f80b858dca68d982`.
- Private binary/ELF/manifest: `worktrees/ota-reliability/artifacts/ota-reliability/candidate/`
  relative to the project root. Credentialed artifacts are ignored and unpublished.

At handoff the installed app0 remains source `dd74154`, ESP digest
`7668a0215df34b7e5c0030a23705ba8016343260d1e6bab7aacfe202897c7190`; that identity
comes from the lowering task's verified deployment and was not queried here.
Its exact configured package and previous rollback package remain intact.

## Next release

Coordinate with the lowering owner before device access. Prefer integrating
this networking commit with the next lowering candidate to avoid overwriting
newer motion work or adding unnecessary installs. A changed combined source
requires its integration build/check once; this networking artifact is only
valid for its recorded source. Verify transmitter-on OTA on hardware after the
new receiver implementation is installed, including a four-second receive gap,
saved-log preservation and disarmed image/health handoff. Until then, the
transmitter-off procedure remains the demonstrated path and this candidate
must not be described as a proven hardware fix.
