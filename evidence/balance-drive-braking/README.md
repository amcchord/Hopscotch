# Progressive braking v5 evidence

See [findings and candidate behavior](../../docs/BALANCE_DRIVE_BRAKING_2026-09.md). The installed v4 run was downloaded and both checksums validated; `info-before.json` identifies that firmware. Combined source `43b1967` is installed in app1 and verified by its ESP image digest. All six motors remained disarmed/healthy; the retained 2,981-sample CSV and wire export are byte-identical. Physical testing is pending.

![Recorded physical stops](physical-v4-stops.png)

![Illustrative source-backed model comparison](final-model-stop.png)

- `analyze.py`, `physical-v4-*`: 2,981-row physical run analysis and plots.
- `ramps.*` / `screen_ramps.py`: ramp-only deceleration screen.
- `capture-screen.*` / `screen_capture.py` / `rejected_stop_capture.h`: rejected early PD handoff; it is absent from production code.
- `boost-screen.*` / `screen_boost.py`: rejected strong additional braking.
- `mild-boost-screen.*` / `screen_mild_boost.py`: milder candidates; gain0.5/limit3 subsequently rejected by the broader disturbance screen.
- `model.py` / `pilot_bridge.cpp`: actual C++ helper bridge, parameter overrides for experiments; defaults use current source. Rejected boost/capture paths are absent from this production bridge.
- `final_screen.py`: exact frozen installed source versus candidate, including neutral/startup regression, forward/reverse, input loss, pushes, and recorded stick replay.

The final candidate uses progressive 8–20rad/s² reference braking and unchanged inner control. `final-stops.*` records the paired 192-case stop comparison. Archived `experiment_model.py`/`experiment_bridge.cpp` and `rejected-*.h` preserve rejected options outside production. Their extra brake defaults to zero unless a screen explicitly enables it; an optional arm cap reproduces that intermediate experiment. The ground-drive fix is integrated. Preserve private Wi-Fi credentials and application-only OTA; never copy the credential file between worktrees or deploy a build using example credentials. This worktree builds with an ignored configuration that adds an include path to the existing private root header. Credentialed binaries, build trees, caches, and that machine-specific configuration are not committed.

## Release verification

- `release-manifest.json`: source, whole-file and ESP image digests, capacity, private-configuration check and rollback artifact.
- `validation-combined.txt`: final 10 native / 27 Python / syntax / whitespace / pinned ESP32 build checks.
- `validation-lowering.txt`: 81 helper/contact-model cases rerun after integration; separate lowering evidence documents its mechanical assumptions.
- `ota-preflight.json`, `ota-reconnected.json`, `ota-postflight.json`: original/running slot identities, disarmed health, retained-log integrity. The postflight radio is off at the operator's request; fresh switch-low observation is required after it is turned back on.
- `ota-upload.txt`, `ota-retry.txt`: unsuccessful connection/response-timeout attempts; original running image remained intact.
- `ota-slow-retry.txt`: successful application-only transfer at 1KiB per50ms with120s socket timeout, accepted after62.8s. Transmitter off near transfer end; cause of earlier failures remains unisolated.

No robot movement was initiated. Credentialed binaries, private header, dependencies and machine-specific uploader/build wrappers remain excluded from Git. The source-backed driving comparison predates integration of the separate lowering path; the screened braking helpers/config are unchanged. The full firmware scheduler, mechanical reach and physical combined behavior still require the operator trial.
