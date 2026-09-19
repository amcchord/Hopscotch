# September 19 successful stand-up repeat

Austin reported another very successful stand-up. The first read-only inspection found drive disarmed, arms armed, and 1,303 samples waiting in RAM. After Austin lowered both switches, fresh status confirmed both groups disarmed and `pending save: no`. The full new recording was then downloaded; no old stored file was mistaken for this run.

- `pending-save.serial/.txt`: initial diagnostic and reason download waited.
- `save-observer.serial`: passive wait before operator disarm; it expired without seeing the save.
- `disarmed.serial/.txt`: verified post-disarm state, healthy sensors/motors and unchanged calibration.
- `download.txt`: complete retrieval and checksum validation report.
- `analysis.txt`, `metrics.json`, `comparison.png`: first success versus repeat, with matched metrics/vertical scales.
- `analyze_repeat.py`: reproduces the metrics and figure using the new CSV path as its argument.
- `device-checks.json`: saved state and run identity, including exact checksums and metadata comparison.

The complete CSV and byte-exact transport are `telemetry_logs/bal_20260919_143953_early-recovery-repeat-success.csv` and `.serial`. Both successful logs identify the same build timestamp and controller settings; the learned stored trim changes from 2.4380° to 3.1650° at run start. This is consistent with the unchanged last-flashed firmware, not a fresh cryptographic readback of the application.

No firmware, gains, calibration or motor commands were changed. The existing learner stored 3.51° after this run. All USB readers closed after retrieval. Findings: `docs/BALANCE_STARTUP_RECOVERY_2026-09.md`.
