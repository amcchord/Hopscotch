# Lowering v5 + fast start fix release

Prepared for operator testing from the integration owner's contained checkout.
[Motion behavior](../../docs/BALANCE_LOWER_V5_2026-09.md),
[physical v4 evidence](../balance-lower/trial-v4-20260920/README.md),
[paired model and limits](../balance-lower/forward-preparation-v5/README.md),
[source preservation](../balance-lower/forward-preparation-v5/source-preservation.json),
[combined validation](checks.txt), [dashboard validation](dashboard-checks.txt).

Fast startup owner 9771bee is integrated as 3f3bc37. Ground/standing driving
policies remain unchanged; the lowered setpoint cap is active only in CH11
preparation. No other task's checkout is modified. Historical log metadata is
preserved; schema 5 records the new behavior without increasing sample bytes.

The exact frozen manifest and deployment record will identify the installed
application, slot, health result and saved-log comparison. Keep the exact v4
package for rollback; do not rebuild recovery bytes or upload the filesystem.
No automatic boot rollback. No autonomous motion is initiated by installation.
