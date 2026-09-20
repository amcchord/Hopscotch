# First retained fast stand-up trial

Downloaded directly by the release owner from installed source `17c499c` after
Austin reported forward roll-away. [Archive checksums](archive.json) and
[disarmed download state](preflight.json). The 236-row / 4.750-second run has
fast bit 128 set and ends `bailout_angle_error`. Original CSV/wire files remain
at `telemetry_logs/bal_20260920_fast_tip_v1_wifi.*` in this checkout.

The fast-tip-up owner was given read-only access to these files for diagnosis;
no raw log was copied between checkouts. Capture was quiet, followed by forward
roll during arm support release. The [derived analysis and correction](../roll-away-review/README.md) are
integrated in the [verified combined update](../../lowering-v6-integration/README.md).
