# Successful run before faster supported return

Austin reported this run was perfect and requested only a roughly threefold
increase in laydown speed after the arms touch. Fresh idle/disarmed, finished
saving, healthy powered disabled motors and IMU were verified before export;
post-export checks passed. Installed source is still 45c1a94/app0, ESP digest
`a9ad12aedebfcd1f1aa1b39d3969f567196f357d60e7cbe249d4aa268510af61`.

[Archive identities](archive.json), [preflight](preflight.json),
[post-export](post-archive.json), [analysis](analysis.json),
[plot](lowering-success.png). Raw CSV/wire remain in this checkout at
`telemetry_logs/bal_20260921T013959Z_fast_lower_speed_wifi.{csv,wire}`;
1,192 samples, schema 10, lower_complete. Both CH6 fast choices were selected.

Supported return starts 16.785 s at 82.645 degrees and first reaches flat hold
at 22.695 s: 5.910 seconds. A brief return to Descending at 22.735 s is followed
by flat hold at 22.755 s, retraction at 23.360 s and completion at 24.009 s.
Total lower time is 8.508 s. Final tilt -1.783 degrees, rate -0.969 degrees/s,
Forward errors -0.027/+0.009 rad; wheels +0.018/-0.043 rad/s.

The supported body rate peaks at -53.052 degrees/s despite a nominal 20-degree/s
target-advance pause. Close arm tracking (about 0.046 rad maximum left error)
does not establish a safe higher body-rate limit. Keep the 65-degree/s global
guard and test speed with smooth rate easing and retained support.

Stand-up is the installed fast v2, with quiet capture at 3.005 s/83.523 degrees,
return at 3.425 s, and ramp complete at 5.265 s. No early recovery was needed.
The unflashed fast-v3 gain experiment was deferred in 6106e77 so the requested
lowering-only update preserves this successful stand-up. No motion was
initiated during archive/analysis; no raw logs were copied between worktrees.
