# V13: keep returning after low-torque contact

[Physical failure](../v12-contact-stop-20260921T020520Z/README.md) reproduced a
v12 deadlock: confirmed support followed by near-zero holding torque caused
constant arm targets and the progress timeout. The only runtime policy change
is in fast Descending: if either arm reports less than 0.2 Nm above 15 degrees,
use the established 0.24-rad/s return while forward body rate is no faster than
12 degrees/s, otherwise pause. The existing +4-degree/s backward pause applies.

The maximum fast target/cap, smooth rate easing, 0.06-rad target-update lead,
two-arm catch confirmation, precontact motion, global/support-loss/time limits,
flat dwell, final retraction, normal mode and installed fast v2 stand-up remain.
The fallback limits target advance, not instant motor catch-up to a pending
target; pending advance remains bounded by the existing 0.06-rad limiter.

## Verification

Run `python3 evidence/balance-lower/fast-return-v13/screen.py` from this checkout.
[Summary](summary.json), [fast model](fast-simulation.json),
[low reported torque stress](low-torque.json), [nominal trace](nominal.csv).

- All 329 normal model results remain identical. Fast retains all 258
  completions and all 71 rejection outcomes/reasons versus installed v12.
- Forty new cases preserve real modeled contact forces but reduce reported
  torque after support, either immediately or after 0.8 seconds of fast motion.
  V12 completes 21/40; v13 completes 40/40. These are sensitivity tests, not an
  identified sensor/contact model or physical acceptance.
- The failed run's replay holds the old targets constant. V13 starts returning
  at 19.810 s, 20 ms after support. Recorded sensors remain parked, so both
  replays eventually time out: no future physical completion is inferred.
  One extra stationary 20 ms frame crosses the strict timeout because saved
  snapshot timestamps differ slightly from the internal control tick.
- Every changed target satisfies the 0.06-rad lead limit. Maximum held-target
  error is 0.061 rad when the measured arm moves 0.001 rad during the existing
  backward-rate pause; this is retained explicitly, not rounded away.
- Six normal recorded command replays are identical. Native tests include
  the physical low-torque stall, full pending lead followed by weak load,
  rapid-fall pause, backward pause and existing fault/landing checks.

Schema 13 identifies the corrected lowering policy with unchanged 240-byte
rows and historical exports. [Current state](../../../docs/progress/CURRENT.md)
records combined checks, frozen identity and actual deployment status. Physical
completion and speed still require the operator trial; no autonomous motion.
