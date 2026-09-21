# Faster CH6 supported return v12

The [latest successful physical run](../fast-lower-speed-20260921T013959Z/README.md)
took 5.910 seconds from confirmed support to first flat hold. Austin requested
roughly three times faster motion after contact, preserving the successful
stand-up. V12 changes only CH6 HIGH supported return; LOW/center retain v9 normal
lowering and installed fast v2 stand-up stays unchanged.

| Fast supported policy | Installed v10 | V12 |
| --- | ---: | ---: |
| Maximum arm target ramp, rad/s | 0.60 | 1.80 |
| Motor speed cap, rad/s | 0.75 | 2.25 |
| Body-rate target-advance pause, degrees/s | 20 | 50 |
| Maximum pending arm target travel, rad | 0.24 | 0.06 |
| Floor taper, body degrees | 35→15 | 20→5 |

The existing 600 ms acceleration blend starts only after both contacts and
the 80 ms support confirmation. Fast target motion now eases linearly from
full speed at 60% of the current forward-rate budget to zero at its limit.
Above 15 degrees it pauses immediately if either arm has less than 0.2 Nm
measured load. Tight target lead limits motion still pending after a pause.
The backward pause, 65-degree/s global guard, support-loss rejection, wheel
limits, timeouts, catch, flat confirmation and final retraction remain.

Exploratory naive tripling failed nominal support loss and delayed-response
motion limits. Smooth easing alone still lost support in 34 previously complete
cases. The selected load-aware pause addresses that mechanism without weakening
the fault guards. The selected settings are an operator-test candidate, not a
validated contact model or a guarantee of physical speed/reliability.

## Paired screen and recorded replay

Run `python3 evidence/balance-lower/fast-return-v12/screen.py` from this checkout.
It executes the production C++ policy through the unchanged contact model and
compares all retained v10 cases. [Summary](summary.json),
[fast results](fast-simulation.json), [nominal trace](nominal.csv),
[rebound](trial_rebound_stress.csv), [delayed response](trial_v3_delayed_impact.csv).

- All 329 normal model results are identical, including commands' resulting
  trajectories/timing and fault reasons. All 258 normal completions remain.
- Fast mode retains all 258 completions and rejects the same 71 other cases;
  the contact-loss injection now hits the unchanged global motion guard before
  the support-loss timer. This one changed fault reason is retained explicitly.
- Preparation/catch/support-entry transitions are identical in all cases.
- Among common completions, modeled support-to-first-flat speed ratio has
  median 1.996 and range 1.169–2.648. Nominal supported time is 4.44→2.30 s;
  whole maneuver 8.08→6.04 s. Three times maximum arm speed does not mean three
  times physical descent speed; no physical ratio has been measured yet.
- Five archived recordings replay identical normal commands. Fast commands
  first differ only after confirmed support. Fixed recorded sensors cannot
  predict physical motion after that divergence.

Native tests cover smooth rate easing, unloaded-arm pause, bounded target lead,
mode latching, unchanged catch, floor taper/re-entry, contact loss, global limits
and final dwell/retraction. Schema 12 identifies lowering v12 and installed
fast stand-up v2 with unchanged 240-byte rows. Schema 11 remains readable as the
previous unflashed fast-v3 experiment; historical v10 export text is unchanged.

See [current state](../../../docs/progress/CURRENT.md) for combined validation,
exact package and installation status. No autonomous motion is requested.
