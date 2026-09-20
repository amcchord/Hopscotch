# CH11 lowering v5 — forward motion during arm deployment

**Historical v5:** its physical trial ended before catch with a preparation wheel-speed abort. [Installed v6](BALANCE_LOWER_V6_2026-09.md) adds the earlier handoff; see the [recorded trial](../evidence/balance-lower/trial-v5-20260920/README.md).

September 20, 2026. Installed source `17c499c` for operator testing; consult
[current state](progress/CURRENT.md) for the installed identity and outcome.
The previous [physical v4 attempt](../evidence/balance-lower/trial-v4-20260920/README.md)
raised its target 4.465 degrees backward during 6.2 seconds of preparation. The
body followed it, then rebounded at 105.041 deg/s when the arms struck.

V5 caps the preparation balance target at the measured starting body tilt,
so moving the arms cannot command that backward lean. Arms deploy promptly
at a 4 rad/s cap and hand off while moving, instead of waiting upright for a
stationary pose. The nominal model departs during preparation and reaches
handoff 0.52 seconds after preparation starts; this is not a measured hardware time.
A backward excursion exceeding 1.5 degrees during preparation stops the attempt.

The catch initially parks at 1.85 rad from Forward. Only after six measured
forward degrees may it probe farther, at 0.5 rad/s. First load must coincide
with a measured forward fall and body deceleration. Both arms reverse at
contact; independent two-arm support unlocks continuous return to Forward.
Completion still requires level, quiet body/wheels and measured Forward arms.

The brief preparation uses the existing balance wheel controller with its
setpoint capped. Its wheel velocity can be retained through the handoff up
to 6 rad/s, approximately 0.33 m/s for the assumed 55 mm wheel radius. The old 2 rad/s
sender bound would abruptly remove that measured velocity. Supported stopping
is 3 rad/s² to avoid a long coast. Arm target lead is bounded at 0.24 rad.
The 65 deg/s sampled body limit, 100 ms owner/feedback limits, load gates, deadlines
and completion requirements remain. Persistent wheel motion opposite to its
command now aborts. These changes apply only to the lowering maneuver.

[329-case paired screen](../evidence/balance-lower/forward-preparation-v5/README.md):
261 → 253 completions, 43 improvements and 51 regressions. Most regressions use
the widest unmeasured pivot geometry. All 72 trial-informed contact cases still
complete; delayed contact improves 8 → 15 of 24, including the v3 delayed-impact
example. All 11 injected failures reject completion. This is an experimental
manual-test candidate, not proof of successful lowering on the robot.

The combined source includes the other task's [fast tip-up start fix](FAST_TIP_UP_2026-09.md):
fresh motor feedback is collected after blocking wheel setup, the start waits
for a quiet pose, and the dashboard shows a refusal reason. Ordinary ground
and standing driving remains unchanged.

## Next trial

Use the existing clear-area/spotter procedure in [balance testing](BALANCE_TESTING.md).
Start physically flat with arms Forward, observe both arming switches LOW after
reboot, then arm normally. Use CH6 LOW for the first lowering trial to separate
it from fast standing. Once standing has settled and sticks are centered, pulse
CH11 once. Expect forward movement during arm deployment, contact followed by
arm return, and a level finish with arms ahead. Do not assist with CH12 during
the recorded trial. Disarm both groups and download the log before another run;
only the latest attempt is retained on the robot.

New logs use schema 5 with the same 240-byte samples and unchanged header layout. The version
identifies this policy because all 16 feature bits were already allocated.
Older schema 4/v4 exports retain their exact recorded metadata and checksums.
Use the existing [single-command frozen-package OTA procedure](WIFI_OTA.md).
