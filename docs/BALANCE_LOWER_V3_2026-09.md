# CH11 lowering v3: first-impact yielding

**Historical:** the v3 physical trial stopped upright on loaded arms; see
[v4 continuous return](BALANCE_LOWER_V4_2026-09.md) and the
[trial record](../evidence/balance-lower/trial-v3-20260920/README.md).

The installed combined image is source `8449ddb`, with the lowering fix, OTA
transport update and cooperative log export. See the
[deployment record](../evidence/ota-lowering-v3/README.md),
[model evidence](../evidence/balance-lower/first-impact-v3/README.md)
and [current state](progress/CURRENT.md) for installation status. Do not identify
the installed version by a filename or the preparation manifest's queued status.

The [physical v2 run](../evidence/balance-lower/trial-v2-20260920/README.md) did
initiate a forward fall. Its arms reached the floor while still moving rapidly;
one arm stopped on load while the other kept advancing for another frame. The
body then rebounded backward. The trace ends at `lower_wrong_direction`.

V3 makes a small timing change and changes the response to impact:

- Prepare to 1.25 rad instead of 1.30, with the same slow approach and measured
  settling requirement. Start the forward fall with 2 rad/s² wheel acceleration
  instead of 4. Ordinary balance/standing/ground driving are unchanged.
- Start the fast catch stroke only after at least 0.1° of measured forward
  travel and -1°/s body rate. An isolated tiny movement no longer starts it.
- Stop both forward strokes when either arm first reaches the existing 0.4 Nm
  contact threshold. The other arm must not continue pushing during the impact.
- Once body falling rate has slowed at least 1°/s from its measured peak, allow
  both targets to retreat up to 0.06 rad at 0.5 rad/s. The retreat budget is fixed
  at first contact and cannot restart. Each arm still needs its own contact
  evidence; both must be loaded and slow, with body deceleration, before descent.

Maximum catch reach/speed, wheel speed bounds, direction/rate faults and all
feedback/deadline checks remain. A fault stops the maneuver; it cannot guarantee
a safe landing from an unsupported fall. CH7 cancellation and the ordinary
disarm controls retain priority. CH1/CH2 driving still requires neutral after
control returns to ground drive.

The revised model includes moving-arm contact velocity and harder/faster contact
sensitivities. In 304 paired cases, completions rose from 231 to 248. Four cases
that previously completed now fault; asymmetric contact is still a limitation.
Some completed model catches reach 55.226°/s. This supports a restrained trial,
not a claim of graceful or mechanically safe performance. See the evidence for
the rejected aggressive timing changes and the model's unmeasured assumptions.

For the next supervised trial, verify the installed image, clear the arm sweep,
and use the existing catch restraint. Stand up normally, center both sticks,
wait for settled arm return, then pulse CH11. Watch whether both arms stop/soften
at first contact and whether the body continues forward into support rather
than rebounding. Record any catch, push, asymmetry or intervention. Disarm both
groups afterward, let saving finish, and download the run before another attempt.

New logs retain schema 4 / 240-byte samples, adding bit 8192 for combined flags
16383. Phase IDs and the fast lowering-rate field retain v2 meanings. Old v1/v2
exports retain their exact metadata; do not relabel a historical run as v3.

The same image includes OTA transport version 2: eligible uploads tolerate
brief receive gaps, dashboard streaming pauses during upload, and aborted
transfers expose progress/reason diagnostics. Transmitter-on hardware acceptance
is a separate disarmed test; see [OTA reliability](OTA_RELIABILITY_2026-09.md).
