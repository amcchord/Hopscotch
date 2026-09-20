# CH11 lowering v4: return the arms after contact

The v3 trial caught the robot near upright, then faulted on the loaded rebound.
It never reported `lower_complete`. Austin's subsequent CH12 press brought the
robot flat. [Trial evidence](../evidence/balance-lower/trial-v3-20260920/README.md)
ends before that manual movement.

V4 reverses both arms toward their calibrated Forward pose at first contact.
One-arm contact allows a small retreat; both contacts unlock the full return.
Return speed is limited and eases as the body falls forward. Brief supported
rebound can settle while the arms continue returning; unsupported or excessive
motion still faults. It then continues supported descent until the body is level
and both arms are measured at Forward. Ground driving resumes only after that
completion and a neutral-stick handoff. No CH12 press is required.

See [implementation and simulation limits](../evidence/balance-lower/continuous-return-v4/README.md).
This remains experimental: 329 paired model cases include four regressions and
stiff delayed impacts that still fail. It needs a restrained physical trial.

For the trial, use the installed identity in [current state](progress/CURRENT.md).
Start physically flat with the arms in their Forward position before arming,
as required by the existing arm-reference procedure. Stand up, center CH1/CH2,
wait for startup/arm return to settle, then pulse CH11 once. Expect the arms to
catch, reverse, and continue lowering to level with arms ahead of the body.
Do not add a CH12 command during the test. If it stops, keep it supported and
record the pose rather than repeating the maneuver. Disarm both groups and
retrieve the saved log before another attempt.

V4 uses feature bit 32768; the separate fast tip-up uses 16384. Combined captures
use flags 65535 with unchanged schema 4 / 240-byte samples. Old logs retain their
original metadata. See [OTA procedure](WIFI_OTA.md) for the frozen-package update.
