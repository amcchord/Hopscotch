# September 14: stand-up reliability pass

> Historical record, archived 2026-09-21. Use [CURRENT](../../progress/CURRENT.md)
> and [the OTA guide](../../WIFI_OTA.md) for present identity, ownership and commands.

> **Historical record:** this document describes the dated release or trial below. For the installed firmware, see [current state](../../progress/CURRENT.md). Use [OTA updates and Wi-Fi telemetry](../../WIFI_OTA.md) and the [current test guide](../../BALANCE_TESTING.md) for new work. Older package paths, app0-only USB commands and “next” actions below are preserved as history; they are not instructions for updating the current robot.

This pass corrects motor feedback faults that occur as balance starts. It is a
test candidate for reducing initial roll-away; an unaided stand-up improvement
still needs a physical trial. The radio timing fix remains in place.

## Findings from the latest trial

The [28-second assisted trial](../../../telemetry_logs/bal_20260914_214925_crsf-timing-fix-assisted.csv)
had regular 20 ms samples, but accumulated 12,517 CAN receive misses. Wheel
feedback age reached 145 ms. The initial surge began during arm return, before
ramp completion, reaching a 5.17° target error at about two seconds after balance
engagement. Austin assisted by hand, then reported stable balance, a tap and
deliberate disarm. Contact and tap times were not marked.

The firmware generated approximately 1,450 CAN requests/second during balance:
400 rear-wheel speed writes, 800 redundant front-wheel hold writes, 200 arm
writes and 50 scan/read requests. Yet feedback processing could drain at most
16 messages × 50 Hz = 800 messages/second. The approximate 650/second deficit
agrees with the 12,517 receive misses over 19.265 seconds of recorded balance.
The 32-frame queue could not solve a sustained throughput deficit. Receive age
was stamped when dequeued, so it understated time spent waiting in the queue.
This is a demonstrated feedback problem; it does not prove it is the sole cause
of the initial surge.

The protocol parser also used RS00 scaling for every motor. Hopscotch has four
RS05 wheels and two RS00 arms. The manufacturer [RS05 manual, section 4.1.3](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS05/RS05User%20Manual260713.pdf)
specifies ±50 rad/s and ±5.5 Nm for wheel feedback, compared with the ±33 rad/s
and ±14 Nm arm-motor ranges used by the old decoder. Thus historical wheel
velocities were reported at 0.66 of their physical value. Old logs are preserved;
they are not silently rewritten into different units.

Generic acknowledgements and fault frames could also reuse position/velocity
fields from an earlier response, and then refresh the motor's motion timestamp.
Malformed/unknown frames prematurely stopped queue draining.

## Firmware changes

1. Drain feedback at 200 Hz, with a 64-frame cap and a 1.5 ms elapsed-time budget.
   Both limits apply; a continuously busy bus cannot monopolize the task. Increase
   the RX queue from 32 to 64 frames to absorb short bursts.
2. Refresh stationary front-wheel CSP holds at 50 Hz. Rear balancing-wheel speed
   commands remain at 200 Hz. This removes 600 redundant requests/second, reducing
   expected requests to approximately 850/second. Front hold transmission failures
   are included in telemetry diagnostics; failed speed-limit writes now propagate.
3. Decode RS05 wheel velocity/torque correctly, including remapped motor IDs.
   RS00 arms retain their scaling. MIT encoding uses the selected motor model too.
4. Only complete motion packets update motor kinematics and motion freshness.
   Fault packets set the fault without overwriting position; acknowledgements do
   not clear faults or refresh motion. Unknown and malformed frames are consumed
   without stopping the bounded drain.
5. Mark new logs with feature bits identifying corrected wheel units and the new
   feedback/front-hold cadence. Old logs remain readable with their original
   feature flags, configuration, rows and checksums.

Correcting speed units alone would otherwise increase the velocity feedback gain
by 50/33. This release converts the associated gains and thresholds to preserve
the existing balance-controller response: velocity thresholds/targets multiply by
50/33, while velocity-to-angle and velocity-to-arm gains divide by that factor.
Inner PD, wheel command limits, arm poses, arm rates and balance target schedule
remain unchanged. Native randomized checks verify equivalent outputs before and
after the unit conversion. Calibration and stored trim are preserved.

Representative numeric changes (a unit conversion, not an increase in gain):

| Parameter | Old reported-speed units | Correct physical units |
|---|---:|---:|
| Low velocity P | 0.700 | 0.462 |
| High velocity P | 2.200 | 1.452 |
| Velocity I | 0.350 | 0.231 |
| Velocity knee | 0.800 | 1.212 rad/s |
| Position-to-target-speed P | 0.050 | 0.07576 |
| Arm-assist velocity threshold | 1.400 | 2.121 rad/s |

## Rejected control changes

Screened smaller ramp offsets, removal of ramp damping, tighter target-error
gates, slower returns, target-following limits and equilibrium-tracking variants.
They introduced failures in the approximate model. An acceleration-limited arm
return initially looked plausible, but broader screening produced 26 early and
17 late failures versus 25 early and 10 late failures for the baseline across
108 stress cases. It was removed before release. This model omits contact,
hand assistance and CAN arbitration; those numbers are rejection evidence, not
predicted physical failure rates.

With that experiment removed and the unit conversion applied, all 108 modeled
cases match the baseline's outcome and drift metrics. This verifies behavioral
equivalence of the unit migration. It does not model or certify the benefit of
fixing actual receive losses. The final image retains the established arm motion
profile; early progress commentary about a smoother return described a rejected
experiment, not the released firmware.

## Verification and next trial

`scripts/check_balance_candidate.sh` passes four native suites, 16 Python tests,
syntax/whitespace checks and the ESP32-S3 build. Tests exercise the production
Robstride decoder and MotorManager: both motor types, remapped IDs, truncated
frames, acknowledgements, faults, motion freshness, a 100-frame backlog, count/time
budgets, clock rollover and transmission failures. Existing RS05 startup, USB
transfer, CRSF timing and inner control regressions also pass. Final build uses
1,135,885 flash bytes and 50,720 static RAM bytes.

Evidence is under `evidence/balance-standup-fix/`; the frozen application and
rollback are under `artifacts/balance-feedback-fix/`. Release identity and live
checks are recorded in [current state](../../progress/CURRENT.md).

The next physical trial should be one ordinary stand-up using the
[existing channel routine](../../BALANCE_TESTING.md), initially without a deliberate
tap. Record whether initial hand assistance is needed. After supporting the
robot and lowering both CH9/CH10, retrieve the log and compare initial travel,
receive misses, feedback freshness and recovery. Use CH12 to mark a later tap
in a separate trial. Reliable unaided stand-up is not yet established.

## First physical result

The correction was flashed and verified from source `cad335c`; both groups were
disarmed and all six motors online after programming. Austin's first trial still
ran away, then stabilized after hand intervention. All 1,428 samples were saved
and checksum-verified after a USB cable bump/reboot. Initial wheel displacement
was 18.667 rad versus 9.187 rad in the preceding assisted trial. No unaided
improvement is established. See the [follow-up analysis](BALANCE_HANDOFF_FOLLOWUP_2026-09.md)
for the comparison, calibration-bound bug and rejected further experiments.
