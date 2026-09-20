# Experimental CH11 forward fall and arm catch — v2 candidate

**Candidate only; not installed by this task.** Austin's first physical v1 trial
failed: the robot leaned backward, never qualified arm contact, canceled the
reach and tipped backward on its own. Ground and standing driving worked very
well. The [archived 2,371-sample run and diagnosis](../evidence/balance-lower/trial-20260920/README.md)
reproduce every recorded helper transition through the deployed v1 code.

Austin clarified the desired maneuver: **let the robot fall forward and catch
that fall with its arms**. V2 explicitly leaves upright balancing before
contact. It does not wait for an arm tracking stall while the wheel controller
counteracts the forward arm motion.

## Sequence

A fresh CH11 pulse after ordinary stand-up settles requests the maneuver.
CH7 and both arming switches must remain active, with fresh real RC, IMU and
six-motor feedback. CH1/CH2 are treated as neutral while lowering owns motion.
Existing idle single/double-tap stand-up behavior is unchanged.

| Phase | Action and progression |
| --- | --- |
| Stop | Ordinary standing-drive braking/balance continues. Require quiet wheels, body rate and balance error for 500 ms. |
| Prepare | Position arms 1.3 rad physically forward of their standing-up reference, at 0.25 rad/s. Advance only while balance remains calm; require measured arrival and 200 ms of quiet balance. The body may lean backward slightly here as the normal controller compensates the arm mass. Preparation failure ends the attempt while retaining the measured arm pose. |
| Forward initiation | Transfer wheel ownership out of upright PD/position PI. Slew rear speed backward at up to 4 rad/s², capped at 2 rad/s, to initiate forward body rotation. Pause further acceleration once forward rate reaches 6°/s. After measured forward rate reaches 1°/s, swing the arms farther forward at up to 1.5 rad/s, capped at 2.4 rad travel. |
| Catch | Once measured forward tilt drops 1°, forward rate exceeds 2°/s and both arms extend at least 1.8 rad, retain the small rear speed rather than abruptly braking the body upright. Hold each arm at its first qualifying load; do not continue pushing it into the floor. After both impacts, permit only 0.06 rad of yielding at 0.12 rad/s, pausing during rapid forward descent. |
| Supported descent | Require an actual forward fall, subsequent reduction in falling rate, both arms loaded and slow arm motion for 80 ms. An early catch can qualify after 0.5° of forward travel. Only then unwind the support arms toward flat at 0.16 rad/s, pausing above 12°/s forward descent. Slew rear speed to zero at 0.75 rad/s². |
| Flat hold/retract | Require tilt within ±5°, body rate within 5°/s and quiet wheels for 600 ms. Retract arms at 0.30 rad/s, verify measured arrival, then restore ground drive and end as `lower_complete`. Ground drive still requires neutral after handoff. |

Motor direction comes from the opposite-sign calibrated center axes. The
stored **Forward** reference is the flat-body reference; it points up when the
body stands. Negative center-axis travel reaches physically forward/down.

## Protection and limits

- Each target stays within 0.12 rad of measured arm position. Contact detection
  ignores the first 100 ms of the fast arm sweep and requires at least 1.5 rad
  of extension. First load is 0.4 Nm; sustained support requires 0.2 Nm on both
  arms plus measured body deceleration and slow arm motion. These are contact
  inferences, not dedicated foot sensors.
- Lowering uses the existing fast 6 ms gyro-rate filter and requires every
  motor's feedback to be at most 100 ms old. Ordinary driving retains its
  existing freshness rules. The 200 Hz wheel sender rejects stale lowering
  commands after 100 ms and requests outside ±2 rad/s.
- Wrong-way departure (>2° backward from the launch posture or >12°/s backward),
  missed catch, stale feedback, support loss, excessive motion and lack of
  progress produce faults. No-forward timeout is 1.8 s; catch timeout is 2.5 s.
  Preparation has a 16 s deadline; complete descent has 20 s after departure.
- On fault or CH7 cancellation after departure, stop the wheel command and
  retain measured arm positions through the existing hard-abort/override
  resynchronization. Do not command a rapid arm return during a fall. A fault
  is **not** evidence that an unsupported robot will land safely.
- Normal ground drive, standing-drive v5, balancing gains, startup, arming,
  radio, motor configuration and network implementation are unchanged. The
  controller changes are confined to CH11 lowering and its stored telemetry.

## Evidence and next physical check

The [v2 evidence](../evidence/balance-lower/forward-catch-v2/README.md) includes
229 model cases, native checks, historical replay and the full ESP32 build.
The nominal model reaches flat/retracted in 19.90 s and peaks at 19.84°/s.
Of 216 chosen geometry/plant/servo/inertia variants, 180 complete and 36 fault
while preparing, before intentional departure. All 11 injected fault scenarios
avoid reporting success. These counts are screening outcomes, not reliability.

The new model includes wheel/body/arm coupling and floor forces during every
phase, including preparation. Its balanced preparation is no longer assumed
successful. However, geometry, wide-angle COM, inertia, joint stiffness,
friction and contact forces are not calibrated to this robot. Preparation's
stationary balance cascade is approximated; the full scheduler is not emulated.
Sideways roll, uneven contact and realistic structural impact are absent.
Some completed variants catch at up to **55.15°/s**: software completion alone
does not demonstrate a graceful catch. Peak model torque is not a hardware
load rating. Arm clearance and actual catch severity require physical checks.

The release owner should integrate the candidate and preserve the successful
v5 driving release as rollback. Build with the existing private network
configuration, run combined checks and record an actual installed identity
before testing. This worktree's public-example credential build is compile-only.
No firmware was flashed by this task.

For the next **restrained, supervised** trial, verify the forward arm direction
and clearance, stand up normally with neutral sticks, then pulse CH11. Expect a
slow preparation followed by a deliberate forward departure, arm catch and
supported lowering. Keep the existing disarm control available. Record actual
arm/floor contact, any asymmetry, impact severity and intervention. After
supporting/disarming, allow the log to save and download both CSV and exact wire
before another attempt. The current installed v1 maneuver has already failed;
do not mistake it for this candidate.

## Integration and telemetry

Worktree `worktrees/balance-lower`, branch `codex/balance-lower`, now includes
the exact deployed `eb2bb23` combined release before the focused v2 changes.
Shared CURRENT/JOURNAL and other agents' worktrees are owned by their existing
coordinators and were not edited here.

Schema 4 and the 240-byte sample remain unchanged. Feature bit 4096 identifies
v2 (combined flags 8191). Phase bits 8–11 in `pilot_flags` retain the existing
encoding; phase 9 now means forward initiation and phase 10 means catching.
Stored v1 logs retain their original phase-9 Loading description when exported.
Balance state 4 now covers departure, catch and supported descent. Its wheel
commands are recorded in the existing command columns; its setpoint column is
zero because upright PD does not own that state. For new v2 captures, `roll_rate` records the
6 ms catch-rate filter while the lowering phase is nonzero; ordinary balance
retains its existing slower filter. The feature-tagged CSV metadata documents
this choice, and historical v1 exports retain their stored values.
