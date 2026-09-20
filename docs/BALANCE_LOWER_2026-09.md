# Experimental CH11 return to flat

**Integration update:** this feature is included in installed combined source
`43b1967`. See [current state](progress/CURRENT.md) and the [combined release
record](BALANCE_DRIVE_BRAKING_2026-09.md#installed-combined-release). The isolated
build/handoff details below are historical; physical evaluation remains pending.

## Operator behavior

After stand-up has completed its normal arm return and settled, one fresh CH11
pulse requests a supported return to all four wheels. CH7, drive arming and arm
arming must remain active. CH1/CH2 are treated as neutral for the maneuver so
the existing standing-drive controller first stops travel. Repeated CH11 pulses
do not restart a maneuver. Idle single/double-tap stand-up behavior is unchanged.

The stored arm **Forward** reference points up while the body is standing.
This maneuver reaches physically forward/down along the negative calibrated
center axis, then brings the arms back to their flat Forward reference as the
body lowers. It does not simply invoke the old ReturningArms routine, which
immediately stops wheel balancing and ends logging.

| Stage | Behavior and progression |
| --- | --- |
| LOWER_WAIT | Ordinary wheel balancing/braking continues. Require both wheels <=0.65 rad/s, tilt rate <=4 deg/s and balance error <=2 deg for 500 ms. |
| LOWER_REACH | Move each arm forward at 0.25 rad/s, limiting its target to 0.12 rad ahead of measured position. Both arms must show >=0.4 Nm load, >=0.06 rad resisted motion, <=0.12 rad/s speed and sufficient forward extension for 240 ms. |
| LOWER_LOAD | Retain wheel balancing. Remove contact preload, slowly request 3 degrees of forward lean, and ease the arms back only 0.035 rad. Require >=2 degrees of measured forward body motion, load on both arms, and quiet wheel/body motion before releasing wheel balance. |
| LOWERING | Rear wheels hold zero speed; front wheels retain their existing hold. Arms unwind at up to 0.16 rad/s, pausing above 12 deg/s forward body descent. Fresh sensors, motor feedback, arming, radio, rate, support-loss and progress limits remain required. |
| Ground hold/retract | Require measured tilt within +/-5 degrees, rate <=5 deg/s and quiet wheels for 600 ms. Retract arms to the Forward reference, verify their measured arrival, then restore CSP ground drive and finish the log as `lower_complete`. |

Missing contact, out-of-reach support or failed weight transfer causes slow arm
retraction while wheel balancing continues. A fault after weight transfer stops
the maneuver and holds the measured arm positions; it does not command a fast
return of the supporting arms. Disarm and CH7 cancellation retain priority.
Ground drive still requires neutral sticks after the balance handoff.

## Validation and limitations

The native checks execute the production maneuver helper: calibrated direction,
quiet entry, contact qualification, unsupported cancellation, target lead bounds,
weight-transfer requirement, rate pause, sensor/support faults, timeouts, clock
rollover and measured landing. All ten native executables, 27 Python tests,
syntax checks, whitespace checks and the full pinned ESP32 build passed.

The [contact-model evidence](../evidence/balance-lower/README.md) contains 81
scenarios using this same C++ helper. Of 72 geometry/compliance/servo/body-model
variants, 52 complete and 20 cancel because support is out of reach. The nominal
run takes 26.12 seconds and peaks at 18.09 deg/s; completed variants peak at
14.51–20.10 deg/s. Missing/one-arm contact, an unreachable floor, slow servos,
stale feedback and obstructed arms do not report a successful landing. Loss of
support during descent produces a fault, not a claim of a controlled landing.

**These are screening results, not physical validation.** Arm length, pivot
location, contact stiffness/damping and the wide-angle gravity model are
estimates. Preparation assumes the existing balance controller can follow the
arm equilibrium/lean request; the model does not execute the full firmware
scheduler or validate that coupled response. It omits sideways roll, uneven
contact, floor friction, backlash, structural flex and realistic impact forces.
The small-angle arm equilibrium fit is extrapolated while reaching.

The maximum forward extension is 1.4 times the calibrated center delta, capped
at 2.6 rad (about 142 degrees for the archived 1.77-rad calibration). This exceeds the historical
0.3-fraction forward recovery range and is **mechanically unverified**. Two-arm
torque/tracking plus measured weight transfer is an inference of support, not
a dedicated foot-contact sensor. An obstruction or incorrect calibration can
defeat those assumptions. Before an unsupported trial, verify clearance and
the reach direction with the body secured and unobstructed arm travel.

## First supervised trial after the combined release

1. Verify the combined installed image and retained calibration; use the usual
   fresh-radio/motor/IMU checks from [BALANCE_TESTING.md](BALANCE_TESTING.md).
2. On a level surface with a catch restraint that allows descent, perform normal
   stand-up. Leave CH1/CH2/CH4 centered and wait for stable balance. Keep clear
   of the arm sweep and have the existing disarm control available.
3. Pulse CH11 once, then release. Check that both arms reach toward the floor,
   that the body transfers weight forward, and that lowering remains slow.
   Support/disarm if direction, contact or body motion is wrong; do not force
   the arms through an obstruction or repeat a failed maneuver without its log.
4. After completion, center the sticks, then disarm both groups and keep power
   on for saving. Download the complete `.csv` and `.wire` using
   `scripts/robot_wifi.py log`. Record floor contact, any asymmetry, landing
   harshness, intervention and actual elapsed time beside the log.

## Integration record

Worktree `worktrees/balance-lower`, branch `codex/balance-lower`, based on the
standing-drive owner's committed `10766f7` integration checkpoint. Changes are a
new `balance_lower.h` policy, CH11 routing in `main.cpp`, focused integration in
`balance_controller.cpp/.h`, tests, simulation and these dedicated docs/evidence.
No shared progress files, radio files, normal control gains, motor setup or
other agents' checkouts were edited. No device command, flash or motion occurred.

Existing BalanceState IDs 0–3 remain unchanged; supported descent appends ID 4
(radio mode 2 / phase 6). Preparation remains Balancing (ID 2), with status
labels LOWER_WAIT/REACH/LOAD/CANCEL. Log feature bit 2048 identifies this version;
`pilot_flags` bit 64 marks active lowering and bits 8–11 contain the helper phase.
No binary sample layout change. The stored header feature distinguishes old
logs when exporting. Combine the feature with the braking owner's bit 1024.

The local build uses the public example Wi-Fi header and is **compile-only**.
The release owner must build the integrated source with the actual private
configuration, run the combined release checks and record the installed image.
This prototype must be physically evaluated before claiming graceful or
repeatable real-world lowering.
