# Balance Mode Tuning History

## Current Architecture (v13: Control-Core Split + Sim-Tuned Standup, July 2026)

### Task layout (control-core / comms-core split)
- **Core 0 (comms)**: WiFi + lwIP (framework-pinned) + async_tcp (pinned by
  build flag, priority 3).
- **Core 1 (control)**: 200 Hz balance PD task (prio 18) > 50 Hz control
  task (prio 12: serial cmds, IMU, CRSF, CAN, arming, balance state
  machine, arm/drive) > loopTask (prio 1: display, WebSocket, debug burst).
- Stall forensics: control-tick gaps >100ms recorded (section attribution +
  priority-24 sentinel discriminator), dumped as `# stall_*` in `bal log`.

### Inner loop (200 Hz)
- Back wheels in **Robstride Speed mode** -- switched at the START of
  tip-up (static robot, verified writes cost nothing there; 250ms 0-speed
  keepalive feeds the motor CAN watchdog through the tip). PD (Kp=2.0,
  Kd=0.08 on complementary-filter tilt) commands wheel velocity directly.
- Yaw sync: differential correction holds the L/R wheel position
  difference at its engage value (+/-1.5 rad/s).
- Two-stage dead-man on the control-task heartbeat: >300ms stale ->
  authority clamped to 20 rad/s; >1.5s -> wheels stopped.

### Outer cascade (50 Hz)
- Position P (drift -> target velocity, origin = engage position) ->
  velocity PI (dual-slope: 0.7 below 0.8 rad/s, 2.2 above) -> tilt
  setpoint offset, rate-limited 12 deg/s, clamped +/-8 deg.
- **Position P runs through the standup ramp too** (gain 0.03, clamp 0.6
  rad/s) so standup drift is opposed as it develops; integrator stays
  ramp-gated.
- Standup: refit curve shape (84.0/83.05/82.1, ~1.9 deg tip->fwd rise),
  proportional arm return (both arms land together).
- High-velocity shed: braking-by-lean authority fades 8->14 rad/s
  (leaning back needs forward acceleration; near the speed ceiling that
  self-defeats).
- Single integrator = equilibrium learner: calm-gated, glide-boosted,
  seeded from persisted settings.balance_trim, saved back after >=8s runs.
- Offline: `scripts/balance_sim.py` (firmware-faithful sim + stall
  injection), `scripts/fit_balance_model.py --speed-only`.

### Setpoint schedule (self-calibrating)
- effective_sp = curve(tip_frac) + center_frac*(CENTER-FWD) + stored trim
  + run_curve_shift + capture_shift + sp_offset.
- Arm deltas decomposed onto calibrated tip + center axes.
- **Capture self-calibration**: every settled capture (at the tip stance)
  re-zeros the curve's absolute level for that run -- anchors provide
  shape only.

### Arm assist (the robot's second actuator)
- Neutral = forward pose (= top-dead-center when standing). Excursions
  +0.45 (arms back, brakes forward motion) / -0.30 (arms forward of
  vertical, brakes backward motion) of the calibrated center axis.
- **Engagement lifecycle**: READY -> ACTIVE -> (one recoil HANDOFF) ->
  COOLDOWN; re-arms only after 400ms of genuine calm. External bumps
  arrive out of calm; self-oscillation never re-establishes it -- the
  arms structurally cannot sustain a limit cycle.
- Fast attack (80ms tau, 12 rad/s motors, ~raw velocity input), 0.65s
  release; wheel velocity-P yields up to 60% while arms are deployed.
- **Emergency throw**: wheels railed + velocity error above threshold =
  roll-away; arms bypass the lifecycle to their full stop.

### Safety & infrastructure
- CAN bus-off auto-recovery; motor-side CAN watchdog (0x200C, ~1s);
  verified (read-back) writes for symmetry-critical motor params;
  stale-wheel-feedback abort (400ms); level-based drive-disarm stop
  enforcement; non-blocking USB CDC; loop profiler dumped with `bal log`.
- Telemetry: 50Hz PSRAM log with config header; scripts/
  analyze_balance_logs.py (--plot), scripts/fit_balance_model.py.

### Known open items
- Phase 15 fixes (standup + core split + pre-switched Speed mode) are
  sim-validated and built but NOT yet bench-validated -- run the Phase 15
  validation ladder next session.
- If stall forensics still shows events after the core split, the `sec:` /
  `sentinel_us:` fields in `# stall_*` name the remaining cause class
  (own blocking code vs preemption vs whole-core flash stall).

## Previous Architecture (v10: Position PI with Leaky Origin)

### Core 0 (200Hz) - Balance PD
- Complementary filter: alpha=0.996 (angle), gyro filter alpha=0.08 (~60ms time constant)
- IMU axes: gyro.x for roll rate, atan2(accel_y, accel_z) for accel angle
- PD: `motor_vel = Kp * (effective_sp - tilt) - Kd * gyro_rate`
- **Kp = 2.0, Kd = 0.08**
- Max drive speed: 25 rad/s
- Back wheels only (front wheels held at engage position)

### Core 1 (50Hz) - State Machine + Position PI with Leaky Origin
- **Arm-scheduled base setpoint**: linearly interpolates between ARMS_TIP (86.5) and ARMS_FWD (92.0) based on current arm position
- **Temporary capture shift**: at engage, starts from the measured roll so the handoff is smooth, then fades that offset out as the arms return
  - Capture shift clamped to +/-15 deg (covers all practical engage angles)
- **No command integrator trim** -- any form of trim creates positive feedback with position PI (v5/v6/v7 all proved this)
- **Position PI with leaky origin**:
  - Kp=0.40, Ki=0.15, Kd=0.05
  - Shift clamp +/-5 deg, rate limit 4 deg/s, deadband 0.15 rad, integral max 200
  - Angle-error gate: `pos_gate = 1 - |angle_err| / 8 deg`
  - **Leaky origin**: `_wheel_start_pos += 0.15 * (current_pos - _wheel_start_pos) * dt` (~7s time constant)
    - Origin slowly moves toward the robot's current position
    - The PI only corrects recent drift (bounded to 1-2 rad), preventing saturation
    - Old drift is gradually accepted as the new "home"
- **Effective setpoint = scheduled_base + capture_shift + position_shift**
- Safety: tilt range [30,150], sustained error >35 for 2s, rate >200 dps for 500ms, saturation >3s

### Arm Tip-Up
- Tip speed: 0.7 rad/s (distance-based deceleration per arm)
- Tip offsets: left=2.61, right=1.89 rad from forward ref
- Engage after: arms done AND |tip_expected - roll| < 15 AND |rate| < 50
- **Arm return: 10 rad/s** (fast snap-back, ~0.25s)
- Arms begin returning after balance has actually been captured:
  - |effective_sp - roll| <= 1 deg, |roll_rate| <= 4 dps, |motor_cmd| <= 1 rad/s for 400 ms
  - Failsafe return after 2.5s so the robot cannot stay stuck in the tip pose forever

### Controls
- Ch7 high + single Ch11: Normal tip-up sequence
- Ch7 high + double-tap Ch11 (within 500ms): Force-engage at current position
- `bal engage`: Force-engage via serial
- `bal kp/kd <val>`: Tune PD gains
- `bal vgain <val>`: Tune command integrator gain
- `bal pkp/pki/pkd <val>`: Tune delayed position-hold gains
- `d`: Toggle periodic debug output

---

## Key Findings (Chronological)

### Phase 1: Initial Implementation (50Hz, Cascaded PID)
- Cascaded PID (outer angle -> inner rate) collapsed to effective Kp=0.08, Kd=0.1
- LittleFS telemetry writes blocked control loop for 6+ seconds -> PSRAM buffer fix

### Phase 2: Single PD+I Controller (50Hz)
- Replaced cascaded PID with direct PD + adaptive setpoint
- Velocity bias position PID for drift correction
- **Best pre-200Hz run**: 21.3s, 0% saturation

### Phase 3: 200Hz Dual-Core Architecture
- FreeRTOS task on Core 0 for 200Hz PD loop
- Complementary filter replaces Madgwick (near-zero latency)
- **Gyro filter alpha=0.08 is non-negotiable.** Every lighter filter oscillated.

### Phase 4: Feedforward Compensation
- Proactive setpoint from arm position instead of reactive discovery
- ff_setpoint = BASE_DEG + FF_GAIN * arm_delta
- Led to discovery that balance point with arms at forward ref is ~87-88 degrees

### Phase 5: Balance Stability Hardening (Apr 9 evening)
- **Velocity bias removed from PD loop** -- was canceling tilt recovery (the root cause of all previous runaways)
- Position correction moved to bounded setpoint shift (max +/-5 deg), gated by tilt health
- Safety aborts added: tilt range, sustained error, extreme rate, motor saturation, link-loss
- Measured odometry for drift detection (replaced commanded-target-based drift)
- **Best run with old architecture: 30s, 98% under 3 deg error, 0% saturation** (telemetry: bal_20260409_220324.csv)
- But setpoint mismatch at arm return caused repeated failures

### Phase 6: Dynamic Balance Finding (Apr 9 night)
- Removed all hard-coded balance points
- Velocity-integrating setpoint: `setpoint += gain * filtered_wheel_vel * dt`
- Starts from current roll at engage, tracks dynamically
- Position shift PID removed entirely -- velocity integrator handles position return
- **Problem**: velocity integrator has no signal when stuck at wall (vel=0)

### Phase 7: Command Integrator
- Changed from integrating wheel velocity to integrating **PD motor command**
- Motor command has signal even when wheels are blocked (wall escape for free)
- Removed all stuck/wall detection -- command integrator handles it naturally
- **Problem**: integrator gain tuning -- too high causes positive feedback runaway, too low fails to track arm return

### Phase 8: Arm-Scheduled Setpoint + Command Integrator Trim
- **Breakthrough**: separate the known arm-position-to-balance-point mapping from the unknown trim
- Scheduled base: linear interpolation from 86.5 (arms at tip) to 92.0 (arms at forward ref)
- Command integrator is now just a trim (+/-5 deg) on top of the scheduled base
- Arms snap back at 10 rad/s, setpoint tracks in lockstep via scheduling
- **Best run: 48.9s balance, converged at 92.5 deg, 0.21 avg error** (telemetry: bal_20260409_231344.csv)
  - Kp=1.5, Kd=0.08 at the time
  - But needed hand-bouncing to find balance initially
- Kp increased to 2.0 for more aggressive correction
- Scheduled base refined to 92.0 based on telemetry (avg roll during stable = 91.99)

### Phase 9: April 10 Handoff + Roll-Away Debugging
- **Fixed premature arm return**: older code returned arms after a fixed 1s even if the robot was still catching itself.
  - New gate waits for actual capture before arm return.
  - `bal_20260410_200016.csv`: arm return waited 2.56s, balance duration 15.5s, confirmed stand-up reliability improved.
- **Roll-away remained**: with good capture, the robot still translated across the test bench.
  - `bal_20260410_200016.csv`: drift swung from -5.22 rad to +12.27 rad with motor commands saturating during wall impacts.
- **Attempted delayed position hold**: added measured-odometry position shift only after arms-forward/calm state, with a fresh wheel origin.
  - `bal_20260410_200622.csv`: balance capture was clean, but position correction was still too early/strong in the transient and the run lasted 9.4s.
  - `bal_20260410_201026.csv`: much better balance duration (22.9s) and position hold eventually locked, but the bot had already rolled away before the hold could help; drift reached about -15 rad.
- **Bad experiment: immediate wheel-velocity damping**: added a temporary setpoint shift proportional to wheel velocity.
  - `bal_20260410_201448.csv`: clearly worse. It destabilized the catch phase with |cmd| averaging 16-22 rad/s and roll swinging 43-114 deg.
  - Reverted. Do not reintroduce raw wheel-velocity damping without a much more careful design.
- **Current uploaded fix**: keep the low engage angle only as a temporary capture shift that fades out as the arms return.
  - Previous code incorrectly let the low tip-up capture offset behave like permanent trim; that caused the robot to chase a too-low setpoint and roll away.
  - Current code starts trim at 0, fades capture shift by arm fraction, then lets the command integrator learn only true load/terrain trim.
  - Delayed, gentle position hold remains as a secondary correction after the robot is calm.
  - Status: uploaded after `bal_20260410_201448.csv`; needs the next telemetry run for validation.

### Phase 10: Always-Active Outer Position PI Loop
- **Problem**: delayed position hold activated too late. `bal_20260410_201026.csv` showed clean balance (22.9s, 0.11 deg pre-return error) but -11.2 rad drift because position correction waited for arms-forward + calm + 500ms settle.
- **Solution**: replaced delayed position hold with an always-active outer PI loop that adjusts balance setpoint toward odometer zero from the moment balance engages.
  - Angle-error gate (`pos_gate = 1 - |angle_err| / 8 deg`) naturally suppresses position correction during the catch phase when angle error is large.
  - Engage position used as wheel origin immediately; no delayed origin lock.
  - Mostly PI design: Kp=0.40, Ki=0.03, Kd=0.02 (minimal D, just enough to damp overshoot).
  - More authority: max shift 4 deg (was 2), rate limit 2 deg/s (was 1), deadband 0.3 rad (was 1.0).
  - Integral max raised to 100 (was 50).
- **Removed**: `BALANCE_POS_HOLD_*` delayed-activation constants, `_position_hold_enabled`/`_position_hold_stable` members, settle timer logic.
- **Kept unchanged**: command integrator trim (still reduced to 20% when position correction active), arm-scheduled setpoint, capture shift, 200Hz inner PD, safety systems.
- **First run result** (`bal_20260410_204824.csv`, 23.7s): persistent ~2Hz oscillation, robot always rolling forward into hands.
  - Engage at 80.33, arms returned at 2.56s. Capture was ok but not clean (pre-return |rate|=16.5 dps, |cmd|=3.83).
  - Mid-run: roll oscillating 85-97 deg at 40-55 dps, drift swinging wildly.
  - Final window: settled into persistent oscillation around 91.5-95.6 deg at 20-30 dps. trim=+1.16, drift=-1.68 rad, pos_shift=+0.3.
  - **Root cause**: Kp=0.40 is too aggressive for the inner loop bandwidth. Each degree of position-driven setpoint shift causes 2 rad/s motor command (inner Kp=2.0), which accelerates the robot. The position PI overshoots, reverses, and creates a sustained pendulum-on-wheels oscillation. The command integrator trim climbed positive trying to fight the oscillation instead of finding true balance.
  - **Fix**: drastically reduce Kp to 0.10 (4x slower), raise Ki to 0.04 (let integral do the steady-state work), drop Kd to 0.01, widen deadband to 0.5 rad. The outer loop must be much slower than the inner balance loop to avoid exciting oscillation.
- **Second run result** (`bal_20260410_205347.csv`, 23.7s): clean capture (0.12 deg err, 1.01 dps rate), balanced briefly at 92.3 deg with drift only -1.85 rad. Then disturbance tipped robot backward, causing runaway.
  - **Root cause identified: positive feedback between command integrator trim and position PI.**
  - When robot drifts forward, position PI raises setpoint (correct). Higher setpoint creates sustained positive motor command. Command integrator sees sustained cmd and also raises trim. Both push setpoint higher together. Trim reached +3.7, pos_shift maxed at +4.0, setpoint at 99 deg (7 deg above true balance ~92). Motor stuck at +10 rad/s constantly.
  - Final state: robot at 93.9 deg with setpoint at 99, 5 deg gap maintained by constant +10 rad/s forward wheel drive balanced against gravity.
  - **This is the classic dual-integrator positive feedback problem.**
- **Fix: remove command integrator trim entirely.** This is how real inverted pendulums (Segway, nBot, etc.) work: inner PD on angle, outer PI on position, nothing else. The position PI's integral naturally handles balance-point offsets: if the scheduled setpoint is slightly wrong, the robot drifts, the integral accumulates, and the setpoint corrects. One integrator, no positive feedback.
  - Position PI gains: Kp=0.15, Ki=0.08, Kd=0.02, max shift 5 deg, integral max 200.
  - Effective setpoint = scheduled_base + capture_shift + pos_shift (no more trim term).
- **Third run result** (`bal_20260410_210127.csv`, 7.4s): violent oscillation from first tick, never achieved balance.
  - Engaged at 76.29 deg -- very low. Needed 10.2 deg capture shift but clamp was 8 deg, so setpoint was 2.3 deg above actual roll.
  - Without command integrator, nothing adapted the setpoint. PD oscillations grew from 15 dps to 220 dps. Robot fell backward to 103 deg.
  - **Root cause**: capture shift clamp too tight (8 deg) AND no fast setpoint adaptation during catch phase.
- **Fix: restore command integrator WITH trim decay to prevent positive feedback.**
  - Trim decay: when position PI shift > 0.3 deg, trim decays toward 0 at 0.99/tick (~1.5s half-life). Trim adapts freely during catch (pos_shift near 0), then fades out once position PI takes over.
  - Capture shift clamp raised to 15 deg (covers all practical engage angles).
  - Trim clamp reduced to 3 deg (was 5).
  - Position PI unchanged (Kp=0.15, Ki=0.08, Kd=0.02).
- **Fourth run result** (`bal_20260410_210747.csv`, 20s): clean capture (0.21 deg err), started stable, then same drift/oscillation pattern.
  - Trim decay of 0.99/tick too weak -- trim growth rate (driven by sustained motor commands from the feedback loop) exceeds decay rate. Trim still climbed to +2.03, pos_shift maxed at +4.12, setpoint=97.9 vs true balance ~92.
  - **Conclusion: any command integrator, even with decay, creates positive feedback with the position PI.** The trim adds to the setpoint, the PD reacts, the position PI also reacts, both corrections compound.
- **Fix: remove command integrator for real this time.** The v6 failure (bal_20260410_210127.csv) was caused by the tight capture shift clamp (8 deg), not by the absence of trim. With the 15 deg clamp now in place, the initial setpoint matches the roll perfectly and no fast adaptation is needed. The position PI alone handles the steady-state balance offset through its integral term.
- **Fifth run result** (`bal_20260410_211316.csv`, 33.2s): **best architecture yet.** No trim runaway. Clean capture, beautiful balance (0.1 deg error at t=30s). But position PI too sluggish -- robot drifted 2+ rad before correction built up, hit bench edge at t=41s.
  - The architecture is correct (no positive feedback). The gains are just too conservative.
  - Without the trim amplifying things, we can safely increase position PI gains.
  - **Fix**: Kp 0.15->0.30, Ki 0.08->0.15, Kd 0.02->0.03, deadband 0.5->0.2 rad, rate limit 2->3 deg/s.
- **Sixth run result** (`bal_20260410_222145.csv`, 38.9s): clean capture, excellent balance. But robot drifted to wall and STAYED there. pos_shift climbed to 4.7 (near max 5) but wheels stopped and drift froze at -4.1 rad.
  - **Root cause: position-controlled motors break tilt-to-translation coupling.** The position PI shifts the setpoint (robot leans), but the Robstride motors in position mode hold the lean angle with internal torque at a FIXED wheel position. The wheels don't actually move. In a torque/velocity-controlled Segway, holding a lean requires sustained wheel acceleration -> translation. In position mode, the motor just applies holding torque -> no translation.
  - **Fix: add direct wheel velocity offset** driven by drift error. The position PI manages tilt (for disturbance rejection), but a new velocity term (`_wheel_vel_offset = -0.3 * drift_err * pos_gate`) directly pushes the wheels toward origin in the 200Hz loop. This provides the missing translational coupling.
- Wheel velocity offset tried (`bal_20260410_223145.csv`): no meaningful improvement -- position PI gate bug discovered.
  - **pos_gate was using `base_effective_setpoint` (without pos_shift) instead of `_effective_setpoint`.** The gate saw the gap between the base sp (92) and the tilted robot (97) as "angle error" and throttled the PI to 37.5%. At max shift, gate would be 0%. **Fixed: gate now uses effective setpoint.**
- Sign flip: PI was pushing the robot INTO the wall. raw_shift negation was wrong. **Fixed: removed the leading negation** so negative drift → negative pos_shift → lower setpoint → lean forward → return.
- **Seventh run** (`bal_20260410_223740.csv`, 41.5s): **first return-to-origin!** Robot drifted to -2.5 rad, PI lowered setpoint, robot reversed and swung back to +7 rad (massive overshoot). Ki=0.20 too high.
  - Gains retuned: Kp 0.05→0.15, Ki 0.20→0.12, Kd 0.01→0.05 for less overshoot.
- **Eighth run** (`bal_20260410_224228.csv`, **50.4s NEW RECORD**): clean capture, eventually found stable balance at sp=84, drift=-3.84. But took 15+ seconds of ramping the setpoint down from 91.5 to 84.0. Robot barely translated during the ramp -- drift went from -3.45 to -3.38 over 14 seconds despite 5.3 deg of setpoint change.
  - **Confirmed: angle offset alone barely translates the robot.** Position-controlled motors can hold any lean angle with internal torque at fixed wheel position. A 5 deg setpoint shift produced only 0.07 rad of actual wheel movement in 14 seconds.
  - pos_shift maxed at -8.0. Robot found static balance at sp=84 with drift frozen at -3.84.
  - Need a mechanism that directly drives wheel movement, not just lean angle.
- **Ninth run** (`bal_20260411_091031.csv`, 7.7s): with origin-reset-on-arm-return + PI gains Kp=0.40/Ki=0.15/Kd=0.12. Capture shift faded in 0.25s (no rate limit yet), robot overshot to 98 deg, PD slammed wheels creating -6.35 rad drift in 1 second. PI overshot recovering. Toppled.
  - **Root cause: base setpoint transition too fast** (13 deg in 0.25s = 52 deg/s). PD overshoots, creates massive initial drift.
  - Added `BALANCE_BASE_SP_RATE_MAX = 5.0 deg/s` to rate-limit the base setpoint transition.
- **Tenth run** (`bal_20260411_092019.csv`, 11.5s): rate limit working (ramp took ~1.7s), but drift accumulated to +3.84 rad during the ramp. PI saw positive drift during the ramp and ADDED to the setpoint (compounding), causing overshoot to 100 deg then violent oscillation.
  - **Problem: PI runs during the base SP ramp and interferes.** The ramp intentionally changes the setpoint from ~84 to 92. Drift accumulates during this ramp. The PI sees the drift and adjusts pos_shift, but this fights/amplifies the ramp depending on sign.
  - **Fix needed: delay PI activation until the base setpoint ramp finishes.** Reset wheel origin only after `_smoothed_base_sp` reaches its target. This way the ramp completes undisturbed, then the PI starts fresh from drift=0.
- **Eleventh run** (`bal_20260411_092847.csv`, 20.6s): PI gated until ramp complete -- correct behavior. But 5 deg/s ramp still too fast. During the 2.8s ramp from 78→92, the PD saturated at 25 rad/s for 1+ seconds (robot falling forward, gravity winning). Drift reached +10 rad during ramp, then violent overshoot to 115 deg, then oscillation, drift to -12 rad. PI maxed pos_shift at -6.0, robot settled at sp=86 with drift=-11.5.
  - **Root cause: the ramp speed is not the issue -- it's the distance.** Tilting from 78 to 92 (14 deg) requires massive wheel movement regardless of speed. At 5 deg/s, the robot spends 1.5s in the "far from balance" zone (78-84 deg) where gravity pulls hard forward and the PD saturates. At 1 deg/s, the robot passes through this zone over 6 seconds with gentle motor commands that don't create massive drift.

### Phase 11: Full-State Feedback (Wheel Velocity Offset)
- **Problem identified**: the position PI adjusts the tilt setpoint, but in CSP position mode, shifting the balance angle barely translates the robot. The PD tracks instantly, so angle_err stays near 0, motor_vel stays near 0, wheels barely move. The 50.4s record run (`bal_20260410_224228.csv`) showed a 6 deg setpoint shift producing only 0.07 rad of wheel movement over 14 seconds. The 094120 run showed pos_shift saturated at -6.0 but drift frozen at -9.7 rad.
- **Root cause**: CSP motors hold lean with internal servo torque at a fixed wheel position. In a torque-controlled Segway, maintaining a lean requires sustained wheel acceleration -> translation. In CSP mode, this coupling is broken.
- **Fix**: add a direct wheel velocity offset to the motor command, computed from drift and wheel velocity. This is the missing K3/K4 of the standard full-state-feedback controller: `motor_vel = Kp*angle_err - Kd*gyro_rate + wheel_vel_offset`. The velocity offset is independent of tilt angle -- it directly drives the wheels toward origin regardless of what angle the robot balances at.
- **Architecture**: two outer-loop outputs with distinct roles:
  - Position PI (tilt trim): handles CG offset, slope, balance-point error. +/-3 deg authority.
  - Wheel velocity offset (translation): drives wheels toward origin. +/-5 rad/s authority.
- **Reduced PI authority**: Kp 0.40->0.25, Ki 0.15->0.10, shift max 6->3 deg. PI is now trim only.
- **Velocity offset gains**: vel_kp=0.80, vel_kd=0.15, max=5.0, rate=3.0. Same gating (pos_gate, ramp_complete) as PI.
- **Telemetry improvements**: constants block (# KEY=VALUE) emitted before CSV header in `bal log`, new `vel_offset` and `pos_gate` columns, `_ramp_complete` flag bit.

### Phase 12: Velocity Offset Failed -- Leaky Origin Fix
- **Result of vel_offset** (`bal_20260411_221653.csv`, 46.9s): the velocity offset did not produce sustained wheel velocity. At steady state the PD cancels the vel_offset by shifting tilt equilibrium: `tilt = setpoint + vel_offset / Kp`. With vel_offset=3.0 and Kp=2.0, the robot sits 1.5 deg above setpoint -- mathematically equivalent to a setpoint shift of vel_offset/Kp degrees. No translation occurs.
  - Worse: the tilt offset created positive feedback with the PI (same dual-integrator problem as Phase 10). vel_offset pushed roll above setpoint, PI lowered setpoint, drift grew, vel_offset grew further. By t=38s, oscillation erupted at +/-33 dps with motor commands at +/-6 rad/s.
- **Root cause is fundamental**: in CSP position mode, any additive term in the motor command (whether setpoint shift or velocity offset) reaches equilibrium where motor_vel=0 and wheels stop. The PD is too fast -- it absorbs the offset into tilt angle within a few hundred ms. The missing translation coupling cannot be restored without changing motor control mode.
- **Fix: remove velocity offset, add leaky origin.** Since CSP mode fundamentally prevents return-to-origin, the correct strategy is to stop fighting accumulated drift and accept the current location.
  - `_wheel_start_pos += ORIGIN_LEAK * (current_pos - _wheel_start_pos) * dt` with leak=0.15 (1/s), ~7s time constant.
  - The PI only sees recent drift (bounded to ~1-2 rad). It never saturates on large accumulated drift.
  - Old drift is gradually accepted as the new "home."
- **Restored PI gains**: Kp=0.40, Ki=0.15, shift max=5 deg (moderate authority for recent-drift correction).
- **Removed**: all velocity offset code, config, serial commands, gain members.

### Phase 13: April 12 Session -- Arm Balance Assist + Velocity Trim + Odometry PID (CSP-Era Closing Baseline)

38 runs (`bal_20260412_*.csv`), the largest single-day batch. Three architecture
iterations within the day, all still on CSP position-mode wheels:

1. **Arm balance assist** (morning, commit `21f644d`): after the base setpoint
   ramp completes, the arms actively move toward center proportional to filtered
   wheel velocity (reflex) and accumulated drift (correction). Physically shifts
   the CG to fight drift -- the one actuator CSP mode cannot cancel.
2. **Unified vel_trim + arm balance + setpoint dither** (afternoon, `3c4b1bb`):
   added a velocity-trim integrator (find the setpoint that produces a target
   wheel velocity) plus a dither probe of the CSP dead zone.
3. **Odometry PID + vel_trim, dither removed** (evening, `23f68ef`): odometry
   PID computes a target return velocity; vel_trim integrates toward it.

**Aggregate results across all 38 runs:**
- Median balance duration 26.9 s; 9 runs > 45 s; 3 runs < 10 s.
- **The 51.3 s "record" runs are log-censored, not falls**: 3000 samples at 50 Hz
  = 60 s total minus ~8.7 s tip-up leaves ~51.3 s of logged balance. Five runs
  end at exactly this boundary (`174731`, `175422`, `180748`, `191828`, plus
  `164912` at 52.6 s from a shorter tip). True balance duration is unknown but
  >= logged. Stability within those runs was 87-94% (fraction of samples with
  |angle_err| < 2 deg).
- **Drift was never controlled**: median max |drift| across runs was 11 rad
  (~a meter of travel), worst 60.4 rad (`194229`). Even the best runs wandered
  5-13 rad. The arm assist frequently sat at its 0.25 clamp.
- **vel_trim gain sweep (0.1 -> 1.0)**: gains >= 0.4 pushed trim to its 4-deg
  clamp and cut stability to 36-65% (`224949`, `230134`, `225745`). Gain 0.1-0.25
  was benign but produced no measurable return-to-origin. Same structural wall
  as Phases 10-12: any additive setpoint term reaches motor_vel=0 equilibrium.
- **Odometry PID (odo_kp=0.3, late evening)**: clearly degraded stability
  (36-65% stable vs 87-99% earlier in the day) without fixing drift.

**Conclusion**: the arm assist + vel_trim architecture is the ceiling of what
CSP position mode allows. Balance hold is excellent (the 200 Hz inner PD has
never been the problem); translation control is structurally impossible because
the position servo absorbs every corrective term. This closes the CSP era.
Next architecture: **Speed mode** on the back wheels (lesson #23), where the PD
output directly commands wheel velocity and cannot be absorbed into a static
holding torque.

### Phase 14: Speed-Mode Rebuild (design record, awaiting first test session)

Full architecture change, implemented after data-mining all 80 CSP-era logs
with `scripts/fit_balance_model.py`:

**Fitted plant model** (`telemetry_logs/model_fit.json`):
- `roll_accel = A*(roll - eq) + B*wheel_accel` with A=1.22 1/s^2, B=5.88
  deg per rad/s^2. Unstable pole ~1.1 rad/s (time-to-double ~630 ms).
- Closed-loop identification biases A low; treat as order-of-magnitude.
- Arm curve: balance point 91.5 deg at arms-forward, drops to ~86.9 by 15%
  of arm travel, then flat ~86.4-87.5 out to the tip pose. **The old two-point
  linear interpolation was wrong through most of the range.**

**Architecture**:
- On engage, back wheels switch CSP -> Speed mode (acc limit 100 rad/s^2,
  current limit 10 A). Front wheels stay in CSP holding engage position.
- Inner 200 Hz PD unchanged (Kp=2.0, Kd=0.08, gyro filter alpha=0.08) but
  `motor_vel` is now sent directly as the wheel speed command.
- Outer 50 Hz cascade, active after the base setpoint ramp completes, gated
  by angle error, with the wheel origin reset at ramp completion:
  - Position P: `target_vel = clamp(-0.05 * drift, +/-1.0)`
  - Velocity PI: `sp_offset = 0.8*vel_err + integral(0.05*vel_err)`,
    clamped +/-4 deg. Single integrator (lesson 18).
- Arm-scheduled setpoint upgraded to the fitted piecewise-linear curve
  (`BALANCE_SP_CURVE` in config.h), still driven by measured arm position so
  the setpoint tracks the arms through the whole return ramp.
- Removed: vel_trim, odometry PID, arm balance assist, dead position-PI
  code, stuck/wall detection (Speed mode + current limit handles walls).
- Every exit path (disengage, hard abort, RC/web disarm, link loss) zeroes
  wheel speed FIRST, then restores CSP. A Speed-mode motor keeps spinning at
  its last command, so ordering matters.
- New telemetry columns: `sp_offset`, `target_vel` (replacing
  `arm_bal_frac`, `vel_trim`); `# motor_mode=speed` in the config header.
- Serial tuning: `bal dkp` (drift->velocity), `bal vkp`/`bal vki`
  (velocity PI). `bal vgain`/`pkp`/`pki`/`pkd` removed.

**First Speed-mode run** (`bal_20260702_151416.csv`, 46.8s, tethered by USB):
- Capture parity CONFIRMED: 0.16 deg pre-return error, 0.13 dps, clean
  capture-gated arm return at 2.9s. Zero oscillation anywhere in the run --
  the Speed-mode inner loop is as solid as CSP, with far smaller commands.
- Outer cascade worked as designed (target_vel correctly +0.83 rad/s toward
  origin the whole run, sp_offset walking the right direction), but the robot
  rolled -17 rad backward and settled leaning on the USB tether.
- **Root cause: the arms-forward anchor (91.5) was ~1.5-2 deg too high.**
  The CSP-era curve fit is biased -- CSP "stable windows" include held
  off-equilibrium leans. In Speed mode the true equilibrium revealed itself:
  the integrator converged toward ~89.6-90.4. Holding 1.5 deg backward of
  equilibrium = continuous backward acceleration = the roll-away.
- Ki=0.05 needed 60+s to trim that error; drift outran it in 12s.
- **Fixes applied**: ARMS_FWD anchor 91.5 -> 90.2, VEL_SP_KI 0.05 -> 0.12.
- **Lesson 24: CSP-era stable-window fits overestimate the balance point.**
  Only Speed-mode data (where holding a lean requires wheel acceleration)
  reveals the true equilibrium. Re-fit all curve anchors from Speed-mode runs.
- **Lesson 25: test untethered.** The USB cable is a leash; the robot will
  find equilibrium against it and contaminate the log.

**Second Speed-mode run** (`bal_20260702_152107.csv`, 12.8s, untethered):
- Anchor 90.2 still ~1 deg high: integrator again settled at roll ~89.3
  (base 90.2 + sp_offset ~-1.0). Anchor corrected to **89.3**.
- Ramp-end overshoot to 96.3 deg (arm return began at 1.9s with capture less
  settled, "high cmd at return") -- recovered, then balanced cleanly.
- **At t=12.0 wheel feedback froze: CAN bus failure (TWAI bus-off).** The
  loop balanced blind for 0.8s, robot fell forward, PD saturated at +25 rad/s
  into a dead bus. Speed-mode motors kept executing their last spd_ref --
  wheels kept spinning through the fall AND through the RC safety switch
  (stop frames also lost to the dead controller). Only a motor power cycle
  stopped them. Post-mortem status: state=BUS_OFF, tx_err=128, arb_lost=12821,
  22k+ failed transmissions, no recovery logic -> bus dead until reboot.
- **Lesson 26: Speed mode must assume the bus can die.** Four defenses added:
  1. TWAI bus-off detection + automatic recovery (`Robstride::maintainBus()`,
     called every 50Hz tick).
  2. Balance aborts if wheel feedback is >400ms stale (was balancing blind).
  3. Drive safety switch is now LEVEL-based: while low, any wheel still
     reporting motion gets stop commands re-sent every 250ms.
  4. Motor-side CAN watchdog enabled at arming (param 0x200C = 20000 = 1s):
     the motor stops itself if no CAN command arrives, even with a dead host.
     NOTE: verify on bench -- exact units unconfirmed in the manual.
  Also: CAN TX timeout reduced 10ms -> 2ms so a degrading bus cannot stall
  the 200Hz balance task.

**Dynamic equilibrium learning** (added after run 2): the balance point is
not a constant -- it moves with battery seating, payload, surface, and IMU
mounting bias (anchor history: 91.5 -> 90.2 -> 89.3 in three runs). The
velocity-PI integrator is the equilibrium estimator; it is now:
  1. **Seeded** from `settings.balance_trim` (persisted on LittleFS) when the
     outer cascade activates, so each run starts where the last converged.
  2. **Gain-scheduled**: Ki x4 while "gliding" (|filtered vel error| > 0.4
     rad/s despite good angle tracking = equilibrium estimate wrong). The
     100ms velocity filter averages oscillation to ~0 so the boost only
     fires on genuine one-way glide.
  3. **Persisted**: after >=8s of post-ramp balancing (even if the run ends
     in a fall), the converged integral is blended 50/50 into the stored
     trim and saved.
  Serial: `bal trim` shows the stored value, `bal trim <val>` overrides,
  `bal trim 0` resets after mechanical changes.
- **Lesson 27: don't chase the balance point with constants.** The arm curve
  provides the *shape* (relative setpoint vs arm position); the persisted,
  self-learned trim provides the *absolute* equilibrium. One integrator,
  seeded and persisted, absorbs what three manual recalibrations could not.

**Third Speed-mode run** (untethered, log overwritten by a later tip-up):
- Reached a stable balance position -- trim learning confirmed working
  (stored trim converged to -1.10 deg after the run).
- Two new issues, both addressed:
  1. **Heading drift ("turned a bit")**: in Speed mode the two wheel
     velocity loops are independent; tracking differences integrate into
     yaw (CSP kept the wheels position-locked). Fix: yaw sync -- lock the
     L/R wheel position difference at its engage value with a differential
     speed correction (Kp=2.0 on the half-difference, clamp +/-1.5 rad/s).
  2. **Struggled after a tap**: the glide Ki-boost fired during the violent
     recovery (a tap also produces large velocity error), corrupting the
     equilibrium estimate mid-recovery. Fix: boost only when CALM
     (|rate| < 10 dps and |cmd| < 3 rad/s). Lesson 28: **equilibrium
     learning must freeze during disturbances** -- only learn when the
     motion is slow and steady, otherwise transients poison the estimate.
- NOTE: the on-device log is overwritten at each engage; pull telemetry
  after the interesting run, before re-engaging.

**Fourth Speed-mode run** (`bal_20260702_154823.csv`, 4.5s -- crash caused by
the trim-seeding implementation):
- Clean capture, good ramp... then at cascade activation (t=3.26) the
  setpoint STEPPED down 2.3 deg in one tick: seeded integral (-1.10) applied
  instantly + velocity-P term stepping as the velocity filter snapped from
  its zero reset to the actual -4 rad/s. PD yanked the wheels, robot pitched
  forward to 78 deg, recovery saturated (right wheel +15 rad/s while left
  stalled against the 10A current limit), overcorrected, fell backward.
- **Lesson 29: the effective setpoint must NEVER step.** Every term that
  feeds it needs either a rate limiter or continuous initialization:
  1. Stored trim now folded into the BASE setpoint at engage (delivered
     smoothly by the existing 3 deg/s base ramp); integrator learns only
     the residual, starting from zero.
  2. Velocity filter initialized to measured velocity at activation, not 0.
  3. sp_offset rate-limited at 5 deg/s as a hard guarantee.
  4. Speed-mode current limit raised 10 -> 14A (recovery torque-starved).

**Fifth Speed-mode run** (`bal_20260702_155640.csv`, 19.7s):
- **11+ seconds of the best balance ever recorded**: roll pinned at 87.4,
  sp_offset ~-0.1, drift ~-1.0, wheels essentially still. The learned-trim
  architecture works.
- **Tap at t=16 exposed the Speed-mode velocity-shedding problem**: the
  outer loop raised the setpoint (correct sign) but in Speed mode a STATIC
  angle error commands constant wheel VELOCITY, not acceleration -- so tilt
  never changed and the robot glided away at 3-4 rad/s for 11 rad before
  the correction built enough lean; then overshot backward and fell.
  - **Lesson 30: in Speed mode, shedding momentum needs a fast, large
    setpoint response.** Velocity error must convert to lean quickly
    (kv up 0.8 -> 1.6, offset clamp 4 -> 6 deg, offset rate limit 5 -> 12
    deg/s, Ki 0.12 -> 0.25). Sim on the fitted model: 2.3 rad/s tap now
    absorbed in ~6s with ~3.5 deg peak lean, vs runaway before.
- **Yaw root cause found (not the sync loop)**: during recovery transients
  the wheels were massively asymmetric (br -14.9 vs bl -2.2 rad/s). Param
  readback showed back-left CURRENT_LIMIT stuck at 10A while back-right had
  14A -- **the engage-time param write was lost on the CAN bus**. Asymmetric
  torque under saturation = pirouette. Lesson 31: **safety/symmetry-critical
  motor params must be written with read-back verification and retry**
  (now enforced; engage aborts if the limit cannot be confirmed).

**Sixth Speed-mode run** (`bal_20260702_161437.csv`, 52.1s -- BEST YET):
- Survived an escalating series of ~30 disturbance events over 40 seconds,
  including a hard shove at t=42 that peaked at 17 rad/s wheel velocity and
  104 deg roll -- recovered from all of them. Never fell; run ended at the
  60s log limit.
- Verified fixes working: wheel asymmetry gone (max 1.1 rad/s difference
  even at full saturation, vs 12+ before -- the verified current-limit
  write closed the yaw hole), taps now convert velocity into lean.
- "Not aggressive enough" is visible in the data: sp_offset sat at its 6.0
  clamp for 6.6s cumulative (the binding constraint), the wheel command
  railed at 25 rad/s during the big shove, and recoveries took 2-4s each
  with drift excursions up to 18 rad.
- **Tightening applied** (validated in sim: ~30% faster momentum shedding,
  return-to-rest ~3.1s vs 4.2s, position recovery 6.7s vs 8.9s on a 4 rad/s
  shove): vel_kp 1.6 -> 2.2, Ki 0.25 -> 0.35, offset clamp 6 -> 8 deg,
  offset rate 12 -> 16 deg/s, velocity filter 100 -> 55ms, max wheel
  command 25 -> 30 rad/s (RS05 limit 33).

**Seventh Speed-mode run** (`bal_20260702_164034.csv`, 22.0s):
- Stood up great, strong recoveries -- but constant low-frequency "wiggle":
  broadband sway at 0.1-0.5 Hz, roll std 0.76 deg, setpoint std 0.87 deg.
  Diagnostic signature: **sp_offset std EXCEEDED roll std** and sp_offset
  amplitude at each frequency exceeded the roll amplitude it was supposedly
  correcting -- the outer loop was driving the sway, not damping it.
  The tightened vel_kp=2.2 converts every bit of idle velocity ripple
  (+/-1 rad/s at standstill) into +/-2 deg of setpoint chatter, which the
  faster 55ms filter passed straight through. Classic single-gain
  compromise: what taps need, idle cannot tolerate.
- **Fix: dual-slope velocity response** (soft knee at 0.8 rad/s):
  - below knee: 0.7 deg per rad/s (calm station-keeping)
  - above knee: full 2.2 deg per rad/s (tap authority), continuous at the
    knee so there is no discontinuity to excite.
  - Glide-boost threshold raised 0.4 -> 0.8 rad/s to match (idle wobble was
    also pumping the equilibrium integral via the boost).
  Sim: idle roll std 0.05 deg with process noise (15x quieter), tap
  response unchanged above the knee.
- Lesson 32: **separate the station-keeping and disturbance-rejection
  regimes explicitly.** One linear gain cannot serve both; a soft knee on
  velocity error is the cleanest split (continuous, no mode switching).

**Eighth Speed-mode run** (`bal_20260702_164837.csv`, 40.9s):
- Dual-slope verified: quiet standing at roll 88.0 +/- 0.05 deg, commands
  under 0.1 rad/s -- best station-keeping ever logged. Small-tap recoveries
  clean.
- **Standup surge** (t=2-4): during the base ramp the outer loop is off, so
  the PD chasing the rising setpoint ran the robot to +10 rad drift; the
  operator had to tap it to a stop. Fix: the velocity damper (dual-slope P
  term, target_vel=0) now runs from ENGAGE, through the ramp. Learning and
  position return still wait for ramp completion.
- **Big-long-tap fall** (t=37-41): the backward push was braked perfectly
  (sp_off -7, velocity zeroed). The RETURN whipsawed: sp_off swung -7 -> +3.4
  in 1s, robot accelerated forward chasing a backward lean it could never
  build (leaning back needs MORE forward acceleration), hit the 30 rad/s
  ceiling at +17 rad/s, saturated, faceplanted, bounced, fell backward. The
  impact ejected the battery (source of the CAN errors seen after).
- Lesson 33: **wheel-based velocity braking self-defeats near the speed
  ceiling.** Two fixes:
  1. High-velocity shed gate: velocity-P authority fades 8 -> 14 rad/s;
     beyond that the robot accepts displacement instead of pumping itself
     into saturation (position loop brings it home afterward).
  2. **Arm assist v2** (operator's suggestion): above the velocity knee,
     arms swing toward the tip pose (up to 0.30 frac, 1.5 frac/s). The
     balance point drops ~3 deg instantly via BALANCE_SP_CURVE tracking of
     measured arm position -- an equilibrium shift requiring NO wheel
     acceleration, plus reaction torque in the braking direction. Forward-
     only (the arm axis can only lower the balance point); arms spring back
     to forward when calm. Unlike the CSP-era arm assist, this rides the
     existing setpoint schedule instead of fighting it.
- Also fixed: arm-assist excursions no longer re-apply a fraction of the
  engage capture shift (capture weight forced to 0 once arms have returned).

**Ninth Speed-mode run** (`bal_20260702_170714.csv`, 24.8s):
- Balance itself solid. Two arm-assist v2 bugs found:
  1. **Arms scissored instead of rising together.** The assist scaled the
     TIP deltas (2.71 / 1.96 -- both positive in motor space), but the
     physically-symmetric "arms up" motion is the CENTER axis, whose motor
     deltas are MIRRORED (+1.768 / -1.767 per the robot's calibration
     table). Same-sign motor commands = opposite physical arm directions =
     canceling CG effects. Assist now drives the center axis.
  2. **Target chatter**: engaging at the 0.8 rad/s knee made arm targets
     buzz +/-0.1 rad on idle velocity ripple. Threshold raised to 1.5 rad/s
     with asymmetric slew (fast in at 1.5 frac/s, slow release at 0.4).
- Setpoint math upgraded to match: measured arm deltas are decomposed onto
  the two calibrated axes (tip + center); the tip fraction drives the fitted
  curve, the center fraction adds a linear term toward the CSP-era center
  balance point (83 deg). Tip-up scheduling and arm-assist scheduling no
  longer alias into each other.
- Lesson 34: **arm motors are mirrored; "symmetric" motion means
  opposite-sign motor deltas.** Any arm motion axis must come from the
  calibration table, not from scaling another pose's deltas.

**Tenth Speed-mode run** (`bal_20260702_171838.csv`, 52.5s, log-limit end):
- **Arm assist works.** 8 clean deployments (up to the 0.53 rad clamp),
  forward taps at 5-6 rad/s absorbed with drift excursions of only 1-3 rad
  (vs 11-18 rad wheel-only in run 161437). Operator: "arms help a LOT."
- Confirmed one-directional: backward events (down to -5.9 rad/s) got no
  arm help and took bigger drift excursions (-10 rad at t=26.5). The rest
  stance (arms at forward) is the end-stop of the center axis, so the
  assist could only brake forward motion.
- **Fix: biased neutral stance.** Arms now hold center-frac 0.12 while
  balancing (costs ~0.8 deg of balance point, absorbed by the learned
  trim). Raise from neutral brakes forward motion; LOWER toward forward
  brakes backward motion. Threshold symmetric at +/-1.5 rad/s, deploy fast
  (1.5 frac/s) / relax slow (0.4), clamp raised to 0.40.
- **Standup surge, second look**: better than 164837 (drift +4.3 vs +10)
  thanks to the ramp-phase velocity damper, but still towing: the base
  ramp climbs regardless of whether the robot keeps up. Fix: the ramp now
  PAUSES while wheel speed exceeds ~2 rad/s (velocity-gated ramp rate) --
  the setpoint waits for the robot instead of dragging it.
- Lesson 35: **an actuator resting at its end-stop has one-sided
  authority; bias the rest pose off the stop to get both directions.**

**Runs 11-12** (`bal_20260702_172527.csv`, `bal_20260702_173132.csv`, both
untethered standup runaways to the bench edge):
- **Root cause: USB CDC blocking writes.** The AtomS3R uses native USB CDC
  (ARDUINO_USB_CDC_ON_BOOT); with the cable UNPLUGGED, Serial writes block
  up to the default TX timeout per chunk. Print bursts (engage messages,
  verified-write retries, the 2s debug dump) froze Core 1 -- the telemetry
  shows 0.9-6.8s gaps in the 50Hz log, and all earlier untethered runs had
  1-1.5s gaps nobody noticed.
- While Core 1 was frozen: no setpoint management, no outer loop, no safety
  aborts, no RC switch handling -- but Core 0 kept driving the wheels at
  200Hz against a stale setpoint ~2 deg below equilibrium = sustained
  forward acceleration = runaway to the bench edge. The 7s "arms stuck at
  tip" in run 172527 was the same freeze.
- Fixes:
  1. `Serial.setTxTimeoutMs(0)` -- CDC writes now drop when no host is
     attached instead of blocking. THE fix.
  2. **Dead-man on Core 0**: update() stamps a heartbeat; if it goes stale
     >300ms while balancing, balanceTick zeroes the wheels instead of
     driving blind. A fall is survivable; a runaway is not.
  3. Ramp velocity gate floored at 30% (a fully paused ramp holds the
     setpoint below equilibrium -- itself a source of acceleration).
- Lesson 36: **debug I/O is part of the control loop.** Untethered behavior
  differed from tethered for weeks of wall-clock tuning without anyone
  noticing the logs' time gaps. Check log timestamps for continuity, not
  just values.

**Run 13** (`bal_20260702_173619.csv`, 21.2s, "fell for no reason"):
- The CDC fix shrank the stalls (2s -> 0.7s) but did not eliminate them:
  Core 1 froze again for 739ms at t=20.3 while the robot stood perfectly
  (roll 88.85, vel -0.4). The v1 dead-man then STOPPED THE WHEELS at 300ms
  into the stall -- turning a balanced robot into an unpowered pole. It
  emerged at 83 deg falling and could not recover. The dead-man caused
  exactly the failure it was meant to prevent.
- Lesson 37: **degrade gracefully, don't amputate.** The 200Hz inner PD on
  Core 0 holds balance fine on a briefly-stale setpoint. Two-stage dead-man:
  >300ms stall -> keep balancing with wheel authority clamped to 8 rad/s
  (no runaway possible); >1.5s -> stop wheels (no safety, no RC = give up).
- Remaining stall source unknown (serial is now non-blocking). Suspects:
  AsyncTCP/WiFi task starving the Core 1 loop, I2C hiccup in M5.Imu.update.
  Mitigations + instrumentation added:
  - Display 25 -> 5 fps and WebSocket 10 -> 1 Hz while balancing.
  - Loop profiler: per-section worst-case times (imu/ctl/disp/ws/tel) plus
    max loop-to-loop gap, dumped as # prof_* lines in `bal log`. A big
    loopgap with small section maxima = external task starvation; a big
    section max names the offender.

**Run 14** (`bal_20260702_174401.csv`, 56.0s, "basically perfect"):
- Best run to date: standup travel only +2.2 rad, taps absorbed both ways,
  drift bounded +/-10, ended at +0.3 rad from origin. Roll std 0.85 (taps
  included).
- **Two-stage dead-man validated the hard way**: the profiler caught a
  4.8s(!) Core 1 stall right at engage (prof_ctl=5.5s -- INSIDE the control
  tick, so our own code, not WiFi starvation). The robot rode through it
  invisibly. Sub-section profiling added (crsf/can/bal/adrv) to name the
  blocking call next run; prime suspect is the verified-write/mode-switch
  path in enterBalancing.
- Operator items fixed:
  1. Arm authority asymmetric (0.28 up / 0.12 down): neutral moved to the
     center of the range (bias 0.20, symmetric +/-0.20).
  2. Jerky arm motion: constant-rate slew replaced with asymmetric
     exponential smoothing (tau 120ms deploy / 600ms release) -- no
     velocity discontinuities.
  3. Standup roll: wheel origin now stays at ENGAGE (not reset at ramp
     complete), so the position loop walks the robot back to where it was
     stood up. The travel itself is physics (tilt change requires wheel
     impulse); the return undoes it.

**Run 15** (`bal_20260702_175240.csv`, 41.8s, ended rolling off the stand):
- Standup roll-away-and-return working as designed (operator observed it).
  Arms confirmed helping both directions.
- Big push at the end: assist railed at its +clamp for 2.7s -- authority
  limited. Raised: bias 0.20 -> 0.30, max 0.40 -> 0.60 (symmetric +/-0.30),
  gain 0.12 -> 0.18, arm speed 6 -> 8 rad/s.
- Jerk diagnosed: arm target accel p95 was 25 rad/s^2 -- the assist target
  itself follows threshold-band velocity ripple. Added a dedicated 200ms
  LPF on the assist's velocity input (the main cascade keeps its faster
  55ms filter), deploy tau 120 -> 150ms.
- Sub-profiler first results: the balance run itself was gap-free except
  the known ~650ms engage stall (mode-switch/verified-write path, benign --
  robot is arm-supported). A 6.5s CRSF-section stall appeared during the
  post-crash chaos; pattern suggests loop preemption (async_tcp/WiFi) rather
  than blocking code -- dead-man covers it. Keep watching prof_* lines.

**Run 16** (`bal_20260702_222841.csv`, 52.5s):
- Stood up and balanced perfectly; push tests worked but operator felt the
  arm "center" sits forward of true top-dead-center and braking is stronger
  one way. Data agrees: 4.6s pinned on the LOWER assist clamp, 0s on the
  upper. Bias 0.30 parks the arms well forward of vertical, where the CG
  authority per rad is smaller and asymmetric.
- **Fix: neutral moved to TRUE TDC** (bias = 1.0 = the calibrated center
  pose), symmetric range +/-0.50 around it (arms may swing past vertical
  toward the calibrated backward pose; axis decomposition clamp extended
  to 1.6). CG authority per rad is maximal and symmetric at TDC.
- Arms now return from the tip pose DIRECTLY to the TDC stance during the
  rate-limited arm return (no post-capture hoist transient); assist frac
  initializes at bias so its first tick is not a step. Resting tilt drops
  to ~84 deg (arm mass no longer forward) -- legitimate equilibrium,
  handled by the curve + trim.
- Arm-target log precision raised to 3 decimals (the "jerk" accel metric
  was floored at exactly the 0.01-rad quantization limit -- measure before
  trusting).
- prof note: 5.6s balance-section stall traced to the LittleFS log flush at
  disengage (200KB write) -- after balancing ends, benign. 9.9s CRSF-section
  stall during idle/crash aftermath still unexplained; dead-man covers it.

**Run 17** (`bal_20260702_223919.csv`, 58.4s, rock-solid balance at a
completely wrong stance):
- **Reference-frame mixup**: the arm "forward" pose is defined with the
  robot on all fours; when the robot tips up ~90 deg to balance, that pose
  points straight UP -- it already IS top-dead-center. The calibrated
  "center" pose is ~100 deg BACKWARD when standing. Bias=1.0 therefore
  parked the arms at maximum reverse lean; the robot dutifully found
  equilibrium at roll 78 (quiet std 0.63, drift bounded +/-6, ended -0.4
  from origin) -- the architecture absorbed a huge stance error without
  complaint, which is its own kind of validation.
- **Fix**: bias back to 0.0 (forward pose = standing TDC = idle stance);
  assist swings +/-0.30 around it -- positive toward the calibrated center
  (arms back, brakes forward motion), NEGATIVE forward of vertical (brakes
  backward motion). Negative side is mechanically unverified; range kept
  conservative until bench-checked. Axis decomposition clamp now [-0.5,1.2].
- Arm return goal back to the forward reference (unchanged from original
  design -- which was already correct).
- Stored trim reset to -1.0 (the wrong-stance runs had walked it to -3.0).
- Lesson 38: **poses are frame-dependent; "forward" in the driving frame is
  "up" in the balancing frame.** Verify geometry against the physical robot
  before moving reference stances.

**Run 18** (`bal_20260702_224649.csv`, 54.0s -- struggled standup, weak
push recovery, operator hand-saves):
- **The whole setpoint frame is ~5 deg stale.** Quiet equilibrium measured
  82.9 deg vs the 89.3 anchor: the sp_offset integral sat railed near -8
  ALL RUN just to reach the true balance point. The CG moved (battery
  reseat after the 164837 ejection is the prime suspect) and every anchor
  was still pre-move. Consequences: standup ramps toward a setpoint 5 deg
  too high (the runaway the operator caught), and the outer loop has no
  integral headroom left for actual corrections.
- **Fix: anchors re-zeroed** -5.3 deg (FWD 89.3 -> 84.0, curve + tip +
  center shifted identically to preserve slopes), stored trim reset to 0.
  Lesson 39: **a railed integrator is a calibration debt.** If the trim/
  integral parks at a large value, fold it into the anchors -- the learner
  is for residuals, not for carrying the whole frame.
- **Push recovery was arm-starved by the smoothing chain**: a -7 rad/s
  shove peaked the arms at only 0.17 of 0.30 range -- the 200ms input LPF +
  1.5 threshold + 0.18 gain + 150ms deploy tau meant the arms were still
  winding up when the 0.9s event ended (meanwhile the wheel setpoint railed
  at -8). Operator's read is correct: fast, authoritative arm motion is the
  key to pushes. Rebuilt: input LPF 200 -> 80ms, threshold 1.5 -> 1.0,
  gain 0.18 -> 0.35 (full deploy at 2 rad/s excess), deploy tau 150 -> 80ms,
  arm speed 8 -> 12 rad/s, range +0.45/-0.30 (asymmetric: positive side
  proven, negative awaits bench check). Release stays slow (600ms).

**Run 19** (`bal_20260702_225359.csv`, 38.8s, arms too aggressive -- never
settled): the fast-arm rebuild overshot. Clean 0.5 Hz limit cycle: arm
fraction, roll (+/-2.7 deg), and wheel velocity (+/-4.5 rad/s) all locked
at 0.5 Hz, arm-velocity correlation +0.88 at 240ms lag -- the arms were
FOLLOWING velocity they themselves created, a textbook self-excited
oscillator. Threshold 1.0 rad/s sat inside the normal settle band, so the
arms engaged on ordinary settling motion; the 600ms release kept them
coupled across half-periods; 240ms round-trip lag set the frequency.
- Note: the anchor re-zero direction was validated (sp settled ~85), and
  the run also railed the negative arm clamp constantly (mean frac -1.07?
  -- measurement rel. to an unsettled reference, treat with care).
- **Fix -- keep the speed, gate the entry, shorten the tail**:
  threshold 1.0 -> 2.0 rad/s (outside the settle band: below it arm gain
  is ZERO and the proven wheel-only cascade rules, so the cycle cannot
  sustain), gain 0.35 -> 0.30, input LPF 80 -> 120ms, release 600 -> 350ms.
  Deploy tau (80ms) and arm speed (12 rad/s) unchanged -- pushes still get
  the fast throw.
- Lesson 40: **a fast strong helper needs a strict trigger.** Speed and
  authority are safe only OUTSIDE the band where the primary controller
  is already stable; inside it they become the disturbance.

**Run 20** (`bal_20260702_225944.csv`, 17.4s): settle FIXED (roll std 0.55,
arms still at rest -- the oscillator is dead) but now too slow on pushes:
a -3.1 rad/s push NEVER deployed the arms. Cause: the raw velocity peaked
~3.1 but after the 55ms cascade filter + 120ms assist filter the assist's
input never crossed the 2.0 threshold before the wheels had already
absorbed the event. Threshold and lag double-gated the arms.
- **Fix -- one gate, not two**: assist input LPF cut to 50ms (the cascade
  filter already smooths), threshold 2.0 -> 1.4 (above the 1.0 settle band
  with margin, below real-push peaks), gain 0.30 -> 0.40 so crossings get
  a committed throw. Deploy tau 80ms / release 350ms / speed 12 unchanged.
- Lesson 41: **series filters gate twice.** Every filter ahead of a
  threshold raises the effective threshold for short events; budget total
  latency, not per-stage smoothness.

**Run 21** (`bal_20260702_231038.csv`, 52.5s): standup rolled hard (sp dived
to 78.9 chasing the tip anchor while true equilibrium was ~88.2 -- the
anchors move BETWEEN standup attempts), push response good, then arms
flip-flopped (3 sign reversals) and the robot could not settle.
Two operator-designed fixes implemented:
1. **Capture self-calibration**: a SETTLED capture (err<1 deg, rate<4 dps,
   400ms quiet, at the tip stance before arm return) is a direct
   measurement of the true equilibrium for THIS attempt. The whole curve
   is re-zeroed to it: run_curve_shift = captured tilt - scheduled sp,
   clamped +/-6, applied for the entire run and folded into the persisted
   trim. Fixed anchors only provide the SHAPE; the absolute level is
   measured fresh at every standup. (Timed-out captures skip the
   calibration and fall back to stored trim.)
2. **One-shot arm assist** (quick response, slow return, no back-and-forth):
   arms attack fast (80ms) on first trigger, then relax monotonically to
   neutral (tau 1.0s). Opposite-direction demands are IGNORED until the
   arms are back near neutral -- the counter-swing belongs to the wheels.
   Kills the deploy/counter-deploy flip-flop structurally rather than by
   gain tuning.
- Lesson 42: **the robot measures its own balance point at every capture;
  use it.** Fixed anchors drift between attempts (battery seat, surface);
  the settled capture is ground truth, free, once per standup.

**Run 22** (`bal_20260702_232626.csv`, 42.6s, committed at 9f579c8):
- **Capture calibration works**: measured 84.99 at tip -> predicted 87.9
  arms-forward; actual equilibrium 88.35. Error 0.45 deg (was 5 deg).
- Standup still towed +13 rad: the HIGH-slope velocity gain (tap recovery)
  engaged on the mandatory standup glide and whipped the setpoint 1.5 deg
  past equilibrium. Fix: only the low slope is active until ramp complete.
- Push response: arms threw 0.44 in ~200ms, forward motion killed in 0.5s
  -- then **overcorrection fall**: the wheel P whipped +6.6 -> -2.0 deg
  while the still-deployed arms were ALSO pulling the setpoint down via
  the center term (-2.8 deg). Setpoint dove 9 deg in 0.5s; robot ran
  backward -10 rad/s and fell.
- Fixes: (1) wheel velocity-P authority scales down by up to 60% while
  arms are deployed (actuator coordination -- the corrections were
  stacking), (2) sp_offset rate 16 -> 12 deg/s.
- Lesson 43: **two actuators correcting the same error double-count.**
  When a secondary actuator deploys, the primary must yield authority
  proportionally, or the combined response overshoots.

**Run 23** (`bal_20260703_093643.csv`, 2.3s, two failed standups, safety
abort): a 430ms Core 1 stall hit at the most fragile moment (arm return +
base ramp both in progress). Chain: robot fell forward during the stall;
the dead-man SOFT clamp (8 rad/s) throttled the catch (cmd pinned at
exactly 8.0); Core 1 woke, yanked it back, overshot backward past the
recovery envelope (cmd railed -30, vel -16, rate +150); fell. Meanwhile
the arm return marched the CG down through the whole fight.
- Fixes:
  1. Soft dead-man clamp 8 -> 20 rad/s: stale-setpoint creep is <8 rad/s,
     so the tight clamp never protected against creep -- it only throttled
     genuine catches. (Second time the dead-man's response caused the
     failure it guards against; see lesson 37.)
  2. Arm return pauses while |cmd|>10 or |rate|>30 dps -- the return
     starts calm (capture gate) and now STAYS calm.
- prof: bal 905ms (mostly the known engage mode-switch), crsf 844ms
  (unexplained -- preemption suspected). Sub-second stalls remain a fact
  of this firmware; the balance loop must survive them, and now does with
  full catch authority.
- Lesson 44: **degraded modes must not throttle the recovery they exist
  to enable.** Bound what the failure mode actually produces (small creep
  commands), not what the healthy controller needs (large catch commands).

**Run 24** (`bal_20260703_094448.csv`, 52.5s): four findings, four fixes.
1. **Standup tow (+15.7 rad)**: capture calibration nailed the level, but
   the CSP-era curve SHAPE (flat middle, dip at frac 0.15) made the sp lag
   the true equilibrium through the arm return -- gliding forward at up to
   5.5 rad/s mid-return is the direct signature. Curve replaced with a
   monotonic tip->fwd rise (84.0 / 82.5 / 81.1); level comes from capture.
2. **Best stability ever recorded**: quiet roll std 0.28 deg, dominant
   motion 0.15 deg at 0.23 Hz (the position loop breathing). Left alone.
3. **Arm kick-in latency ~0.35s** (threshold cross to useful deployment):
   input filter cut to ~raw (tau 20ms) -- the one-shot latch tolerates
   noise blips, so filter lag buys nothing.
4. **Backward push overcorrected (-10 rad)**: the sign-latch left the arms
   stranded positive (relaxing from the prior forward push) so the
   backward push got wheels-only at full whip. Latch amended: strong
   opposite demand clears the arms out FAST (2x attack tau), and
   re-deploy unlocks at |dev|<0.10 instead of 0.03. Still never swings
   through neutral -- anti-flip-flop preserved.
- Lesson 45: **back-to-back opposite disturbances are the norm, not the
  exception** (every push has a catch-recoil). One-shot latches need a
  fast hand-off path for the counter-direction.

**Run 25** (`bal_20260703_095326.csv`, 54.6s): the fast-arm changes brought
the oscillator back at 0.56 Hz -- arms deployed 83% of the time, 89
direction reversals, arm-vel correlation +0.79 at 340ms. Bumps in BOTH
directions handled well (the handoff works); the arms just never leave the
loop afterward. Standup tow still present (noted, deferred).
- **Fix: arm engagement state machine** (READY -> ACTIVE -> HANDOFF ->
  COOLDOWN). The discriminator between an external bump and self-
  oscillation is CALM: a bump arrives out of calm; an oscillation never
  re-establishes it. One event = at most two swings (push + recoil
  handoff), then arms hold neutral until |vel_err|<1 and |rate|<15 dps
  for 400ms re-arms them. Full-speed response preserved from READY;
  sustained oscillation is structurally denied arm participation.
- Lesson 46: **gate the helper on the event boundary, not on the signal.**
  Signal-level gates (thresholds, filters, latches) all eventually leak
  during sustained excitation; an explicit engagement lifecycle with a
  calm-based re-arm cannot.

**Run 27** (`bal_20260703_222223.csv`, 23.4s, evening session):
- **The lifecycle works end-to-end**: tap #1 -> arm throw +0.42, one recoil
  handoff, then 3.5s later the STILLEST balance on record (roll pinned at
  87.85 +/- 0.05, cmd 0.0 for 3+ seconds). Tap #2 recovery was mid-flight
  and healthy when the run ended.
- **The "tipped over" was a spurious safety abort, not a control failure**:
  a 449ms Core 1 stall made the wheel-feedback timestamps (updated by
  Core 1 itself) look stale; the CAN-death abort fired mid-recovery and
  stopped the motors. Fix: 500ms grace on the stale check after the loop
  wakes from a stall (real CAN death still aborts -- loop running, feedback
  aging). Lesson 47: **a watchdog that shares a failure domain with what
  it watches will false-positive** -- feedback age measured by a stalled
  clock is not feedback age.
- **Standup tow is physics**: displacement to tilt back delta-theta is
  ~(B/A)*delta_theta regardless of speed -- predicted ~14 rad, measured
  +11. Can't be tuned away; compressed instead: arm return 1.5 -> 2.5
  rad/s, base sp rate 3 -> 4 deg/s (operator call). The position loop
  walks it back afterward.

**Run 26 feedback** (state machine validated on-robot): disturbance
response confirmed good with settling restored. Operator tweaks applied:
- Release tau 1.0 -> 0.65s (~50% quicker return to neutral).
- **Emergency arm throw**: if the wheels are railed (>=90% of max cmd)
  while velocity error is still above the arm threshold, a roll-away is
  in progress and the wheels have nothing left -- the arms bypass the
  engagement lifecycle and throw to their FULL stop in the braking
  direction. Exits as a normal ACTIVE engagement (relax + cooldown).
  Wheel saturation is the one unambiguous "use everything" signal: no
  double-counting concern because the primary actuator is pinned.
- Standup weirdness still open (curve shape refit pending, needs a few
  more captures for data).

### Phase 15: Sim-First Fixes -- Core 1 Stalls + Standup Roll-Away (Jul 3, design record)

Both open items from Phase 14 attacked offline before the next robot
session, per the sim-first plan.

**Model refit from Speed-mode data only** (`fit_balance_model.py
--speed-only`, 23 runs): A=8.8 1/s^2 (unstable pole 3.0 rad/s -- the CSP-era
1.22 was closed-loop-bias garbage), B=5.9 deg per rad/s^2, true small-signal
motor velocity-loop lag tau_m=60ms (trajectory fit with the 100 rad/s^2 acc
limit modeled; the naive fit said 200+ms because large excursions conflate
the acc limit and the 160ms round-robin feedback staleness). Quiet-window
noise: roll std 0.15 deg, rate std 1.2 dps, wheel vel std 0.33 rad/s.
`model_fit.json` refreshed.

**Simulator built** (`scripts/balance_sim.py`): 200Hz inner PD + motor lag +
acc limit + 50Hz outer tick ported line-for-line from balance_controller.cpp
(ramp + vel gate, capture shift fade, capture self-cal, dual-slope PI,
gates, shed, arm-assist lifecycle), with Core 1 stall injection (outer tick
freezes, Core 0 keeps running the two-stage dead-man) and floor/body push
injection. Validated against three logged failures before use:
1. Standup tow: sim reproduces 5-10 rad of ramp-phase drift with current
   constants (logs: 6-16).
2. Run 23 mechanism: a -40 dps shove that is caught 5/5 without a stall
   falls 3/5 when a 0.6s stall lands during the arm return.
3. Run 13 dead-man: v1 (wheel stop at 300ms) falls where v2 (two-stage)
   survives the same 740ms stall.
Documented sim caveat: with the fitted A the quiet-stance limit cycle is
~3 deg std vs 0.3-0.9 real -- absolute stability is pessimistic, so variants
are compared relatively and "standup failure" is scored only through
ramp-complete+3s.

**Standup roll-away root cause confirmed in the traces**: during arm return
+ ramp the scheduled sp sits 1-4 deg above the actual roll continuously; in
Speed mode that error IS commanded velocity (Kp=2 rad/s per deg), so the
robot glides 2-5 rad/s for the whole ramp. Three contributors, three fixes
(sim matrix, 54 cases each: 3 A-values x 3 eq-shift values x 3 capture
offsets x 2 seeds):
1. **Curve shape refit**: the true tip->fwd equilibrium rise is ~1.9 deg
   (per-run measured +1.0..+3.3), not the 2.9 the anchors assumed. Anchors
   now 84.0 / 83.05 / 82.1 (ARMS_TIP raised to 82.1).
2. **Proportional arm return**: equal-rate return finished the short (right)
   arm first; the axis decomposition read the imbalance as a center
   excursion and the equilibrium dipped ~1 deg mid-return for nothing.
   Per-arm speeds now scale with remaining distance so both arms land
   together (also cut median ramp time 3.1 -> 2.3s).
3. **Early position P**: the position loop now runs THROUGH the ramp
   (origin at engage, gain 0.03 vs 0.05, clamp 0.6 vs 1.0 rad/s) so drift is
   opposed as it develops. Integrator stays ramp-gated -- no wind-up.
Combined: standup failures 5/54 -> 2/54, median |drift@ramp| 3.2 -> 2.4 rad,
median peak tow 4.7 -> 3.8 rad in sim, push response unchanged.
- **Rejected: velocity-gating the arm return** (pause while gliding): it cut
  the tow further but TRIPLED standup failures. The return RAISES the
  equilibrium toward the robot -- pausing it during a glide blocks the very
  self-correction that ends the glide. Lesson 47: **the arm return is part
  of the correction, not a disturbance; never gate it on the symptom it
  cures.**
- **Rejected: carrot ramp / eq-tracking standup**: capping the effective
  setpoint to tilt +/- lead (or replacing the ramp with a velocity-zeroing
  tracker) collapsed in the matrix -- both interact with the catch
  transient, where velocity has glide semantics inverted (a braking
  transient reads as "sp too high" and walks the sp the wrong way).

**Core 1 stall, cause 1 (every run): the engage-time mode switch.** Every
July log has a 0.4-7.3s gap at exactly the enterBalancing tick -- the
blocking CSP->Speed switch (fixed delays + up to 4x30ms verified-write
retries per param, all on the 50Hz loop). Fixed by **moving the switch to
the START of tip-up**: the robot is static on all fours, blocking there
costs nothing, and the verified current-limit writes complete before the
robot ever lifts (a failed switch now refuses the standup instead of
hard-aborting a robot already up on its arms). A 250ms 0-speed keepalive
during the ~9s tip keeps the motor-side CAN watchdog fed. Force-engage
(double-tap, no tip-up) keeps the blocking switch. enterBalancing is now
essentially instant (setDriveRunMode no-ops when the mode already matches).

**Core 1 stall, cause 2 (sporadic, 0.4-2.5s mid-balance): preemption.**
The Async TCP library defaults to priority 10 on ANY core; the Arduino
loopTask (the whole 50Hz loop) is priority 1. Fixed with a full
**control-core / comms-core split**:
- Core 0 (comms): WiFi + lwIP (already pinned there by the framework),
  async_tcp pinned via `-DCONFIG_ASYNC_TCP_RUNNING_CORE=0`, priority
  dropped to 3.
- Core 1 (control): the 200Hz PD task moved here at priority 18; the 50Hz
  control tick (serial commands, IMU, CRSF RX/TX, CAN, arming, balance
  state machine, arm/drive) moved out of loopTask into a dedicated task at
  priority 12; loopTask (priority 1) keeps display, WebSocket telemetry,
  and the debug burst -- preemptable by control, never the reverse.
Serial commands stay on the control task so CAN access remains
single-threaded.

**Stall forensics** (replaces guessing): any control-tick gap >100ms is
recorded in a ring (dumped as `# stall_*` lines with `bal log`) with the
profiler section that grew (own blocking code names itself), the balance
state, and the **sentinel gap**: a priority-24 task on the control core
stamps every 10ms. Sentinel gapped too = whole core dark (flash-cache stall
from a LittleFS write, or interrupts off); sentinel kept ticking = control
was blocked or preempted below 24. (Naming the preempting task directly via
runtime stats is off the table -- this framework build ships without
CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS.) New `# prof_ctlgap_max_us` is the
headline stall metric; flash-write audit confirmed trim persist and log
flush only happen on exit paths.

**Stall-injection campaign** on the fixed controller: rides through stalls
up to ~1.2s in any phase (capture / return / ramp / steady); >=2s stalls
mostly fall -- dominated by the frozen outer loop and the 1.5s hard stop,
which is correct behavior (no RC, no safety = give up). Dead-man params
verified in sim: soft clamp value 12/20/30 indistinguishable (keep 20);
extending the hard stop 1.5->3s only helps marginally and costs the safety
argument (keep 1.5); **Core 0 ramp-continuation during stalls: no effect**
with the fixed standup (the sp is no longer parked below equilibrium when
stalls hit) -- rejected, no new Core 0 complexity.

**Bench validation ladder for the next session** (minimal robot time):
1. **Idle soak** (10+ min, WiFi client attached, dashboard open, drive
   armed): `bal log` must show `# stall_events=0` and
   `# prof_ctlgap_max_us` < ~30000. Also confirms 200Hz/50Hz co-existence
   on core 1 (watch the `[Loop] avg/max` line).
2. **Wheels-up tip-up + engage/disengage cycling**: verify the pre-switched
   Speed mode -- wheels should hold still against a hand push during the
   tip (velocity servo), and the telemetry gap at t~8.9s should drop from
   0.4-7.3s to ~0. Then one wheels-down tip-up to confirm the wheels hold
   against the arm push on the real floor.
3. **Standup trials** (3+): expect standup drift < ~3 rad (vs +10-16),
   both arms reaching forward together, no mid-return equilibrium dip.
   Pull telemetry, re-run `fit_balance_model.py --speed-only` and
   `balance_sim.py validate`; iterate constants only if sim and robot
   disagree.
4. **Push tests** unchanged from the Phase 14 ladder (arm assist and outer
   cascade were not retuned).

**Test ladder for the first Speed-mode session** (one variable at a time):
1. **Wheels-up mode switch**: robot on a stand, drive armed. Serial:
   `mode 20 2` / `spd 20 1.0 2.0` / `mode 20 5` to verify Speed-mode entry,
   spin, and CSP restore per motor. Then `bal engage` + immediate disengage
   to verify the automatic switch/restore path both directions.
2. **Force-engage hold** (double-tap Ch11) at the known balance point:
   expect capture parity with the CSP baseline (<0.3 deg tracking). If the
   inner loop buzzes or lags, tune motor `spd_kp`/`spd_filt_gain` or lower
   inner Kp before proceeding.
3. **Full tip-up sequence**: verify capture, arm return, and setpoint ramp
   with the new arm curve. Watch `pre_rate`/`pre_cmd` in the analyzer.
4. **Push disturbance**: firm push while balancing. Expect wheel translation
   absorbing the push, then return toward origin (watch `target_vel` and
   `sp_offset` in telemetry -- sp_offset should return to a small steady
   value, not wind up like the old vel_trim).
5. **Return-to-origin**: displace the robot ~1 m; drift should converge back
   within +/-1 rad over tens of seconds (slow pole by design).
   Success bar: 3+ min balance, drift held within +/-1 rad, survives a push.
   After the session: `scripts/save_telemetry.sh`, then re-run
   `fit_balance_model.py` on the Speed-mode logs to re-fit tau_m and re-check
   the gain recommendations.

---

## Telemetry Files (Chronological)

| File | Duration | Notes |
|------|----------|-------|
| `latest_run.csv` | 21.3s | Pre-200Hz best run, adaptive sp found 91.0 |
| `200hz_balanced_16s.csv` | 16.3s | Early 200Hz, gentle outer loops |
| `200hz_feedforward_test.csv` | - | FF test, confirmed balance shifts |
| `200hz_base87_close_run.csv` | ~30s | Close to stable with base=87 |
| `bal_20260409_220324.csv` | 30s | **Best old-arch run**: 98% < 3 deg, velocity bias removed |
| `bal_20260409_221043.csv` | 2.8s | First velocity integrator test, sign wrong |
| `bal_20260409_221845.csv` | 2.7s | Velocity integrator sign fixed, still too slow |
| `bal_20260409_222714.csv` | - | Heavier filter, still forward runaway |
| `bal_20260409_223214.csv` | 21.3s | Position shift fighting velocity integrator |
| `bal_20260409_224632.csv` | 21.3s | Position shift removed, cleaner but still drifts |
| `bal_20260409_225509.csv` | 21.3s | Fast arms (10 rad/s), better disturbance handling |
| `bal_20260409_225753.csv` | 10.5s | Integrator gain too high (2.0), positive feedback runaway |
| `bal_20260409_230538.csv` | 17.8s | Gain=1.0, filter heavy, slow to find balance |
| `bal_20260409_230938.csv` | 34.0s | Nonlinear filter, Kp=1.5, good 5s stable then runaway |
| `bal_20260409_231344.csv` | **48.9s** | **BEST RUN**: Kp=1.5/Kd=0.08, converged at 92.5, 0.21 avg err |
| `bal_20260409_232108.csv` | ~3s | Setpoint seed to 92.5 while arms still at tip -- immediate runaway |
| `bal_20260409_232735.csv` | 21.1s | Arm-scheduled setpoint, trim drifting wrong direction |
| `bal_20260409_233040.csv` | 12.0s | Kp=2.0, 4s perfect balance, then trim sign bug caused runaway |
| `bal_20260410_200016.csv` | 15.5s | Capture-gated arm return worked (2.56s delay), but wall bouncing/drift remained |
| `bal_20260410_200622.csv` | 9.4s | Early delayed-position-hold test; capture clean, but roll-away still present |
| `bal_20260410_201026.csv` | 22.9s | Best April 10 post-capture run; balanced well, but drift reached ~-15 rad before hold stabilized |
| `bal_20260410_201448.csv` | 5.7s | **Bad velocity-damping experiment**: severe oscillation/saturation, reverted |
| `bal_20260410_204824.csv` | 23.7s | Always-active pos PI (Kp=0.40): persistent ~2Hz oscillation, Kp too aggressive for inner loop |
| `bal_20260410_205347.csv` | 23.7s | Pos PI (Kp=0.10): clean initial balance, then trim+PI positive feedback runaway to sp=99 |
| `bal_20260410_210127.csv` | 7.4s | No trim: engaged at 76 deg, capture clamp too tight, no adaptation, violent oscillation |
| `bal_20260410_210747.csv` | 20.0s | Trim+decay: clean start, but 0.99 decay too weak, trim+PI still compounded to sp=97.9 |
| `bal_20260410_211316.csv` | 33.2s | **No trim + wide capture**: correct architecture, 0.1 deg balance, but PI too slow; drifted to bench edge |
| `bal_20260410_222145.csv` | 38.9s | Stronger PI gains: excellent balance, but position-controlled motors don't translate from tilt alone; stuck at wall |
| `bal_20260410_223145.csv` | 34.7s | Gate bug fix + velocity offset: no improvement, PI gate was self-throttling |
| `bal_20260410_223740.csv` | 41.5s | **Sign fix + I-heavy**: first return-to-origin! But Ki=0.20 caused massive overshoot (+7 rad) |
| `bal_20260410_224228.csv` | **50.4s** | **NEW RECORD**: Kp=0.15/Ki=0.12/Kd=0.05, stable balance at sp=84. But angle-only shift barely translates |
| `bal_20260411_091031.csv` | 7.7s | Origin reset + no rate limit: overshoot to 98, -6.35 rad drift in 1s |
| `bal_20260411_092019.csv` | 11.5s | Rate limit 5 deg/s: ramp gentler but PI interfered during ramp, amplified drift |
| `bal_20260411_092847.csv` | 20.6s | PI gated until ramp done: correct, but 5 deg/s still too fast; PD saturated during ramp |
| `bal_20260412_163928.csv` | 36.0s | First arm-balance-assist run: 95.6% stable, drift -6.7 rad |
| `bal_20260412_164317.csv` | 29.4s | Arm assist hit clamp (0.60 frac bug pre-limit), drift 12.9 rad |
| `bal_20260412_164912.csv` | 52.6s | **Log-censored (still balancing at cutoff)**: 87% stable, arm assist active |
| `bal_20260412_170823.csv` | 4.5s | Early fall during capture |
| `bal_20260412_171517.csv` | 26.9s | 96% stable but drift ran to -12.1 rad |
| `bal_20260412_172103.csv` | 50.2s | Near-censored, 86.8% stable, drift ended -0.8 rad by luck |
| `bal_20260412_173405.csv` | 2.8s | Failed capture, arm return before settle |
| `bal_20260412_174731.csv` | 51.6s | **Log-censored**, vel_trim=0.1 added: 93.8% stable, max drift 11.8 rad |
| `bal_20260412_175422.csv` | 51.3s | **Log-censored**, 91% stable, drift -10.9 rad |
| `bal_20260412_180129.csv` | 11.4s | Disturbance fall |
| `bal_20260412_180748.csv` | 51.3s | **Log-censored**, 88.7% stable, drift bounded ~8 rad |
| `bal_20260412_183346.csv` | 17.8s | Arm return before settle, fell |
| `bal_20260412_183812.csv` | 44.0s | 94% stable but runaway drift to -27.7 rad |
| `bal_20260412_184543.csv` | 36.6s | vel_trim gain 0.15: 99.3% stable, drift -8.9 rad |
| `bal_20260412_184913.csv` | 26.9s | vel_trim gain 0.25: 98.5% stable |
| `bal_20260412_185218.csv` | 34.8s | **Best stability of the day: 99.9% < 2 deg**, drift -5.0 rad |
| `bal_20260412_185728.csv` | 47.5s | vel_trim gain 0.4: trim climbing (1.73), stability held 95.8% |
| `bal_20260412_190131.csv` | 33.4s | vel_trim gain 0.4: 96.9% stable |
| `bal_20260412_191828.csv` | 51.3s | **Log-censored**, back to gain 0.1, drift ended -0.3 rad |
| `bal_20260412_193312.csv` | 18.9s | Gain 0.2, fell after disturbance |
| `bal_20260412_193809.csv` | 49.0s | Gain 0.2: 98% stable, drift -7.9 rad |
| `bal_20260412_194229.csv` | 35.8s | **Worst drift recorded: -60.4 rad** while 88.7% "stable" -- textbook CSP translation failure |
| `bal_20260412_194831.csv` | 23.3s | trim 1.23 climbing, fell |
| `bal_20260412_200050.csv` | 19.4s | 73% stable, oscillation after disturbance |
| `bal_20260412_202206.csv` | 20.0s | Arm return before settle, 70.5% stable |
| `bal_20260412_202543.csv` | 18.7s | trim 1.36 at fall |
| `bal_20260412_215957.csv` | 20.4s | Positive drift run (+11.8 rad), trim went negative correctly but no translation |
| `bal_20260412_220427.csv` | 49.8s | 95.5% stable, drift -5.6 rad |
| `bal_20260412_220940.csv` | 4.0s | Failed capture |
| `bal_20260412_221620.csv` | 17.6s | Force-engage test (eng_rate ~0), clean but trim climbed 1.5 |
| `bal_20260412_222511.csv` | 24.3s | Force-engage test, 86% stable |
| `bal_20260412_224607.csv` | 18.4s | odo_kp=0.3 added: trim pinned at +3.1 |
| `bal_20260412_224949.csv` | 19.7s | vel_trim gain 1.0: trim slammed to clamp (3.95), 65% stable, drift -26.5 rad |
| `bal_20260412_225413.csv` | 13.9s | Gain 1.0: trim at clamp 4.0, fell |
| `bal_20260412_225745.csv` | 19.9s | Gain 0.4 + odo 0.3: 55.6% stable -- odometry PID destabilizing |
| `bal_20260412_230134.csv` | 32.0s | 36.2% stable, trim -3.55, persistent oscillation |
| `bal_20260412_230544.csv` | 14.1s | Gains reduced (0.15/0.1): still 36.2% stable |
| `bal_20260412_230929.csv` | 22.8s | Final CSP-era run: 62.4% stable, drift -6.9 rad |

---

## Lessons Learned

1. **Gyro filter alpha=0.08 at 200Hz is non-negotiable.** Every lighter filter oscillated.

2. **The arm return shifts balance by ~5.5 degrees** (86.5 -> 92.0). Arm-scheduled setpoint is the answer.

3. **Velocity bias in the PD loop is fatal.** It cancels tilt recovery when drift is large. All position/trim corrections must go through the setpoint, not direct motor command addition.

4. **The PD needs Kp >> Kd near balance.** At Kp=1.0/Kd=0.12, the Kd term was 1.1x the Kp term near balance, creating sluggish corrections. Kp=2.0/Kd=0.08 gives Kp 5x Kd advantage.

5. **Command integrator sign**: positive motor_vel means sp > tilt (robot leaning forward of setpoint, sp too high). Trim must go NEGATIVE to lower it: `trim -= gain * cmd`.

6. **Nonlinear filter on command integrator** prevents positive feedback: alpha=0.01 for small oscillations (2s time constant), alpha=0.10 for large commands (0.2s).

7. **Fast arm return (10 rad/s) is better than slow.** Brief sharp impulse vs prolonged disturbance. The arm-scheduled setpoint tracks it perfectly regardless of speed.

8. **The 200Hz PD is rock-solid when the setpoint is correct.** Multiple runs confirmed < 0.3 degree error. Failures were always from setpoint management, never from the inner PD loop.

9. **The true balance point with arms at forward ref is ~92.0 degrees** (confirmed across multiple runs averaging roll during stable periods).

10. **Double-tap Ch11 for force-engage** allows testing balance at any arm position without tip-up.

11. **Arm return should be capture-gated, not timer-only.** April 10 testing showed fixed-timer arm return can work in clean cases, but capture gating is more reliable across engage angles.

12. **The low engage angle is a handoff condition, not a balance trim.** If the robot engages at 80-84 deg and that offset is preserved into arms-forward balance, it will chase the wrong setpoint and roll away.

13. **Position hold must start from a stable origin.** Locking the origin at balance engage includes the whole stand-up/arm-return translation. Current approach waits until arms are forward and the bot is calm, then resets the wheel origin.

14. **Raw wheel-velocity damping was destabilizing.** The April 10 velocity-brake experiment made balance much worse; avoid direct velocity-to-setpoint damping unless it is redesigned and heavily gated.

15. **Use `scripts/analyze_balance_logs.py` after every run.** Watch `eng_roll`, `ret_s`, `pre_rate`, `pre_cmd`, `trim`, `pos_shift`, `meas_drift`, and flags. The most useful failure signatures so far have been high pre-return rate/cmd, permanent trim near clamp, and large drift before position hold locks.

16. **Position correction must be always-active, not delayed.** Waiting for arms-forward + calm + settle timer means the robot has already rolled away 10+ rad by the time correction starts. The angle-error gate provides the necessary protection during the catch phase without requiring explicit delayed activation.

17. **The command integrator trim and position PI create a positive feedback loop.** When the robot drifts, the position PI adjusts the setpoint. The resulting sustained motor command causes the command integrator to also adjust the setpoint in the same direction. Both corrections compound, pushing the setpoint far from the true balance angle (observed: setpoint at 99 vs true balance ~92).

18. **Use a single outer PI on position, no separate trim integrator.** This is the standard inverted pendulum architecture (Segway, nBot, etc.). The position integral naturally handles balance-point offsets: if the scheduled setpoint is wrong, the robot drifts, the integral accumulates, and the setpoint corrects. One integrator eliminates positive feedback.

19. **Position-controlled motors break tilt-to-translation coupling.** In a torque/velocity-controlled Segway, holding a tilt off-balance requires sustained wheel acceleration, which inherently translates the robot. With Robstride motors in position mode, the motor applies holding torque at a fixed position -- the robot leans but the wheels don't move. A direct wheel velocity offset driven by drift error is needed to provide the translational coupling that position control eliminates.

20. **Velocity offset in CSP mode is mathematically equivalent to a setpoint shift.** Adding vel_offset to the motor command reaches equilibrium at `tilt = setpoint + vel_offset / Kp` where motor_vel=0. The PD absorbs the offset into a tilt angle change within a few hundred ms. No sustained wheel velocity occurs. This is identical to the setpoint-shift approach that was already proven insufficient.

21. **Any additive offset creates positive feedback with the PI.** Whether it's a command integrator trim (Phase 10) or a velocity offset (Phase 11), the tilt offset makes the PI see roll above/below setpoint, the PI adjusts, drift grows, the offset grows further. The feedback loop is structural in CSP mode.

22. **Leaky origin ("drift acceptance") prevents PI saturation.** Instead of fighting large accumulated drift, slowly move the wheel origin toward the current position (0.15/s, ~7s time constant). The PI only sees recent drift (1-2 rad max), never saturates, and the robot accepts its current location as "home" over time. This is a pragmatic fix for CSP mode's fundamental limitation.

23. **For true return-to-origin, switch to Speed mode.** In Speed mode, motor_vel directly commands wheel velocity (not integrated into position targets). The PD cannot cancel a velocity offset because there is no position target for the motor servo to hold. This is a larger architectural change for future work.
