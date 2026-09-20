# Balance Mode Tuning History

**Current workflow:** use the [Wi-Fi / OTA guide](../docs/WIFI_OTA.md) for firmware updates and validated `.csv`/`.wire` downloads, and the [balance test guide](../docs/BALANCE_TESTING.md) for physical trials. The installed combined release retains driving damping v4; [current state](../docs/progress/CURRENT.md) identifies the exact image and pending powered checks. The architecture, packages, USB commands and per-session “next” actions below are historical records, not current deployment instructions. Later entries record corrections without rewriting earlier observations.

## July Architecture (v14: Control-Core Split + Forensic Flight Recorder, July 2026)

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
  COOLDOWN; re-arms only after 300ms of genuine calm. External bumps
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
- Telemetry v2: complete 120s/6000-sample 50Hz PSRAM capture with 200Hz
  window diagnostics, raw/filter IMU comparison, full inner/outer command
  construction, motor/arm torque and feedback age, arm lifecycle, power,
  run note/markers, terminal reason, and checksum. Binary persistence and
  learned-trim writes are deferred until fully idle. The profiler is reset
  at run entry and frozen at exit, so `bal log` no longer records its own
  idle download stalls. `scripts/save_telemetry.sh` rejects corrupt or
  truncated transfers; `scripts/analyze_balance_logs.py --details` reports
  timing, authority, sensors, power, lifecycle, and safety evidence.

### Known open items
- The run-33 follow-up changes -- ramp-phase offset clamp +/-1.5 deg and
  drift return Kp 0.05 -> 0.08 -- are sim-validated and built but not yet
  robot-validated. Start with the stand-up baseline in
  `docs/BALANCE_TESTING.md`, then advance through its disturbance ladder.
- Runs 32/33 showed zero balance-time control stalls after the CRSF byte
  budget. If a new run records one, the run-scoped `sec:` / `sentinel_us:`
  fields distinguish own blocking code, preemption, and whole-core stalls.

### Test-round telemetry preparation (after Run 33)
- Replaced the misleading 120s/3000-sample configuration (which silently
  filled at ~60s) with a real 6000-sample capture covering the full 120s.
- On-flash format is a versioned binary record (1.32MB of samples) with an
  FNV-1a checksum. `bal log` validates it before CSV export; the host also
  requires the checksum and exact row count before accepting a file.
- Added capture-time gain/config snapshot, build ID, operator `bal note`,
  CH12/`bal mark` event alignment, end reason, and a final row immediately
  before every controller-detected safety exit.
- Added direct observability for the remaining tuning questions: raw vs
  filtered IMU, 200Hz cadence and clipping, command before/after clamps,
  every setpoint component, feedback age, arm lifecycle/demand/tracking,
  yaw saturation, motor torque, and bus voltage/current.
- Flash I/O and trim persistence now execute only after the robot and arms
  are idle. A new balance attempt is refused during the short save window,
  guaranteeing that every permitted test is instrumented and no flash
  cache stall can interrupt balance or arm return.

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

**Run 28** (`bal_20260703_222915.csv`, 23.1s): mechanical standup fastest
yet (arms home 1.5s, cascade at 1.46s) and balance held -- but the robot
breathed +/-1 deg all run and never truly stilled. Consequence: the calm
re-arm condition (|vel_err|<1.0 for 400ms) sits INSIDE the breathing band,
so the arms stayed locked in COOLDOWN the entire run. The tap at t=21.7
got ZERO arm response through vel +12 rad/s; the emergency throw (90% of
max cmd = 27 rad/s) fired at t=22.9 with roll already at 68. Rolled away.
- Fixes: calm re-arm re-banded to sit outside breathing, inside
  oscillation (vel<1.8 rad/s, rate<20 dps, 300ms; breathing is +/-1.5,
  the 0.56 Hz oscillator was +/-4.5); emergency throw at 60% of max cmd
  (18 rad/s -- above any oscillation's +/-8, fires ~0.5s earlier).
- Lesson 48: **a re-arm condition calibrated inside the plant's normal
  motion band disarms the helper permanently.** Band boundaries must be
  measured from the actual quiet-state statistics, not guessed.

**Run 29** (`bal_20260703_223630.csv`, 2.4s, standup runaway): the 2.5
rad/s arm return was dynamically infeasible. Arms home in 1.05s = the
physical equilibrium moved 3 deg in 1s while the robot's tilt never left
84 -- a velocity-mode PD chases a moving equilibrium with VELOCITY, not
the acceleration needed to tilt, so the gap never closed and the chase
ran away (+18 rad/s, faceplant). The base-ramp velocity gate worked
(base slowed to 1.2 deg/s) but is irrelevant: the ARMS set the real
equilibrium and kept marching. The ramp-phase velocity damper added
+1.7 deg of pro-cyclical chase fuel on top.
- Fixes: (1) return speed back to 1.5 rad/s (halving return time
  quadruples required acceleration); (2) **self-pacing arm return** --
  pauses while the robot lags (|base sp - tilt| > 1.5 deg or wheel vel
  > 2.5 rad/s), resumes when caught up: the equilibrium can no longer
  outrun the robot, structurally; (3) ramp-phase velocity damper clamped
  to braking only (may lower the setpoint, never raise it).
- Lesson 49: **gate the actuator that moves the physical equilibrium,
  not just the reference that follows it.** Slowing the setpoint while
  the arms marched was theater.

**Run 30** (`bal_20260703_224227.csv`, 9.5s): the self-pacing arm return
made things WORSE, exactly opposite its intent. The lag gate (gap>1.5 or
vel>2.5) paused the return 56% of the time, holding the robot in the
FRAGILE tip/mid-return stance for seconds (capture at 81.7 followed by a
violent fight: roll swung 72-85, cmd railed 22, drift ran +10 -- operator
help needed). The robot is hardest to balance exactly where the gate kept
parking it; getting the arms home QUICKLY (at a trackable rate) is safer
than pausing en route.
- Reverted: self-pacing lag gate and the ramp-phase p_term>0 clamp (both
  from the run-29 postmortem -- the correct fix from that crash was ONLY
  the return-speed revert to 1.5 rad/s, which stays). Crisis pause stays.
- Lesson 50: **the mid-return stance is the most fragile configuration --
  minimize time spent there, don't add mechanisms that extend it.** The
  run-29 runaway had one cause (2.5 rad/s return); the two extra
  "protections" added alongside the real fix were net harm. When a change
  breaks something, revert THE change, don't wrap it in compensators.

**Run 31** (`bal_20260703_224824.csv`, 4.9s): **THE STALL SOURCE, CAUGHT.**
prof_crsf_max = 31.4 SECONDS. CrsfReceiver::update()'s drain loop --
`while (_serial->available()) read()` -- is unbounded: during an RX byte
storm (RF noise / receiver garbage at 420kbaud) bytes arrive as fast as
they are drained and available() never goes false. This run froze Core 1
seven times (0.6s, 2.5s, 8.7s, 11.4s, 31s...); the robot fell during an
11s freeze mid-standup with the setpoint frozen and the dead-man clamping.
In hindsight this one loop explains EVERY mystery stall since the Speed-
mode rebuild began: 449ms (false abort, run 27), 740ms (dropped robot,
run 173619), the 2-8s engage weirdness, the profiler forever blaming the
crsf section.
- Fix: hard byte budget per update (1024 bytes ~ 3x legitimate per-tick
  traffic, worst case ~1-2ms). A byte storm now costs dropped radio
  frames, not the robot.
- Lesson 51: **every `while (peripheral has data)` loop is unbounded by
  contract.** Drain loops in control firmware need byte/time budgets --
  the peripheral, not the code, decides when an unbounded loop exits.

**Run 32** (`bal_20260703_231458.csv`, 51.2s -- first run on the CRSF byte
budget; stood up, needed a hand to stop the tow, then very stable through
7 taps, ran away on the recoil of the final big push):
- **ZERO control stalls during the run** -- no timestamp gaps in standup or
  balance. The 33 forensics events are all state:0 post-run: the `bal log`
  serial dump itself occupying the control task at idle (benign there; do
  not run `bal log` while balancing).
- **Standup**: capture 0.01 deg / arms returned together in 0.5s -- but
  towed +8.3 rad. Cause measured: tip eq 80.8 -> true fwd eq 85.3, a +4.5
  deg rise vs the curve's 1.9. The integrator carried +2.7 all run and the
  >8s persist blended it into stored trim, so the next standup starts
  pre-corrected. Curve shape left alone -- rise measurements now span
  +1.0..+4.5 across runs; the trim learner is the mechanism for that
  variance (lesson 27), the shape only needs to be roughly right.
- **Final runaway dissected**: backward push handled (full -0.30 arm
  throw, motion killed). The forward RECOIL got wheels-only: while the
  arms relaxed through the neutral band, one noisy sample dipped demand
  below 0.05 at the same tick |dev| crossed 0.10 -- ACTIVE exited to
  COOLDOWN mid-event, so the handoff never fired; re-arm needs 300ms of
  calm that a runaway never provides. Meanwhile sp_offset railed +8, vel
  hit 11, and the emergency throw missed by a hair: cmd held 13-15 rad/s
  for 1.5s, PEAKING at 17.3 vs the 18.0 (60%) trigger. Hand stop.
- Fixes: (1) ACTIVE -> COOLDOWN now also requires 150ms of accumulated
  calm -- a one-tick blip can no longer end the event while the recoil is
  still building; (2) emergency throw threshold 0.60 -> 0.45 (13.5 rad/s),
  still well above the +/-8 oscillation band.
- Lesson 52: **event-over must be judged by the robot's state, not the
  actuator's position.** The arms passing through neutral says nothing
  about whether the disturbance is finished -- only calm does.

**Run 33** (`bal_20260703_233710.csv`, 51.2s): the run-32 fixes VALIDATED
on tape -- the final big tap (t=38.3) was the textbook lifecycle: full arm
throw in 150ms, motion killed in 1.4s, smooth relax through neutral, wheels
absorbed the recoil (cmd peak 7.6, sp_offset never railed), settled in
3.5s. No cooldown trap, no runaway. Operator: "basically the ideal
recovery."
- **Standup tow smoking gun found** (+9.6 rad again, hand stop): the base
  setpoint (curve + learned trim) tracked the robot within 0.7 deg through
  the whole ramp -- but the RAMP-PHASE VELOCITY DAMPER contributed +4.6 of
  the +4.9 deg peak setpoint error. During standup the robot chases the
  rising equilibrium from BELOW: velocity is the robot keeping up, and
  "braking" it by raising the setpoint commands MORE velocity. Positive
  feedback, same signature both runs (spoff +4.5..+5.2 at the tow peak).
  Trim learning (0.62 -> 0.97 between runs) couldn't help because trim was
  never the driver.
- Fix 1: sp_offset clamped to +/-1.5 deg while the ramp is incomplete
  (full +/-8 after). Keeps ~3 rad/s of genuine surge damping, kills the
  spiral. Sim regression: standup matrix unchanged, pushes unchanged.
- Fix 2: "very slowly got back to 0" -- 9.6 rad at drift_kp=0.05 meant a
  0.48 rad/s return target, 33s to get home. DRIFT_VEL_KP 0.05 -> 0.08
  (still inside the fitted-model stable grid).
- Lesson 53: **a velocity damper assumes the velocity is the error.**
  During transitions the velocity IS the tracking -- damp it and you fight
  the maneuver. Phase-dependent authority (tight clamp during the ramp)
  separates the two regimes.

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


---

## September 13, 2026 — historical/media review and prepared firmware

Scope: reviewed all 112 CSVs (193,071 rows), five photos and sampled frames across seven videos. Baseline `e8b1280`; local branch `codex/balance-review-ready` in the root checkout. Austin confirms unchanged hardware/calibration but possible sensor mounting movement by a few degrees.

Findings: the largest final-July stand-up target surge occurs after ramp completion; the prepared 1.5-degree ramp clamp alone cannot address it. Seventeen logs are likely capacity-censored, not measured falls. The April 9 `220324` file does not substantiate its historical 30-second/98-percent description. July videos precede the final evening CRSF/controller changes. Angle tracking alone is insufficient evidence of station keeping. Full analysis, evidence links and corrected measurements: [September review](../docs/BALANCE_REVIEW_2026-09.md).

Changes: fast task owns IMU reads; each successful sample is integrated once, with stale/invalid and hard-deadman faults latched. Measured arm arrival and timeouts gate transitions. Wheel yaw mixing respects per-wheel speed limits. Persistent trim uses a recent calm estimate. Motor acceleration-limit verification now propagates failure. Serial input is bounded; tuning is between runs; web disarm is queued to the control task, and serial/web disarm requires RC switches low before rearming. Log operations require both motor groups disarmed. Telemetry retains v2 layout with an IMU-age extension and window diagnostic aggregation, adds end-to-end USB checksums/partial-write handling, and preserves raw successful/failed transfers on the host.

Defaults: retain inner PD 2.0/0.08, velocity PI and arm lifecycle. Restore unflashed drift return gain 0.08 to July's tested 0.05. Keep the already-prepared 1.5-degree ramp clamp. A simulation-only gradual post-ramp transition regressed early and push failures and was not ported. The refreshed approximate model's 72 cases show 3 early/11 later candidate failures versus 2/11 for July; 18 signed push cases had no failures for either. These are screening results, not evidence of a physical reliability improvement.

Verification: native C++ checks (100,000 randomized mixes, freshness/rollover, actual USB writer under partial writes/stall/disconnect), twelve Python tests including full 6,000-row transfer and corruption cases, syntax/whitespace checks, and successful PlatformIO 6.7.0 build. Build output and simulation evidence are retained under `evidence/balance-review/`. The exact source/binaries and rebuilt baseline rollback are identified in `artifacts/balance-candidate/manifest.json`.

Physical state: no upload, calibration reset, live robot operation, or remote Git push. Next: attach USB with both motor groups disarmed, back up device data/settings, flash firmware only, verify IMU/calibration/disarm on the floor, then capture one normal stand-up and 30-second untouched attempt. See [test guide](../docs/BALANCE_TESTING.md).


### Same-session continuation — authorized flash and device verification

Austin attached USB and authorized flashing, then connected the main battery. Saved an exact 8 MB flash backup and extracted settings plus a previously unreviewed 673-row, undated tip-up-only log. The partition table matched. Flashed the candidate application only, verified preserved calibration/trim and live IMU, and found the installed HWCDC library's zero-timeout unsigned-counter underflow path during serial review. Corrected the live timeout to 1 ms and kept active `bal status` out of LittleFS, rebuilt and reflashed.

Final flashed source: `51c8d49`; firmware SHA-256 `853267ab800a95f81ceb2c721df65f761ad1cd35018c592611f24232e4e71262`. OpenOCD verification passed. All six motors online; both groups disarmed; current CAN counters zero; calibration/0.97° trim preserved; fresh IMU age samples 2.091–8.421 ms with no latched fault. Final legacy USB transfer matched all 673 backed-up rows. Raw USB captures now bypass Git newline normalization. Full records: `evidence/balance-review/device/`; private device backup: `artifacts/device-backup-2026-09-13/`. No motor arming, physical balance attempt, calibration reset or remote push. Next: one filmed normal stand-up/30-second capture, then disarm both groups and retrieve v2 data.


### September 13 continuation — CH11 startup regression corrected

Austin reported that repeated CH11 presses did nothing. A live single-press capture confirmed the channel was received, followed by acceleration readback failures on both rear RS05 motors and a Speed-mode setup refusal before arm movement. Explicit disarmed reads returned current limit 10 A and acceleration zero from each rear motor. The manufacturer RS05 manual marks acceleration `0x7022` write-only; the September candidate incorrectly made its readback mandatory based on the shared RS00-oriented protocol assumptions.

Corrected MotorManager to check and retry failed acceleration transmissions without an unsupported readback; preserved mandatory current-limit readback and stop-on-failure behavior. Diagnostics now distinguish missing replies from actual mismatches. Added native tests using the production MotorManager with simulated RS05 responses: fail against prior source, pass after correction, including both rear motors, missing/zero acceleration responses, transmission failures and current-limit failures. Consolidated validation passes: both native suites, twelve Python tests, syntax/whitespace and firmware build (flash 1,134,401 bytes, static RAM 50,448 bytes). No balancing gains or calibration changed.

Workspace remains the root checkout on `codex/balance-review-ready`; no remote push. Evidence: `evidence/balance-start-diagnosis/`. Findings: `docs/BALANCE_START_FIX_2026-09.md`. Austin has reconnected USB and confirmed disarm. Next: package the correction separately, recheck live disarm, upload only the application, and verify startup before the next user-triggered trial.


Correction release: source `b5be4ef`, application SHA-256 `46e77224ad774204625c8878574f8581b40fe0547fb4cb6e53d1a7cabe6b9608`. Live preflight confirmed both motor groups disarmed. OpenOCD application-only programming at `0x10000` and verification passed; prior package and original flash backup retained. After boot, all six motors online with no errors, both groups disarmed, current CAN counters zero, calibrated arm deltas and 0.97° trim retained, fresh IMU age 7.093 ms with no fault. Evidence includes `flash.txt`, `preflash-status.serial`, `postflash-status.serial` and `postflash-checks.json`. No calibration reset, filesystem upload, tool-initiated motion or remote push. Awaiting one user-triggered test while recording.


First corrected-firmware physical trial: Austin operated the transmitter; both rear motors entered Speed mode, tip-up proceeded, arms returned and BALANCE was reached. Austin reports initial runaway followed by hand assistance and stable vertical balance. This is an assisted recovery, not an unaided stand-up success. The recorder reached its 120-second duration limit with 5,648 samples while balance continued. USB serial observations and the operator report are retained in `evidence/balance-start-diagnosis/`; exact intervention time is unmarked. No further tuning or flashing was done. Full v2 save/download is pending Austin supporting/disarming both groups; the pending question requests CH9/CH10 LOW and battery/USB left connected. All host serial readers have completed and released the port. Next: retrieve and validate the saved log, then analyze engagement and arm-return motion.


### September 13–14 continuation — USB recovery and slow tip-up diagnosis

After Austin disarmed, repaired the USB writer/host receiver in source `213c9bc`; flashed application-only SHA-256 `0a72f4d040df679f624a0a1b39fda73309f36aea6c8a7c08cf097d5e7d9cf12b` with verification. Preserved calibration and 1.09° learned trim. Retrieved all 5,648 rows of the assisted run with valid binary and transport checksums. The surge began during arm return, before ramp completion; operator intervention time is unmarked, so no unaided balance success is claimed. Retained failed and successful USB captures. No controller retune or autonomous motion.

September 14: Austin reported very slow movement and authorized flashing a correction. Downloaded the latest 20-row log with valid checksums: all TIP_UP, 14.849-second sample span, 15.276-second run ending in tip-up timeout / arm tracking. Frozen run profile isolates repeated delays up to 1,210,924 µs inside receiver update. Arm targets advanced only 0.014 rad per delayed tick; the 200 Hz IMU task stayed alive. Replaced per-byte UART reads with nonblocking blocks, added a 2 ms budget between blocks alongside the 1,024-byte cap, enlarged the RX buffer, rejected short CRC-valid control packets and exposed live timing diagnostics. Kept gains/calibration unchanged.

The production-parser tests cover traffic storms, partial/extended/malformed packets, all channels, failsafe freshness and rollover. Controlled driver regression: old code 1,228,800 µs, corrected code 320 µs. Consolidated validation passes all three native suites, 16 Python tests, syntax/whitespace checks and firmware build (1,135,289 flash bytes, 50,464 static RAM bytes). Evidence in `evidence/balance-slow-start/`; findings in `docs/BALANCE_TIMING_FIX_2026-09.md`. Next: freeze package, verify disarm, flash and measure actual receiver timing before declaring ready for a user-triggered test. No remote push.


September 14 release/result: flashed and verified application from source `5368df8290886c4b95052e68e5b40b2465bf1418`, SHA-256 `154d8fbec14d326eab58db723026148e0e134776d7631b22f645a74c2fa8d229`, application only. Before arming, receiver maximum was 486 µs. Austin started a physical test during observation, so the observer's disarmed-only assertion no longer applied; guarded status commands were refused during active balance. No tool initiated motion. Austin reports initial roll-away with hand assistance, then stable balance, a tap and deliberate motor disable.

Retrieved and verified all 1,405 rows: 28.109-second span, end `drive disarmed`, file CRC `0x77D64F12`, USB FNV `0x8115BEE4`. BALANCE began at 8.846 s and continued for 19.265 logged seconds. Actual run receiver max 531 µs, sample p99/max 20/25 ms, inner max 5.488 ms, zero recorded stalls or IMU freshness faults. Initial surge still develops during arm return with regular timing; peak initial command 10.84 rad/s, reported wheel velocity 7.48 rad/s. Contact/tap times unmarked. This confirms the timing fix, not unaided stand-up reliability. No balance tuning followed.

Final readback: IDLE, both groups disarmed, all six motors online, no motor errors/IMU fault/CAN TX failures; lifetime receiver max 564 µs. Existing trim learner stored 2.31°; calibration retained. Receive misses reached 12,517 and feedback age 145 ms, so receive pressure remains open. Existing CSV compilation timestamp is stale because the unchanged logger translation unit was reused; use package hash for identity. Corrected host analyzer's angle-offset units to degrees and checked Python tests/CLI output; firmware was not rebuilt or reflashed for that host-only reporting change. Findings, plot and release checks retained under `docs/BALANCE_TIMING_FIX_2026-09.md`, `evidence/balance-slow-start/`, and `artifacts/balance-timing-fix/`. No remote push; serial readers finished. Next: investigate initial target/arm-return transition and motor feedback quality before gain changes.


### September 14 — first stand-up reliability pass after timing repair

Austin requested a pass at initial roll-away and more reliable stand-up/balance. Found a definite throughput mismatch: approximately 1,450 balance requests/replies per second versus an 800/s receive cap, consistent with 12,517 drops during 19.265 seconds of balance. Prepared bounded 200 Hz draining and 50 Hz front-wheel holds, reducing expected traffic to about 850 requests/s. Corrected RS05 wheel units (50 rad/s, 5.5 Nm ranges) while converting velocity-dependent balance gains/thresholds to preserve existing response. Decoder now rejects short frames, consumes unknown traffic without ending the drain, and prevents acknowledgements/faults from overwriting or refreshing motion samples.

Screened several startup target/arm changes. The acceleration-limited return regressed a broader 108-case model comparison (26 early/17 late failures vs 25/10 baseline) and was removed. Final feedback-only/unit-conversion controller matches all baseline model outcomes and drift metrics. These approximate models omit contact and CAN losses; no physical improvement is claimed yet. Added production decoder/manager regression coverage, unit-invariance checks and new log feature flags. Four native suites, 16 Python tests, syntax/whitespace and firmware build pass (1,135,885 flash bytes, 50,720 static RAM bytes). Findings: `docs/BALANCE_STANDUP_PASS_2026-09.md`; evidence: `evidence/balance-standup-fix/`. Prior source still running and observed disarmed; next is fresh preflight, application-only upload and a user-triggered test. No remote push or tool-initiated motion.


### September 14 continuation — feedback release and failed physical stand-up

Flashed/verified `cad335c7d2c193f911ea903ee302d34fdc6cc36d`, application SHA-256 `f23b6faafba6cb615123955e0744ff73b28793b33b5c384ad4addea17339dcb3`, application only. Preflight and postflash both groups disarmed; all six motors online, retained calibration/2.31° trim, IMU healthy, CAN misses/TX failures zero before the attempt. Austin then ran a trial: stood, ran away far, hand stop, stable afterward. He attributes reboot to bumping USB. Host observer missed the actual attempt; retrieved all 1,428 saved rows afterward (binary checksum `0x1D0F7192`, transport `0x6A821A10`, end `arms disarmed`). Post-reboot profiler has live scope and does not establish prior-run receive losses.

The initial four-second peak wheel displacement doubled: 18.667 vs 9.187 rad; corrected average wheel-speed peak 26.322 vs 11.035 rad/s. Motion feedback age after the first 50 ms of BALANCE was at most 1 ms, with normal recorded control/sensor cadence and no recorded IMU/CAN TX faults. This does not establish unaided reliability. Three recent assisted runs show 3.7–4.5° capture-to-later-calm tilt shifts versus the scheduled 1.9°. Contact timing remains unknown; quiet supported capture must not be treated as proven free equilibrium.

Prepared a narrow bug correction: bound total captured trim before subtracting stored trim, replacing the relative ±6° clamp that biased the same measured pose based on the previous run. Example 78.1°/82.2°/+2.3112° previously reconstructed 78.5112°, now 78.1°. Native cases verify stored-trim independence and total limits. Log feature bit 8 distinguishes the behavior. No gain, arm-speed, pose, schedule or calibration changes. All four native suites, 16 Python tests and firmware build pass (flash 1,135,969/static RAM 50,720 bytes). Faster-return and larger-curve experiments retained for evidence but rejected: aggregate improvements included new failures and contradict prior physical evidence. No new upload yet; user asked to secure the sensor and prepare a stationary disarmed check because tilt readings shifted about 6°. Findings and comparison: `docs/BALANCE_HANDOFF_FOLLOWUP_2026-09.md`, `evidence/balance-handoff-followup/`. No remote push or tool-initiated motion.

Capture-bound correction release: Austin reported ready after the sensor-securing/flat-pose request; fresh status confirmed both groups disarmed. Flashed and verified source `a8aa9a123fdb3e197d56e3d9ba8d5b9a6cb0ace7`, application `068d043b64f957f577bae658446638e56c3c2b60ea523946bff4614a8d76a4c3`, application only. Postflash both groups disarmed, six motors online/no errors, 0.35° trim, stationary tilt −1.8°, healthy IMU (5.977 ms), receiver max 517 µs, CAN misses/TX failures zero. USB recorder active with note `capture-trim-secured-sensor`; one short operator-triggered trial requested with prompt support/disarm on roll-away. Physical benefit unverified. Original calibration storage untouched; exact reread deferred until posttrial after fixing a host status-command typo. No tool motion or remote push.

Capture-correction trial: Austin reports initial roll-away and intervention, then excellent balance. Retrieved all 1,119 rows over 22.363 seconds, checksum `0x3F3F58BA`/USB `0x28979A65`, features 15, end `drive disarmed`. This time USB captured the run and the profiler is frozen `balance_run`: receiver max 516 µs, inner logged max 5.009 ms, no stalls. CAN receive misses and TX failures stayed zero throughout the same boot. Early peak wheel speed ~19.8 rad/s, displacement ~15.24 wheel radians; unaided stand-up still not achieved. Both groups disarmed; exact calibration retained (center 1.768/−1.767, back 3.661/−3.670), trim learner stored 2.44°. Readers closed.

Austin steered the work toward detecting and aggressively stopping initial runaway instead of guessing a fixed balance angle. A 3.5° schedule/ramp-pacing candidate had been compiled but was NEVER flashed; its four-file source diff was removed. Retain its screening/build evidence as rejected work, not installed behavior. Current installed application remains `a8aa9a1`/`068d043b64f957f577bae658446638e56c3c2b60ea523946bff4614a8d76a4c3`. New direction: early speed/acceleration detection and bounded arm catch during return. Existing arm assistance, including emergency override, is inside the ramp-complete gate and cannot respond to these initial events. No new physical test while the response is being implemented and checked.


### September 14 — early wheel-feedback recovery candidate

Austin redirected development toward detecting incipient runaway and responding strongly while learning equilibrium. Discarded the unflashed 3.5° fixed-angle candidate. Early arm catch was also implemented/tested but rejected for model regressions; neither was deployed. Selected a one-shot fresh-feedback detector (1 rad/s + outward 2 rad/s², or 4 rad/s, confirmed 60 ms) during measured arm return. Starts the existing single velocity integral at Ki 1.0 for at most 800 ms/ramp completion, with 6°/s and ±6° bounds/anti-windup; preserves learned correction across ramp completion. Recovery targets zero wheel speed, then captures the settled hold position after 400 ms calm while preserving raw travel in logs. Normal inner gains, arm schedule/speed/calibration remain. New feature bit 16 and recovery phase flags retain the v2 layout.

Five native suites, 19 Python tests and firmware build pass (flash 1,138,013/static RAM 50,752 bytes). Production detector replay triggers at 0.720/0.760 s with only 0.142/0.216 rad travel in the two recent corrected-unit logs. Approximate 324-case model: early/late failures 86/60 versus installed 145/49; 67 rescues but 19 new failures. This is a diagnostic candidate, not proof of physical reliability. Findings and rejected experiments retained under `docs/BALANCE_STARTUP_RECOVERY_2026-09.md` and `evidence/balance-startup-recovery/`. Actual device still a8aa9a1 before release; next fresh disarm check, authorized application-only flash, stationary verification and short operator trial. No tool arming, calibration reset or remote push.


Early wheel-recovery release: source `720f2e31939a249a215b2b7f7c197130f2301310`, application `5531c6287dd5ef8d398d426a7596d8c6d91e6c914825d137bc9a1ef485734e35` flashed at `0x10000` only and verified. Fresh preflight/postflash both groups disarmed, all six motors online/no errors. IMU 5.974 ms, tilt −1.9°, no fault; receiver max 516 µs, CAN misses/TX failures zero. Center/back calibration deltas and 2.44° trim retained; forward coordinates zeroed normally on boot with unchanged raw arm pose. Observer running with note `early-wheel-recovery-v1`; one short operator-triggered test requested. Physical benefit pending. No tool motion, settings reset, filesystem upload or remote push.


### September 14 — first reported unaided success; preserve and push

Austin reported “That worked perfectly!!” after the requested early-recovery test, then explicitly requested documentation, commit and GitHub push. No firmware edits or further flash followed. Retrieved all 1,203 rows/24.173 s (15.350 s BALANCE), file `bal_20260914_233101_early-recovery-first-success.csv`, binary checksum `0x29CB8EA4`, USB `0xA2EED9A2`, end drive disarmed. The host recorder had expired before motion; the saved onboard run/profile is complete. First-four-second average speed 3.561 versus prior assisted 19.735 rad/s (82.0% lower), peak travel 4.186 versus 15.244 rad (72.5% lower). Detection 0.746 s; ramp completion 2.206 s retains the learned correction; calm/hold capture 5.681 s at 0.410 rad remaining displacement. Final displacement 0.197 rad. Arm assistance never active, no saturation/recovery-limit/IMU/CAN-TX fault rows, no stalls; run receiver maximum 537 µs.

Posttrial all six motors online/no errors, both groups disarmed, calibration retained, existing trim learner stored 3.16°, CAN receive misses/TX failures zero. Findings/figure/metrics/script/raw transfers preserved in `docs/BALANCE_STARTUP_RECOVERY_2026-09.md` and `evidence/balance-startup-recovery/`. Installed source `720f2e31939a249a215b2b7f7c197130f2301310`, application `5531c6287dd5ef8d398d426a7596d8c6d91e6c914825d137bc9a1ef485734e35`. This is one operator-reported success supported by telemetry, not a measured reliability rate. Prior five native/19 Python checks and build remain valid because firmware was unchanged. New analysis, checksum validation and visual inspection passed. All serial readers finished. Next: push `codex/balance-review-ready` as authorized; repeat this exact firmware before further tuning.


GitHub publication completed as authorized: pushed the full `codex/balance-review-ready` history, including source, prior fixes, evidence and successful-trial documentation. Initial published head `2868c14747ff4c726dad7c6d44e3874d3b045f82` matched a fresh remote ref read; branch tracking is configured. The tested firmware is still source `720f2e3` and remains installed without further edits. Main was not merged or rewritten. Exact local flash packages and private device backups remain outside Git; the tracked tested-firmware identity records the source/application hash. This follow-up records the completed push.


### September 19 — successful repeat reported; save awaiting arm disarm

Austin reports another very successful stand-up and offers the available telemetry. USB identifies the same board (`98:88:E0:0E:65:B0`). Read-only status: IDLE, drive disarmed, arms armed; logging off, 1,303 samples in RAM, pending save YES. Learned trim 3.51°, IMU healthy, all six motors online/no errors, CAN receive misses/TX failures zero. The disarmed-only helper correctly rejected this armed state. Requested CH9 and CH10 LOW with support and power/USB retained; a 60-second passive observation still showed arms armed. Both readers are now closed. No download attempted while the new run awaits saving, and no firmware, settings or motor actions were taken. Evidence under `evidence/balance-repeat-2026-09-19/`; comparison helper prepared for use after retrieval. Next: operator arm disarm, save confirmation, checksummed download and comparison.


September 19 save/retrieval completed: Austin confirmed both switches low. Read-only status verified drive/arms disarmed and pending save no; downloaded all 1,303 samples/26.044 s, including 17.220 s BALANCE (`bal_20260919_143953_early-recovery-repeat-success.csv`; binary `0xD98A7CF5`, USB `0xA13C1DF6`, end drive disarmed). Build timestamp and controller metadata match the first successful run, with learned starting trim 3.1650° versus 2.4380°. Recovery triggered 0.780 s, ramp complete 2.180 s, calm hold at 4.700 s (0.981 s sooner than first success). Peak initial average wheel speed 3.291 vs 3.561 rad/s, peak wheel travel 3.516 vs 4.186 rad (16.0% lower). No active arm assist, saturation, recovery-limit, IMU/TX fault or stalls. No contact markers; treat post-settle metrics as descriptive, not a controlled comparison. Two archived reported successes are not a population success rate.

Posttrial: both groups disarmed, six motors healthy, calibration retained, learned trim 3.51°, IMU healthy, CAN receive misses/TX failures zero. All readers closed. No firmware/settings/motion action; only evidence and documentation. Checksums/row structure, metadata comparison, analysis syntax and plot inspection passed; unchanged firmware checks were not redundantly rerun. Findings, raw transfers and reproducible comparison are retained under `evidence/balance-repeat-2026-09-19/`, linked from the main recovery findings and README. This evidence is being committed/published on the existing branch under the prior document/commit/push authorization.


September 19 publication verified: GitHub `codex/balance-review-ready` matched evidence commit `977b075f0cdc803347b5540b1f13716f18d04e39` after push. All four newly archived raw USB files match their on-disk bytes; their intentional CRLF endings are excluded from whitespace checks, which pass for the text/analysis/documentation changes. Robot firmware remains unchanged, disarmed and ready for later use.


### September 19 — settling recommendations, no firmware change

Austin asked what could settle sooner or more robustly. Read-only source/log review finds first forward stop at 2.525/2.420 s versus calm hold 5.681/4.700 s; reverse filtered speeds reach about −2 rad/s while the existing integral relaxes from 2.557/2.302° to about 1.70/1.73°. Recommend investigating a bounded faster unwind only during confirmed post-ramp recoil, retaining the initial catch and continuous integral. Preserve 400 ms calm confirmation: logged fields briefly satisfy calm criteria for roughly 244–260 ms before recoil.

An isolated, source-derived 324-case model screen changes only active-recovery unwind gain: current early/late failures 86/60; 1.5× 82/46 (3 new failures, 21 rescues); 2× 79/43 (2 new failures, 26 rescues). Changing-cohort median settling improves, but common surviving/settling cases have zero median paired improvement. This is mixed screening evidence, not proof of faster physical settling. Extra hysteresis/blending remains a proposal, not implemented. Findings and primary-source control principle are documented in `docs/BALANCE_SETTLING_REVIEW_2026-09-19.md`; scripts/results under `evidence/balance-settling-review-2026-09-19/`. Analysis syntax and unchanged production source verified. No device connection, firmware/settings edits, motion or flash; no commit/push this advice turn.


### September 19 — confirmed recoil-release candidate authorized

Austin requested implementing and uploading the settling experiment. Added confirmed opposing-motion release only during active recovery after measured ramp completion: 0.35 rad/s entry for 60 ms, 0.15 rad/s exit, 120 ms blend to 2× ordinary Ki (0.462). Extra release cannot create extra opposite-sign integral. Existing angle/rate/freshness limits, initial catch, arm motion, inner/stable gains, continuous integral and 400 ms calm hold remain. Feature bit 32 / sample flag 0x01 records the new phase without changing the 220-byte layout; old saved logs retain their version metadata.

Five native suites and 21 Python tests, syntax/whitespace and build passed. Production replay enables release at 2.630/2.560 s in the successful old traces; this tests phase selection, not predicted new motion. Final 324-case model: early/later failures 79/42 vs 86/60, 0 new failures, 25 rescues, identical pre-ramp outputs in all cases. Common surviving/settling cases have zero median paired time improvement. Physical benefit remains unverified. App SHA-256 `5d3465e0269f9df19065d1583de35d093b46f947fdd27e49ef0de0bbf28d4dd0`; flash usage 1,138,913 / static RAM 50,768 bytes. Findings/evidence in `docs/BALANCE_RECOIL_RELEASE_2026-09.md` and `evidence/balance-recoil-release/`. The previous advice analysis is included; its historical screen now explicitly loads frozen baseline source/config. No upload yet; next fresh disarm check, application-only flash, stationary health checks and user test.


Confirmed recoil-release upload: source `1d80257e5609a173d9bbe104997eab1f9d4c7341`, application SHA-256 `5d3465e0269f9df19065d1583de35d093b46f947fdd27e49ef0de0bbf28d4dd0`, programmed/verified at 0x10000 only. Fresh preflight confirmed both groups disarmed/no pending save and six healthy motors; prior successful log already archived. Postflash IDLE/disarmed, six motors online/no errors, calibrated center/back deltas and 3.51° trim retained, ordinary boot coordinate zeroing with matching raw poses. IMU 5.953 ms/no fault, tilt −1.9°, receiver max 446 µs, CAN misses/TX failures zero. Saved log size retained. New package and proven restoration package checksums verified. Raw serial captures remain byte-exact; human-readable build/flasher output has trailing whitespace trimmed. No tool arming, calibration reset or filesystem upload. Observer started with note `confirmed-recoil-release-v1`; awaiting user-triggered comparison. Release evidence/findings are being published on the existing authorized branch.


### September 19 — first recoil-release physical run, successful report

Austin ran the test during release documentation and reported “This worked great,” then that it felt faster and looked good. Passive observer captured the actual attempt and saved-log completion. Fresh status confirmed both groups disarmed and save complete; retrieved all 1,241 rows / 25.349 s including 15.980 s BALANCE. File `bal_20260919_151011_confirmed-recoil-release-success.csv`, checksum `0xCE65B6D9` / USB `0xB8588771`, features 63, build Sep 19 2026 15:04:50, note confirmed-recoil-release-v1, end drive disarmed. No contact markers supplied.

Compared with latest baseline: settled 4.600 vs 4.700 s (only 0.100 s sooner), reverse filtered peak 1.830 vs 1.984 rad/s (7.8% lower magnitude), rollback 2.063 vs 2.478 rad (16.7% less). Initial travel 3.848 vs 3.516 rad (+9.4%), speed 3.467 vs 3.291 (+5.3%); total absolute travel to settling only 1.7% lower. Equal five-second post-settle windows: speed RMS 0.0673 vs 0.0676 rad/s, travel range 0.023 vs 0.035 rad. Starting trim/battery differ; one trial does not establish reliability or causality.

Initial trigger/boost/ramp timings match latest baseline 0.780/1.560/2.180 s. New phase flagged 2.640–4.520 s; all 95 samples meet eligibility and inferred median learning multiplier is 2.000. No recorded arm assist, saturation/recovery limits, IMU/CAN-TX fault or stalls. Inner max 5.017 ms, BALANCE sample max 21 ms, IMU max 10 ms, rear feedback after 50 ms at most 6 ms; receiver max 525 µs, control gap 5.431 ms. Raw observer has stop retries after operator disarm while motors still moving; cause unmarked, later status healthy. Posttrial both groups disarmed, six motors online/no errors, calibration retained, trim learner 3.57°, IMU healthy, CAN misses/TX failures zero. All readers closed. Firmware remains source 1d80257 / SHA 5d3465e0…d4dd0. No new tuning or upload. Save exact candidate and repeat ordinary starts before further changes. Findings, plot, analysis script, raw transfers/config differences and checks are being published to the existing authorized branch.


### September 19 — standing drive requested and implemented

Austin explicitly authorized CH1 left/right and CH2 forward/back driving after standing, plus deployment. Added neutral/calm post-startup unlock, bounded velocity/turn slew, cruising feedforward inside the 200 Hz controller, velocity tracking through the existing single-integral cascade, new-position/heading hold after stopping, fresh-input/feedback gating and centered reacquisition after pauses. Conservative limits 1 rad/s average, 0.5 rad/s per-wheel turn, accel/decel 0.5/0.75 rad/s². Ground drive is suppressed while CH7 selects balance before tip-up. No tool arming or motion. Persistent trim qualifies only at rest; initial catch/recoil/inner/stationary gains and calibration remain.

Screening found that asking the integral to learn cruising speed worsened stops. Feedforward reduced new model failures; the 4× glide boost during driving worsened them and is suspended while moving/braking. Reduced final limits introduce zero new failures in the screened forward-stop-reverse and input-loss cases (54 each, only 36/31 actually command movement; some never unlock). All 324 neutral model trajectories exactly match installed source. The planar model does not validate turning, tire slip, braking distance or physical reliability; its quiet motion also differs materially from actual logs. Rejected variants were never flashed and are retained in evidence.

Six native suites and 23 Python tests, syntax/whitespace and firmware build pass. New 236-byte schema-3 samples append pilot intent/state to the unchanged 220-byte v2 prefix; 120-second/6,000-sample capacity retained with 1,416,000-byte PSRAM allocation. New host tests require pilot values only for new records. Two newly written telemetry fixtures initially lacked transport framing; corrected the fixtures without weakening integrity validation, then the full gate passed. Source/sample-layout tests and old-format reads are covered; full-length physical filesystem write remains untested. App SHA-256 `83d50ac277799094e58395cab3a9296e66c27718226da847a0442eea5a4c7c7d`, 1,141,584 bytes (flash used 1,141,221; static RAM 50,800). Findings under `docs/BALANCE_STANDING_DRIVE_2026-09.md`, evidence under `evidence/balance-standing-drive/`. Existing recoil-release image remains installed until fresh disarmed preflight and authorized application-only upload.


### September 19 — first standing-drive release/test; response adjustment requested

Flashed/verified source `90f07e83e3472cebbc5ca2646e585bfbce20f2d7`, app `83d50ac277799094e58395cab3a9296e66c27718226da847a0442eea5a4c7c7d`, application only after disarmed preflight. By postflash read Austin had armed both groups: disarm-helper correctly rejected that state, while raw status confirmed IDLE, six healthy motors, retained calibration/trim and healthy IMU/CAN. No tool arming/motion. Prior v2 log already archived; physical v2 compatibility download not completed before new run replaced it.

Austin reports all directions work, turning slow/responsive, forward/back very slow after a long delay; requests faster response. Saved full first-drive run `bal_20260919_155240_standing-drive-first-test.csv`, 2,881 samples/57.601 s, 48.780 s BALANCE, schema3/features127, binary `0x90FA4B79` / USB `0xED14696F`, end arms disarmed. Driving ready at4.880 s, no dropout before disarm. Full forward/reverse took6.420/6.840 s to exceed0.25 rad/s, then overshot to+3.169/−3.097 against±1 request. Turning differential followed command sign; RMS error0.090rad/s. Final2s average-speed RMS0.036. No saturation/IMU/CAN-TX fault rows; innermax5.032ms, IMUmax20ms, feedback<=2ms after first50ms. Posttrial both groups disarmed, six motors healthy, calibration retained, trim3.89°. Raw observer/transfer/metrics/plot preserved; no contact timestamps.

Root cause inference: low-error stationary P (0.462) × innerKp2 nearly cancels cruising velocity feedforward, leaving integral adjustment to initiate motion. New response-v2 uses moving/braking low-error P1.0, speed limit2rad/s, turn1.5, acceleration1.5/braking2/turnslew3rad/s². No stationary/stand-up gains, safety gates or calibration changes. Same schema3 adds feature128 (total255), preserving historical v1 export limits. Six native suites/23 Python tests, syntax/whitespace/build pass; app `b28793cdd8d2b45cdfbcb344cc594ba85ba79fadcdb2e373bca321a1d86b172b`,1,141,840bytes. All324 neutral model paths unchanged; selected short and long drive/input-loss screens add no new failures; rejected3rad/s cases added1–2 new input-loss failures. Model differs from observed quiet hold/dead zone and does not prove response improvement. Findings/evidence in `docs/BALANCE_DRIVE_RESPONSE_2026-09.md` and `evidence/balance-drive-response/`. Next freeze and authorized application-only flash after fresh disarm, then user test.


Response-v2 upload verified: source `d1ae97d5ef31b96a9f3f39a2765998f06b8d965b`, app `b28793cdd8d2b45cdfbcb344cc594ba85ba79fadcdb2e373bca321a1d86b172b`, application only0x10000. Both fresh preflight/postflash disarmed; IDLE, six healthy motors, retained calibration/3.89° trim. IMU1.102ms/no fault, receiver max495µs, CAN misses/TX failures0. Normal boot coordinate zeroing preserves raw poses/deltas. Downloaded existing v1 log through v2: all2,881 rows and original config/limits/features127/checksums identical. `bal_20260919_160318_v1-export-on-drive-response-v2` is the same first drive, not another trial. Exact packages verified and retained. New feature source/evidence on existing authorized branch; no tool arming/motion/settings reset/filesystem update. Recorder with note standing-drive-response-v2 started, user invited to test brief small inputs and report delay/stopping. Physical v2 behavior still pending.


### September 19 — delayed full-stick driving; acceleration control and planned arms

Austin requested 10× forward/back and 3× turn limits, emphasized that the delay occurs at full stick, and proposed deliberate arm movements to initiate lean and assist acceleration/braking. Reattached USB allowed complete retrieval of response-v2: 1,666 rows/33.304 seconds,24.480 seconds BALANCE, binary0x298924B0/USB0x72A5CE52. Full forward reached half of its2rad/s limit after3.06 seconds and90% after4.32 seconds. No logged saturation, setpoint clamp, IMU or CAN-TX fault; final2s velocity RMS0.0326rad/s. Thus literal actuator saturation is unsupported; the v2 stronger-P inference did not fully explain/fix the delay. Live USB-only counters after reboot show link0/motorRX0 and TX errors, not the moving trial's bus health. Both groups disarmed, trim3.33°, calibration retained. Prior host observer expired in idle; the onboard saved run is authoritative.

Prepared a driving-only200Hz acceleration state-feedback controller (angle8.8,rate3.1,speed3.0; travel error clipped8rad/s, total balance acceleration100rad/s²), initialized from prior command and bounded at wheel limits. It responds immediately to requested speed without waiting for equilibrium learning. Driving P-to-angle removed; ordinary equilibrium I gated to speed errors below1rad/s. Calm400ms stop gate followed by bounded30rad/s² transition back to stationaryPD. CH2limit20rad/s,CH1limit4.5rad/s, reference accel12/brake16/turnslew18. Corrected actual yaw selection so stationary1.5rad/s heading clamp no longer truncates intended4.5 turns. Balance retains priority under30rad/s wheel cap.

Planned arms use negative requested acceleration, gain0.008333333, bounded±.10 center fraction (~10°),80ms smoothing. Active/recoil catches take priority. Speed between zero and intended velocity is expected acceleration; overspeed/wrong direction/braking lag still drives recovery arms. Measured pose still schedules equilibrium; calibration/arm limits unchanged. Schema4 appends pilot_arm to exact236-byte prefix (240bytes,1,440,000PSRAM),features511 andpilot_flags16 identify acceleration/handoff. Older logs retain metadata and blank unknown fields; v2 rollback requires downloading new schema4 logs first.

Seven native executables,25 Python tests, syntax/whitespace and build pass. Flash1,143,661/staticRAM50,832 bytes; image1,144,032 bytes,SHA75ccdb4412dc3ebd0e4cc567cde845ab6bb3107d7d105e7fdcf69f4b2afa8118. Final source-hashed model screen invokes actual C++ helpers:324 neutral trajectories exactly match installed; three54-case drive/input-loss profiles add zero failures, with only34/38/31 cases unlocking driving. Coarse model's quiet ripple, omitted yaw/traction/contact/current and full arm reaction prevent physical reliability claims. Earlier reference-governor/damping/small-balance-acceleration candidates rejected for falls or slow braking, never flashed. Findings/evidence: docs/BALANCE_DRIVE_AGILITY_2026-09.md andevidence/balance-drive-agility/. No candidate deployment yet; next freeze/package, freshdisarm/logpreflight, authorized app-only upload and powered stationary verification before operator trial.


Acceleration/arm-assist v3 upload verified: source5a07ebabef8f6431e8a9908fa4b53170b24a2308, app75ccdb4412dc3ebd0e4cc567cde845ab6bb3107d7d105e7fdcf69f4b2afa8118, app-only0x10000. User powered battery/transmitter while disarmed; a transient USB disappearance was followed by a fresh successful disarm preflight before programming. Powered postflash: link1, six motors online/error0,25.2V, both groups disarmed, calibration/3.33°trim retained, IMU5.997ms/no fault, CANmiss/TXfail/buserror0. Live limits20/4.5 andschema4 confirmed. Boot allocation message not captured; verify buffer during first new run. Historicalv2 export via new firmware matches all1,666 original fields and config/binarychecksum0x298924B0; pilot_arm blank, changed transport0xA5D84DD1. This is a compatibility copy, not a new physical trial. Frozen package and v2 restore package checksum-verified. No tool arming/motion, settings reset or filesystem update. Next operator small-input test, center between, disarm both groups and download. Release/evidence published on the existing authorized branch after this record.


### September 19 — v3 physical trials expose startup variation and driving oscillation

Before a passive observer was started, Austin ran two tests. Downloaded the first before the second overwrote storage. First:561rows/11.204s,2.380sBALANCE,binary0x50545AF6/transport0x0A0FFD86,endbailout_angle_error. Pilot never ready/moving, acceleration bit16 absent, planned arms0; raw commands match originalPD within0.00264rad/s given rounded telemetry. Runaway arose in arm return: recovery0.68s, crisis pause1.26s near.52tipfraction, correctionrailed6°,filtered speed28.53rad/s/travel26.60rad. Reverse stick appeared1.60s after runaway and was gated out. No IMU/CAN-TX fault. No firmware change followed.

Second on identical5a07eba image:2,381rows/47.604s,38.780sBALANCE,binary0x64EC16B9/transport0x58692019. Austin reports successful stand-up, all front/back rocking during driving, eventual wall contact. Ready4.10s; predrive tiltspan.101°/speedRMS.065rad/s; firstrequest6.74s->actual+.5rad/s7.04s. Turning tracks4.5rad/s. Braking rings around4–5Hz: firststop measured+11.83to−4.75rad/s,tiltspan5.27°. Planned arms bounded.10, but ordinary/emergency recovery reaches+.45/−.30. Driving-only emergency discrimination needs correction: inherited13.5rad/s command threshold is below new20rad/s speedlimit and fires on highspeed braking errors. Inputpause9.50/25.38s coincidesbodyrate>30°/s whileRCfresh. No recordedIMU/CAN-TXfault or preterminalsaturation. Exactwalltimeunmarked; analysisexcludesfinalbout>=32s. No physicalmodelprecisionclaim: finalscreenmissedfastrealoscillation.

Capturedtilt82.112°failedvs85.329°successful,withinitialrawaccel−2.012/−1.913°,so afixedsensoroffsetisnot established. Startuprobustnessremainsunresolved; newdrivingcontrollerwasinactiveinfailedstart. Both buffers/transferchecksumsverified, bothgroupsdisarmed, samefirmwareretained, passiveobservernotstarted/note=none. Findings/plots/reproducibleanalysis:docs/BALANCE_DRIVE_TRIALS_2026-09-19.md,evidence/balance-drive-agility/. Proposednext: retunedampeddrivefeedbackusingrealringing,easeramps(e.g.6/8forscreeningonly),gradedbrakingarmsandheadroom-awarefullemengencycriterionduringdriving; retain20/4.5limits andtrueemergencycatch. No new code or flash during this analysis. Avoid another fullspeedtrial before correction.


### September 19 — driving-only fork: damping and graded arm recovery

Austin requested a new focused thread to implement the driving recommendations, test in simulation, flash, then stop for operator testing. Preserved original stand-up/stationary control. Observed v3 4.33–4.67Hz ratios show raw body-rate/wheel magnitude12.95–14.57, motor response gain1.13–1.25 and original filtered-rate lag50–54degrees. The prior weak-coupling/first-order model could not represent those effects. Added optional uncertain second-order motor and delay paths plus already-standing initialization; these ratios are closed-loop observations, not identified physical parameters.

Selected separate6ms fresh-IMU driving filter/rate gain1.5 (was3.1), angle8.8/speed3 unchanged, requested accel/brake6/8 (was12/16), speed20/turn4.5 unchanged, ordinary arms30% of prior demand capped.12, full driving catch only on speed disturbance plus near30rad/s rail and confirmed2deg/8deg-per-second outward lean60ms same direction, or immediate8deg/20deg-per-second severe outward lean. Legacy stationary arm logic, true full catch range, planned assistance/priority, neutral/freshness/disarm gates remain. Schema4/240bytes/features1023 adds versioned metadata; actual driving fast rate is not separately sampled.

Final screen uses frozen5a07eba production helpers versus current actual C++ helpers:324 neutral traces exactly identical; six72-case profiles (small/full/reverse/input-loss/turn-entry/impulses), candidate0falls vs24baseline,0newfalls;69/72 per profile actually unlock,414 commanded cases. Fast spectral power lower, but quiet-hold ripple differs from hardware and yaw/traction/contact/arm reaction are absent. Nominal20rad/s stop remains long (~7.32s first below.3,50.6wheel-rad over9s); stronger speed/braking alternatives shorten it with more body motion/input pauses and were not selected. No claim of physical reliability or braking distance.

Seven native executables/25Python tests, syntax/whitespace/build pass. Updated ramp-duration test after its old3s assumption correctly failed; retained ramp/stop/bounds assertions. Fixed a Python/C++ rounding artifact in unchanged-PD model pass-through before exact neutral comparison. App90a3df4d16276860059caacc059959ea6d774d2dc8528b68353aee90043e757c,1,145,552bytes, flash1,145,193/staticRAM50,848. Findings/evidence under docs/BALANCE_DRIVE_DAMPING_2026-09.md and evidence/balance-drive-damping/.

Initial read-only USB preflight: IDLE/bothgroupsdisarmed/no pending save, trim3.33degrees retained, motor traffic stopped/stale CAN faults; requested battery power for postflash checks. Downloaded existing v3 log again before programming:bal_20260919_192941_v3-preserved-before-drive-damping matches all2,381rows AND metadata of185559, binary0x64EC16B9/USB0x58692019. No new physical trial, motion or settings changes. Next exact source freeze/package and application-only flash, stationary checks, then stop without trial observer.

Driving damping v4 uploaded and readback-verified at0x10000 only: sourcea2774d4999b1c0f9ce707a9377aecd2a3661a3fe/app90a3df4d16276860059caacc059959ea6d774d2dc8528b68353aee90043e757c. Exact new/v3 packages verified; original8MBbackup hash rechecked. Postflash live v4constantsmatch, IDLE/bothgroupsdisarmed, IMU6.046ms/no fault, RC linked/max488us, trim3.33 and center/back deltas retained, saved log571828bytes remains. Powered motor check incomplete: RX0 since boot, all motor readings defaults and CAN errors increasing; battery confirmation unanswered. This lack of motor traffic predates upload. Forward refs0 are transient software coordinates before operator arming, not a persistent calibration loss. No tool arming/motion, filesystem/settings writes, observer or physical trial. Stopped after upload/available stationary checks; follow-up can verify powered motors and later retrieve Austin's physical test. Source published on existing authorized GitHub branch; final release record follows.


### September 19 — Wi-Fi/OTA release and untethered test workflow

Installed combined firmware source `6018cd4` with driving damping v4 control math, calibration and trim retained. Application SHA-256 `3bd85bfdd26f48df86c5d87b76797282f4c2479a7e96b2838217f479553171cf`, 1,187,632 bytes; frozen private package `artifacts/wifi-ota/release/`. Core-0 networking publishes bounded live snapshots and uses control-owned disarmed maintenance for association, saved-file export and inactive-slot application OTA. The dashboard is embedded in the application; do not upload LittleFS. USB remains necessary for gain tuning, calibration, console notes and detailed diagnostics. Radio remains the motion-control path.

Motor-power-off bench checks verified update/reboot, image/checksum/authentication rejection, interrupted upload, reconnect and concurrent telemetry traffic without new 200 Hz intervals above 7.5 ms in the final runs. The saved pre-upgrade v3 trial (schema 4, 2,381 samples) exported wirelessly with matching original rows; it is a compatibility copy, not a new physical test. RC-link quality was excluded from final network tests at Austin's request. No powered balance/drive validation of the combined release has occurred. See [release evidence](../evidence/wifi-ota/README.md) for measurements and limits.

Documentation now defaults to `scripts/robot_wifi.py log` for validated `.csv`/`.wire` archives and `scripts/robot_wifi.py ota` for updates. Analyze the downloaded CSV separately; use CH12 for untethered markers and keep local operator notes. Only the latest run is stored, so retrieve it after every disarmed test. [Operating guide](../docs/WIFI_OTA.md) and [test procedure](../docs/BALANCE_TESTING.md) supersede historical upload/download instructions above. This documentation pass did not change firmware or device state. Next: operator stationary motor-feedback check while disarmed, then supervised RC-ready physical trials.
