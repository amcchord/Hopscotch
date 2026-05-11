# Balance Mode Tuning History

## Current Architecture (v10: Position PI with Leaky Origin)

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
