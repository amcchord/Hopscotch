# Balance Mode Tuning History

## Current Architecture (v3: Arm-Scheduled Setpoint + Command Integrator)

### Core 0 (200Hz) - Balance PD
- Complementary filter: alpha=0.996 (angle), gyro filter alpha=0.08 (~60ms time constant)
- IMU axes: gyro.x for roll rate, atan2(accel_y, accel_z) for accel angle
- PD: `motor_vel = Kp * (effective_sp - tilt) - Kd * gyro_rate`
- **Kp = 2.0, Kd = 0.08**
- Max drive speed: 25 rad/s
- Back wheels only (front wheels held at engage position)

### Core 1 (50Hz) - State Machine + Setpoint Management
- **Arm-scheduled base setpoint**: linearly interpolates between ARMS_TIP (86.5) and ARMS_FWD (92.0) based on current arm position
- **Command integrator trim**: integrates filtered PD motor command to handle surface/battery/wall offsets
  - Nonlinear filter: alpha 0.01 (small cmds) to 0.10 (large cmds)
  - Gain: 0.5 deg/(rad/s)/s, rate max: 2.0 deg/s, trim clamp: +/-5 deg
  - Sign: `trim -= vel_gain * filtered_cmd * dt` (positive cmd = sp too high = lower trim)
- **Effective setpoint = scheduled_base + trim**
- Safety: tilt range [30,150], sustained error >35 for 2s, rate >200 dps for 500ms, saturation >3s

### Arm Tip-Up
- Tip speed: 0.7 rad/s (distance-based deceleration per arm)
- Tip offsets: left=2.61, right=1.89 rad from forward ref
- Engage after: arms done AND |tip_expected - roll| < 15 AND |rate| < 50
- **Arm return: 10 rad/s** (fast snap-back, ~0.25s)
- Arms begin returning 1s after balance engage

### Controls
- Ch7 high + single Ch11: Normal tip-up sequence
- Ch7 high + double-tap Ch11 (within 500ms): Force-engage at current position
- `bal engage`: Force-engage via serial
- `bal kp/kd <val>`: Tune PD gains
- `bal vgain <val>`: Tune command integrator gain
- `bal pkp/pki/pkd <val>`: Tune position return (currently disabled)
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

### Phase 7: Command Integrator (current approach)
- Changed from integrating wheel velocity to integrating **PD motor command**
- Motor command has signal even when wheels are blocked (wall escape for free)
- Removed all stuck/wall detection -- command integrator handles it naturally
- **Problem**: integrator gain tuning -- too high causes positive feedback runaway, too low fails to track arm return

### Phase 8: Arm-Scheduled Setpoint + Command Integrator Trim (current)
- **Breakthrough**: separate the known arm-position-to-balance-point mapping from the unknown trim
- Scheduled base: linear interpolation from 86.5 (arms at tip) to 92.0 (arms at forward ref)
- Command integrator is now just a trim (+/-5 deg) on top of the scheduled base
- Arms snap back at 10 rad/s, setpoint tracks in lockstep via scheduling
- **Best run: 48.9s balance, converged at 92.5 deg, 0.21 avg error** (telemetry: bal_20260409_231344.csv)
  - Kp=1.5, Kd=0.08 at the time
  - But needed hand-bouncing to find balance initially
- Kp increased to 2.0 for more aggressive correction
- Scheduled base refined to 92.0 based on telemetry (avg roll during stable = 91.99)

**Latest issue (current)**: integrator trim sign was wrong -- positive motor cmd was raising trim instead of lowering it, causing slow runaway. Fixed by flipping sign to `trim -= gain * cmd * dt`.

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
