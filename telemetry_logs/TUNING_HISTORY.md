# Balance Mode Tuning History

## Current Architecture (200Hz Dual-Core + Feedforward)

### Core 0 (200Hz) - Balance PD
- Complementary filter: alpha=0.996 (angle), gyro filter alpha=0.08 (~60ms time constant)
- IMU axes: gyro.x for roll rate, atan2(accel_y, accel_z) for accel angle
- PD: `motor_vel = Kp * (effective_sp - tilt) - Kd * gyro_rate + velocity_bias`
- Kp = 1.0, Kd = 0.12
- Max drive speed: 25 rad/s
- Back wheels only (front wheels held at engage position)

### Core 1 (50Hz) - State Machine + Outer Loops
- Feedforward setpoint: `ff_sp = BASE_DEG(70) + FF_GAIN(7.0) * avg_arm_delta`
- Adaptive fine-tuning: adapt_rate=0.05 on top of feedforward
- Position return: velocity bias PID (pkp=1.0, pki=0.3, pkd=0.2)
- Arm control, CRSF, display, serial commands

### Arm Tip-Up
- Tip speed: 0.7 rad/s (distance-based deceleration per arm)
- Tip offsets: left=2.61, right=1.89 rad from forward ref
- Engage after: arms done AND |ff_sp - roll| < 15 AND |rate| < 50
- Arm return: 0.5 rad/s constant speed, starts 1s after PID engage

### Controls
- Ch7 high + single Ch11: Normal tip-up sequence
- Ch7 high + double-tap Ch11 (within 500ms): Force-engage at current position (skips tip-up)
- `bal engage`: Force-engage via serial
- `bal ffgain <val>`: Tune feedforward gain live
- `bal base <val>`: Tune base balance angle live
- `bal kp/kd/adapt <val>`: Tune PD and adaptation
- `bal pkp/pki/pkd <val>`: Tune position return PID
- `d`: Toggle periodic debug output

---

## Key Findings (Chronological)

### Phase 1: Initial Implementation (50Hz, Cascaded PID)
- Cascaded PID (outer angle → inner rate) collapsed to effective Kp=0.08, Kd=0.1
- Heavily overdamped angle with coupled gains
- LittleFS telemetry writes blocked control loop for 6+ seconds → PSRAM buffer fix
- Best run: 21.3s balance, 0% saturation with single PD + adaptive setpoint

### Phase 2: Single PD+I Controller (50Hz)
- Replaced cascaded PID with direct `motor_vel = Kp * err - Kd * rate + Ki * integral`
- Kp and Kd independently tunable (was the key insight)
- Adaptive setpoint (cumulative error tracking) finds the true balance point
- Position return PID as velocity bias (not setpoint shift) for drift correction
- **Best pre-200Hz run**: 21.3s, 0% sat, roll 89-94, drift 1.7 rad, adaptive sp converged to 91

### Phase 3: 200Hz Dual-Core Architecture
- FreeRTOS task on Core 0 for 200Hz PD balance loop
- Complementary filter replaces Madgwick for balance angle (near-zero latency)
- Direct gyro rate (gyro.x) instead of differentiated Madgwick roll
- IMU read at 200Hz on Core 1, shared to Core 0 via volatile struct + spinlock

#### Critical Finding: Gyro Filter Alpha
- **alpha=0.08 (60ms time constant): ALWAYS STABLE** - multiple 21s runs, 0% saturation
- alpha=0.10-0.15: oscillation grows within 3-5 seconds
- alpha=0.20: oscillation grows within 2-3 seconds
- alpha=0.30-0.50: violent oscillation from first tick
- The heavy filter is essential because the direct gyro has more high-frequency content than the old differentiated+filtered Madgwick signal

#### The Arm Return Problem
- Arms at tip position: balance point ~86-87 degrees
- Arms at forward ref: balance point ~70 degrees
- **16-degree shift** during arm return -- too large for any reactive controller
- Adaptive setpoint too fast → destabilizes PD
- Adaptive setpoint too slow → robot drifts off before it catches up
- Every failure mode traced back to the arm return shifting the balance point

### Phase 4: Feedforward Compensation (Current)
- Proactively compute setpoint from arm position instead of reactive discovery
- `ff_setpoint = 70.0 + 7.0 * avg_arm_delta_from_forward_ref`
- Arms at tip (delta ~2.3): ff_sp = 86.1 (matches measured balance)
- Arms at forward (delta 0): ff_sp = 70.0 (matches measured balance)
- Setpoint tracks arm position in real-time -- no lag, no oscillation risk
- Gentle adaptive fine-tuning (0.05 rate) handles surface/calibration differences
- Velocity bias position PID handles drift correction

---

### Phase 5: Correct Base Balance Point (Current)
- Force-engage test (double-tap Ch11) with arms at forward ref: **balance point = 87.2 degrees**
- Previous assumption of 70 degrees was WRONG -- that was the robot slowly falling forward
- Base setpoint set to 87.0, feedforward gain set to 0 (arms don't shift balance point)
- **Best 200Hz run: 6 seconds of perfect ±0.2 degree balance at 87.0, 0% saturation, zero drift**
- Then arm return torque tips robot forward → drives into wall → can't recover
- Remaining issues:
  - Arm return creates dynamic torque that tips robot forward (not a static balance shift)
  - Velocity bias pushes wheels backward at wall → tips body MORE forward (wrong)
  - Need: (a) gentler arm return to reduce disturbance, (b) wall escape via setpoint shift

## Saved Telemetry Files

### latest_run.csv
Pre-200Hz best run: 21.3s balance, 0% saturation, adaptive setpoint found 91.0

### 200hz_balanced_16s.csv
Early 200Hz run with gentle outer loops: 16.3s, 0% sat, first 12s had drift under 1 rad

### 200hz_feedforward_test.csv
Feedforward test with ff_gain=0, base=86.1: confirmed balance point shifts (robot tipped to 102)

---

## Lessons Learned

1. **Gyro filter alpha=0.08 is non-negotiable at 200Hz.** Every lighter filter oscillated.

2. **The arm return shifts balance by 16 degrees.** This is the single biggest challenge. Reactive controllers (adaptive setpoint, PID) can't track it fast enough. Feedforward from arm position is the answer.

3. **Position control must be velocity bias, not setpoint shift.** Shifting the balance setpoint based on wheel drift makes the robot lean in the drift direction (wrong). Adding a velocity bias to the motor output drives the wheels back without changing the balance angle.

4. **The 200Hz PD is rock-solid when the setpoint is correct.** Multiple runs confirmed ±0.3 degree balance with 0% saturation for the first 1-4 seconds of every run. The failures were always from outer loop interference or the arm return.

5. **PSRAM telemetry buffering is essential.** LittleFS writes at 50Hz blocked the control loop for seconds. Buffer in PSRAM, write to flash only after session ends.

6. **Complementary filter axes for M5Stack AtomS3R:** gyro.x = roll rate, atan2(accel_y, accel_z) = accel angle.

7. **The true balance point is ~87 degrees regardless of arm position.** Force-engage test with arms at 0,0 confirmed 87.2 degrees. The earlier observation of roll=70 was the robot slowly falling forward, not the equilibrium. Arms barely affect the balance point.

8. **Double-tap Ch11 for force-engage** allows testing balance at any arm position without the tip-up sequence.
