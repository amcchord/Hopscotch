# Balance Mode Tuning History

## Current Working Configuration (latest_run.csv)

**Results: 21.3s balance, 0% saturation, roll 89-94 deg, max drift 1.7 rad**

### Balance PD Controller
- Kp = 1.0 (linear, no expo)
- Kd = 0.15
- Filtered roll rate: alpha=0.25 (in main.cpp)
- Max drive speed: 25 rad/s

### Adaptive Setpoint
- Base setpoint: 83.0 degrees
- Adapt rate: 0.5
- Starts at current roll on engagement (zero initial error)
- Freeze conditions: |error| < 12 deg AND |rate| < 30 deg/s
- Clamp: base ± 8 degrees (75-91)
- Converges to ~91.0 in this run

### Position Return PID (lean correction in degrees)
- Pos Kp = 0.3 (deg lean per rad of wheel drift)
- Pos Ki = 0.2 (deg lean per rad*sec accumulated drift)
- Pos Kd = 0.05 (deg lean per rad/s wheel velocity)
- Integral max: 100
- Sign: positive drift → positive correction → lean backward → drive back

### Arm Tip-Up
- Tip speed: 0.7 rad/s (distance-based deceleration per arm)
- Tip offsets: left=2.61, right=1.89 rad from forward ref
- Engage after: arms done AND |roll-setpoint| < 10 AND |rate| < 50
- Arm return: 0.5 rad/s, starts 1s after PID engage

### Key Learnings

1. **Cascaded PID was wrong**: The inner/outer loop collapsed to effective Kp=0.08, Kd=0.1 -- heavily overdamped angle with coupled gains. Single PD+adaptive setpoint is far better.

2. **Roll rate must be filtered**: Raw numerical derivative of Madgwick roll is too noisy. Alpha=0.25 low-pass filter is critical for Kd to damp instead of excite oscillation.

3. **Adaptive setpoint finds balance point**: The true balance point varies (82-91 deg depending on arm position, surface, etc). The adaptive setpoint tracks cumulative error to find it automatically.

4. **Position PID sign matters**: Positive wheel drift needs to INCREASE setpoint (lean backward) to drive back. The opposite sign was wrong for months.

5. **Expo curve caused issues**: Amplified engagement transients. Linear Kp with proper gains works better.

6. **Arms must complete tip-up before PID**: Otherwise arms fight the PID.

7. **PSRAM buffering for telemetry**: LittleFS writes at 50Hz blocked the control loop for seconds. PSRAM buffer solved this.

8. **Balance point ~83-91 deg, not 90**: Depends on arm position. Adaptive setpoint handles this.

## What Still Needs Work

- Robot still drifts and gets stuck against walls (position PID working but slow)
- Balance duration limited by running out of test space
- Needs testing on larger surface
- Could benefit from expo curve once base is solid (gentle near balance, aggressive for recovery)
- Consider using raw gyro instead of differentiated Madgwick for roll rate
