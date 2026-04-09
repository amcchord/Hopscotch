#include "balance_controller.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <cmath>
#include <esp_heap_caps.h>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

float BalanceController::moveToward(float current, float target, float rate, float dt) {
    float diff = target - current;
    float step = rate * dt;
    if (fabsf(diff) <= step) {
        return target;
    }
    if (diff > 0.0f) {
        return current + step;
    }
    return current - step;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void BalanceController::begin(MotorManager* motors, ArmController* arms) {
    _motors = motors;
    _arms   = arms;
    _state  = BalanceState::Idle;
    _targets_initialized = false;
    _filter_initialized = false;
    _logging = false;
    _log_count = 0;
    _log_saved = false;

    if (!_log_buf) {
        _log_buf = (BalanceSample*)heap_caps_malloc(
            BALANCE_LOG_MAX_SAMPLES * sizeof(BalanceSample), MALLOC_CAP_SPIRAM);
        if (_log_buf) {
            Serial.printf("[Balance] Log buffer allocated in PSRAM (%d bytes)\n",
                          BALANCE_LOG_MAX_SAMPLES * (int)sizeof(BalanceSample));
        } else {
            Serial.println("[Balance] WARNING: PSRAM alloc failed, trying regular heap");
            _log_buf = (BalanceSample*)malloc(BALANCE_LOG_MAX_SAMPLES * sizeof(BalanceSample));
            if (_log_buf) {
                Serial.printf("[Balance] Log buffer allocated in heap (%d bytes)\n",
                              BALANCE_LOG_MAX_SAMPLES * (int)sizeof(BalanceSample));
            } else {
                Serial.println("[Balance] WARNING: could not allocate log buffer");
            }
        }
    }

    _kp = BALANCE_KP;
    _kd = BALANCE_KD;
}

// ---------------------------------------------------------------------------
// Complementary filter + PD balance (Core 0, 200Hz)
// ---------------------------------------------------------------------------

void BalanceController::balanceTick(const RawImuData& imu, float dt) {
    // Complementary filter: fuse accel (absolute but noisy) with gyro (fast but drifts)
    // Roll = rotation around X axis: accel angle from Y and Z gravity components
    float accel_angle = atan2f(imu.accel_y, imu.accel_z) * 57.2957795f;

    // Gyro rate: direct X-axis measurement with light smoothing
    float gyro_raw = imu.gyro_x;

    if (!_filter_initialized) {
        _tilt_angle = accel_angle;
        _gyro_rate = gyro_raw;
        _filter_initialized = true;
    } else {
        _tilt_angle = COMPLEMENTARY_ALPHA * (_tilt_angle + gyro_raw * dt)
                    + (1.0f - COMPLEMENTARY_ALPHA) * accel_angle;
        // Low-pass on gyro rate: alpha=0.08 at 200Hz = ~60ms time constant
        // This exact setting held balance for 20+ seconds in testing
        _gyro_rate = 0.08f * gyro_raw + 0.92f * _gyro_rate;
    }

    // Only run PD when actively balancing
    if (_state != BalanceState::Balancing) return;
    if (!_targets_initialized) return;

    float angle_err = _effective_setpoint - _tilt_angle;
    float motor_vel = _kp * angle_err - _kd * _gyro_rate + _velocity_bias;

    if (motor_vel >  BALANCE_MAX_DRIVE_SPEED) motor_vel =  BALANCE_MAX_DRIVE_SPEED;
    if (motor_vel < -BALANCE_MAX_DRIVE_SPEED) motor_vel = -BALANCE_MAX_DRIVE_SPEED;

    float pos_delta = motor_vel * dt;
    _back_left_target  += pos_delta;
    _back_right_target += pos_delta;

    if (_motors->isDriveArmed()) {
        _motors->sendDrivePosition(MotorRole::BackLeft,  _back_left_target,  BALANCE_MAX_DRIVE_SPEED);
        _motors->sendDrivePosition(MotorRole::BackRight, _back_right_target, BALANCE_MAX_DRIVE_SPEED);

        _motors->sendDrivePosition(MotorRole::FrontLeft,  _front_left_hold,  0.0f);
        _motors->sendDrivePosition(MotorRole::FrontRight, _front_right_hold, 0.0f);
    }

    _last_angle_err = angle_err;
    _last_motor_vel = motor_vel;
}

// ---------------------------------------------------------------------------
// State transitions (Core 1 only)
// ---------------------------------------------------------------------------

void BalanceController::enterTippingUp() {
    _state = BalanceState::TippingUp;
    _targets_initialized = false;

    float fwd_l = _arms->getForwardLeft();
    float fwd_r = _arms->getForwardRight();

    _arm_left_target  = _motors->getMotor(MotorRole::ArmLeft).position;
    _arm_right_target = _motors->getMotor(MotorRole::ArmRight).position;
    _arm_tip_left_goal  = fwd_l + BALANCE_ARM_TIP_LEFT;
    _arm_tip_right_goal = fwd_r + BALANCE_ARM_TIP_RIGHT;
    _arm_left_goal    = _arm_tip_left_goal;
    _arm_right_goal   = _arm_tip_right_goal;
    _arm_ramp_speed   = BALANCE_ARM_TIP_SPEED;
    _arms_reached_tip = false;

    _arms->setOverrideTargets(_arm_left_target, _arm_right_target, _arm_ramp_speed);

    Serial.printf("[Balance] TIPPING UP  arm goal: L=%.2f R=%.2f\n",
                  _arm_left_goal, _arm_right_goal);

    startLog();
}

void BalanceController::enterBalancing(float current_roll) {
    _state = BalanceState::Balancing;

    _adaptive_adjustment = 0.0f;
    _cumulative_error = 0.0f;
    _ff_setpoint = current_roll;
    _effective_setpoint = current_roll;

    _back_left_target  = _motors->getMotor(MotorRole::BackLeft).position;
    _back_right_target = _motors->getMotor(MotorRole::BackRight).position;
    _targets_initialized = true;

    _wheel_start_pos = (_back_left_target + _back_right_target) * 0.5f;
    _pos_integral = 0.0f;
    _prev_wheel_drift = 0.0f;

    _front_left_hold  = _motors->getMotor(MotorRole::FrontLeft).position;
    _front_right_hold = _motors->getMotor(MotorRole::FrontRight).position;

    _arms_reached_tip = true;
    _arms_returning   = false;
    _balance_start_ms = millis();

    Serial.printf("[Balance] BALANCING @200Hz  base=%.1f ff_gain=%.1f  Kp=%.3f Kd=%.4f  tilt=%.1f  wheels: L=%.2f R=%.2f\n",
                  _base_deg, _ff_gain, (float)_kp, (float)_kd, (float)_tilt_angle,
                  (float)_back_left_target, (float)_back_right_target);
}

void BalanceController::enterReturningArms() {
    _state = BalanceState::ReturningArms;
    _targets_initialized = false;

    _arm_left_goal  = _arms->getForwardLeft();
    _arm_right_goal = _arms->getForwardRight();
    _arm_ramp_speed = BALANCE_ARM_RETURN_SPEED;

    stopLog();

    Serial.println("[Balance] RETURNING ARMS to forward reference");
}

void BalanceController::forceEngage() {
    if (!_motors || !_arms) return;
    if (!_motors->isDriveArmed()) {
        Serial.println("[Balance] Cannot engage -- drive not armed");
        return;
    }
    float tilt = _filter_initialized ? (float)_tilt_angle : 0.0f;
    Serial.printf("[Balance] FORCE ENGAGE at tilt=%.1f\n", tilt);
    _arms_returning = false;
    _arms_reached_tip = true;
    startLog();
    enterBalancing(tilt);
}

void BalanceController::disengage() {
    BalanceState prev = _state;

    if (prev == BalanceState::Idle || prev == BalanceState::ReturningArms) {
        _state = BalanceState::Idle;
        _targets_initialized = false;
        if (_arms) {
            _arms->clearOverride();
        }
        stopLog();
        return;
    }

    enterReturningArms();
    Serial.printf("[Balance] Disengaged from %s -> ReturningArms\n",
                  prev == BalanceState::TippingUp ? "TIP_UP" : "BALANCE");
}

// ---------------------------------------------------------------------------
// State machine + outer loops (Core 1, 50Hz)
// ---------------------------------------------------------------------------

void BalanceController::update(float roll_deg, float roll_rate_dps,
                                bool ch7_active, bool ch11_edge, float dt) {
    if (!_motors || !_arms) return;

    // Use complementary filter output when available, fall back to Madgwick
    float tilt = _filter_initialized ? (float)_tilt_angle : roll_deg;
    float rate = _filter_initialized ? (float)_gyro_rate : roll_rate_dps;

    if (_logging && (millis() - _log_start_ms >= BALANCE_LOG_DURATION_MS)) {
        stopLog();
    }

    if (!ch7_active) {
        if (_state != BalanceState::Idle) {
            if (_state == BalanceState::ReturningArms) {
                _state = BalanceState::Idle;
                _targets_initialized = false;
                if (_arms) _arms->clearOverride();
                stopLog();
            } else {
                disengage();
            }
        }
        return;
    }

    switch (_state) {

    case BalanceState::Idle:
        if (ch11_edge && _motors->isArmArmed()) {
            enterTippingUp();
        }
        break;

    case BalanceState::TippingUp: {
        float dist_l = fabsf(_arm_left_goal - _arm_left_target);
        float dist_r = fabsf(_arm_right_goal - _arm_right_target);

        float scale_l = dist_l / 1.5f;
        if (scale_l > 1.0f) scale_l = 1.0f;
        if (scale_l < 0.05f) scale_l = 0.05f;

        float scale_r = dist_r / 1.5f;
        if (scale_r > 1.0f) scale_r = 1.0f;
        if (scale_r < 0.05f) scale_r = 0.05f;

        float speed_l = BALANCE_ARM_TIP_SPEED * scale_l;
        float speed_r = BALANCE_ARM_TIP_SPEED * scale_r;

        _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  speed_l, dt);
        _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, speed_r, dt);

        float motor_speed = (speed_l > speed_r) ? speed_l : speed_r;
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, motor_speed);

        float arm_err_l = fabsf(_arm_left_target - _arm_tip_left_goal);
        float arm_err_r = fabsf(_arm_right_target - _arm_tip_right_goal);
        bool arms_done = (arm_err_l < 0.05f && arm_err_r < 0.05f);

        // Compute expected balance from current arm position for engage check
        float tip_dl = _motors->getMotor(MotorRole::ArmLeft).position - _arms->getForwardLeft();
        float tip_dr = _motors->getMotor(MotorRole::ArmRight).position - _arms->getForwardRight();
        float tip_expected = _base_deg + _ff_gain * (tip_dl + tip_dr) * 0.5f;
        float tip_error = fabsf(tip_expected - tilt);
        if (arms_done &&
            tip_error < BALANCE_ENGAGE_THRESHOLD_DEG &&
            fabsf(rate) < BALANCE_ENGAGE_RATE_MAX_DPS) {
            enterBalancing(tilt);
        }

        logSample(tilt, rate);
        break;
    }

    case BalanceState::Balancing: {
        // Feedforward: compute expected balance point from current arm position
        float arm_delta_l = _motors->getMotor(MotorRole::ArmLeft).position - _arms->getForwardLeft();
        float arm_delta_r = _motors->getMotor(MotorRole::ArmRight).position - _arms->getForwardRight();
        float arm_avg_delta = (arm_delta_l + arm_delta_r) * 0.5f;
        _ff_setpoint = _base_deg + _ff_gain * arm_avg_delta;

        // Safety bailout
        if (fabsf(_ff_setpoint - tilt) > BALANCE_BAILOUT_THRESHOLD_DEG) {
            Serial.printf("[Balance] BAILOUT  tilt=%.1f ff_sp=%.1f\n", tilt, _ff_setpoint);
            disengage();
            return;
        }

        // Gentle adaptive fine-tuning on top of feedforward
        float ff_err = _ff_setpoint - tilt;
        if (fabsf(ff_err) < 10.0f && fabsf(rate) < 30.0f) {
            _cumulative_error += ff_err * dt;
            if (_cumulative_error >  20.0f) _cumulative_error =  20.0f;
            if (_cumulative_error < -20.0f) _cumulative_error = -20.0f;
        }
        _adaptive_adjustment = _adapt_rate * _cumulative_error;
        if (_adaptive_adjustment >  5.0f) _adaptive_adjustment =  5.0f;
        if (_adaptive_adjustment < -5.0f) _adaptive_adjustment = -5.0f;

        // Position return PID as velocity bias
        float wheel_avg = (_back_left_target + _back_right_target) * 0.5f;
        float wheel_drift = wheel_avg - _wheel_start_pos;
        float wheel_vel = (wheel_drift - _prev_wheel_drift) / dt;
        _prev_wheel_drift = wheel_drift;

        _pos_integral += wheel_drift * dt;
        if (_pos_integral >  _pos_integral_max) _pos_integral =  _pos_integral_max;
        if (_pos_integral < -_pos_integral_max) _pos_integral = -_pos_integral_max;

        // Wall escape: when drift is large but wheels barely move, the robot
        // is stuck against an obstacle. Normal velocity bias just pushes harder
        // into the wall. Instead, shift the setpoint to make the robot LEAN AWAY
        // from the wall, which tips it in the escape direction.
        bool at_wall = (fabsf(wheel_drift) > 3.0f) && (fabsf(wheel_vel) < 1.0f);

        float wall_setpoint_shift = 0.0f;
        if (at_wall) {
            // Drift positive (drove forward, wall ahead): drop setpoint below roll → PD drives backward
            // Drift negative (drove backward, wall behind): raise setpoint above roll → PD drives forward
            float wall_lean = -0.5f * wheel_drift;
            if (wall_lean >  5.0f) wall_lean =  5.0f;
            if (wall_lean < -5.0f) wall_lean = -5.0f;
            wall_setpoint_shift = wall_lean;
            // Disable velocity bias at wall -- it fights the lean escape
            _velocity_bias = 0.0f;
        } else {
            // Normal: velocity bias for position return
            _velocity_bias = -(_pos_kp * wheel_drift
                             + _pos_ki * _pos_integral
                             + _pos_kd * wheel_vel);
        }

        // Effective setpoint = feedforward + adaptive + wall escape
        _effective_setpoint = _ff_setpoint + _adaptive_adjustment + wall_setpoint_shift;

        // Arm return (1s hold then return)
        if (!_arms_returning && (millis() - _balance_start_ms >= 1000)) {
            _arms_returning = true;
            _arm_left_goal  = _arms->getForwardLeft();
            _arm_right_goal = _arms->getForwardRight();
            Serial.println("[Balance] Arms beginning return to forward");
        }

        if (_arms_returning) {
            _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  BALANCE_ARM_RETURN_SPEED, dt);
            _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, BALANCE_ARM_RETURN_SPEED, dt);
        }
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target,
                                  _arms_returning ? BALANCE_ARM_RETURN_SPEED : 0.0f);

        logSample(tilt, rate);
        break;
    }

    case BalanceState::ReturningArms: {
        _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  _arm_ramp_speed, dt);
        _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, _arm_ramp_speed, dt);
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, _arm_ramp_speed);

        float arm_err_l = fabsf(_arm_left_target - _arm_left_goal);
        float arm_err_r = fabsf(_arm_right_target - _arm_right_goal);
        if (arm_err_l < 0.05f && arm_err_r < 0.05f) {
            _state = BalanceState::Idle;
            _arms->clearOverride();
            Serial.println("[Balance] Arms returned -> Idle");
        }
        break;
    }

    } // switch
}

// ---------------------------------------------------------------------------
// Telemetry logging
// ---------------------------------------------------------------------------

void BalanceController::startLog() {
    if (_logging) return;
    if (!_log_buf) return;

    _log_count = 0;
    _log_saved = false;
    _log_start_ms = millis();
    _logging = true;

    _last_angle_err = 0.0f;
    _last_motor_vel = 0.0f;

    Serial.println("[Balance] Telemetry log started (PSRAM buffer)");
}

void BalanceController::logSample(float roll_deg, float roll_rate_dps) {
    if (!_logging || !_log_buf) return;
    if (_log_count >= BALANCE_LOG_MAX_SAMPLES) return;

    BalanceSample& s = _log_buf[_log_count];
    s.t_ms      = millis() - _log_start_ms;
    s.state     = static_cast<uint8_t>(_state);
    s.roll      = roll_deg;
    s.roll_rate = roll_rate_dps;
    s.setpoint  = (_state == BalanceState::Balancing) ? (float)_effective_setpoint : _ff_setpoint;
    s.angle_err = _last_angle_err;
    s.motor_vel = _last_motor_vel;
    s.integral  = _cumulative_error;
    s.bl_pos    = _motors->getMotor(MotorRole::BackLeft).position;
    s.br_pos    = _motors->getMotor(MotorRole::BackRight).position;
    s.bl_vel    = _motors->getMotor(MotorRole::BackLeft).velocity;
    s.br_vel    = _motors->getMotor(MotorRole::BackRight).velocity;
    s.arm_l     = _motors->getMotor(MotorRole::ArmLeft).position;
    s.arm_r     = _motors->getMotor(MotorRole::ArmRight).position;
    _log_count++;
}

void BalanceController::stopLog() {
    if (!_logging) return;
    _logging = false;

    uint32_t duration = millis() - _log_start_ms;
    Serial.printf("[Balance] Telemetry log stopped (%lu ms, %d samples)\n",
                  duration, _log_count);

    flushLogToFile();
}

void BalanceController::flushLogToFile() {
    if (_log_count == 0 || !_log_buf) return;

    Serial.printf("[Balance] Writing %d samples to %s...\n", _log_count, BALANCE_LOG_PATH);
    uint32_t start = millis();

    File f = LittleFS.open(BALANCE_LOG_PATH, "w");
    if (!f) {
        Serial.println("[Balance] WARNING: could not open log file for writing");
        return;
    }

    f.println("t_ms,state,roll,roll_rate,setpoint,angle_err,motor_vel,integral,bl_pos,br_pos,bl_vel,br_vel,arm_l,arm_r");

    for (int i = 0; i < _log_count; i++) {
        const BalanceSample& s = _log_buf[i];
        f.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                 s.t_ms, s.state,
                 s.roll, s.roll_rate, s.setpoint,
                 s.angle_err, s.motor_vel, s.integral,
                 s.bl_pos, s.br_pos, s.bl_vel, s.br_vel,
                 s.arm_l, s.arm_r);
    }

    f.close();
    _log_saved = true;

    Serial.printf("[Balance] Log saved (%lu ms to write)\n", millis() - start);
}

void BalanceController::dumpLog() {
    if (_log_count > 0 && !_log_saved) {
        flushLogToFile();
    }

    File f = LittleFS.open(BALANCE_LOG_PATH, "r");
    if (!f) {
        Serial.println("[Balance] No log file found");
        return;
    }
    Serial.printf("[Balance] --- Log dump (%u bytes, %d samples) ---\n", f.size(), _log_count);
    while (f.available()) {
        Serial.write(f.read());
    }
    f.close();
    Serial.println("[Balance] --- End of log ---");
}

void BalanceController::clearLog() {
    if (_logging) {
        stopLog();
    }
    _log_count = 0;
    _log_saved = false;
    LittleFS.remove(BALANCE_LOG_PATH);
    Serial.println("[Balance] Log file deleted");
}

bool BalanceController::hasLog() const {
    if (_log_count > 0) return true;
    return LittleFS.exists(BALANCE_LOG_PATH);
}

size_t BalanceController::logSize() const {
    if (_log_saved) {
        File f = LittleFS.open(BALANCE_LOG_PATH, "r");
        if (!f) return 0;
        size_t sz = f.size();
        f.close();
        return sz;
    }
    return _log_count * sizeof(BalanceSample);
}

// ---------------------------------------------------------------------------
// Debug / status
// ---------------------------------------------------------------------------

const char* BalanceController::getStateString() const {
    switch (_state) {
        case BalanceState::Idle:          return "IDLE";
        case BalanceState::TippingUp:     return "TIP_UP";
        case BalanceState::Balancing:     return "BALANCE";
        case BalanceState::ReturningArms: return "RET_ARMS";
    }
    return "?";
}

void BalanceController::printStatus() {
    Serial.println("=== BALANCE STATUS ===");
    Serial.printf("  State: %s\n", getStateString());
    Serial.printf("  Feedforward: base=%.1f  ff_gain=%.1f  ff_sp=%.1f\n", _base_deg, _ff_gain, _ff_setpoint);
    Serial.printf("  PD: Kp=%.4f  Kd=%.4f  adapt_rate=%.3f\n", (float)_kp, (float)_kd, _adapt_rate);
    Serial.printf("  Balance loop: %d Hz (Core 0)\n", BALANCE_LOOP_HZ);
    Serial.printf("  Complementary filter: tilt=%.1f  gyro_rate=%.1f\n",
                  (float)_tilt_angle, (float)_gyro_rate);
    Serial.printf("  Max drive speed: %.1f rad/s\n", BALANCE_MAX_DRIVE_SPEED);

    if (_state == BalanceState::Balancing) {
        float wheel_avg = (_back_left_target + _back_right_target) * 0.5f;
        float drift = wheel_avg - _wheel_start_pos;
        Serial.printf("  Effective setpoint: %.2f  (ff=%.1f + adapt=%+.2f)\n",
                      (float)_effective_setpoint, _ff_setpoint, _adaptive_adjustment);
        Serial.printf("  Velocity bias: %+.2f  Wheel drift: %+.1f rad\n",
                      (float)_velocity_bias, drift);
        Serial.printf("  Last motor vel: %.2f rad/s\n", (float)_last_motor_vel);
    }

    if (hasLog()) {
        Serial.printf("  Log file: %s (%u bytes)\n", BALANCE_LOG_PATH, logSize());
    } else {
        Serial.println("  Log file: none");
    }
    Serial.printf("  Logging active: %s\n", _logging ? "YES" : "no");
    Serial.println("======================");
}
