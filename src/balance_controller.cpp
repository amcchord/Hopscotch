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
// State transitions
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

    // Initialize so adaptive setpoint starts at current roll (zero initial error).
    // Formula: adaptive_sp = setpoint_deg - adapt_rate * cumulative_error
    // So: cumulative_error = (setpoint_deg - current_roll) / adapt_rate
    _cumulative_error = (_setpoint_deg - current_roll) / _adapt_rate;
    _adaptive_setpoint = current_roll;

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

    Serial.printf("[Balance] BALANCING (PD+adapt+posPID)  sp=%.1f Kp=%.3f Kd=%.4f adapt=%.2f  wheels: L=%.2f R=%.2f\n",
                  _setpoint_deg, _kp, _kd, _adapt_rate, _back_left_target, _back_right_target);
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
// Main update (called at 50 Hz from main loop)
// ---------------------------------------------------------------------------

void BalanceController::update(float roll_deg, float roll_rate_dps,
                                bool ch7_active, bool ch11_edge, float dt) {
    if (!_motors || !_arms) return;

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

        float tip_error = fabsf(_setpoint_deg - roll_deg);
        if (arms_done &&
            tip_error < BALANCE_ENGAGE_THRESHOLD_DEG &&
            fabsf(roll_rate_dps) < BALANCE_ENGAGE_RATE_MAX_DPS) {
            enterBalancing(roll_deg);
        }

        logSample(roll_deg, roll_rate_dps);
        break;
    }

    case BalanceState::Balancing: {
        // Safety bailout (use base setpoint, not adaptive)
        if (fabsf(_setpoint_deg - roll_deg) > BALANCE_BAILOUT_THRESHOLD_DEG) {
            Serial.printf("[Balance] BAILOUT  roll=%.1f err=%.1f\n", roll_deg, _setpoint_deg - roll_deg);
            disengage();
            return;
        }

        // Adaptive setpoint: track cumulative error to find the true balance point.
        // If the robot consistently sits below the setpoint, the setpoint drifts down
        // to meet it. This separates "finding equilibrium" from "maintaining balance."
        // Only adapt the setpoint when the robot is in a calm, steady state.
        // Freeze during pushes, large errors, and fast motion so the
        // setpoint doesn't chase transient disturbances.
        float base_err = _setpoint_deg - roll_deg;
        if (fabsf(base_err) < 12.0f && fabsf(roll_rate_dps) < 30.0f) {
            _cumulative_error += base_err * dt;
            if (_cumulative_error >  50.0f) _cumulative_error =  50.0f;
            if (_cumulative_error < -50.0f) _cumulative_error = -50.0f;
        }

        _adaptive_setpoint = _setpoint_deg - _adapt_rate * _cumulative_error;
        if (_adaptive_setpoint > _setpoint_deg + 8.0f) _adaptive_setpoint = _setpoint_deg + 8.0f;
        if (_adaptive_setpoint < _setpoint_deg - 8.0f) _adaptive_setpoint = _setpoint_deg - 8.0f;

        // Position PID: drives the robot back toward its starting wheel position.
        // Outputs a lean angle offset (degrees) that shifts the balance setpoint.
        float wheel_avg = (_back_left_target + _back_right_target) * 0.5f;
        float wheel_drift = wheel_avg - _wheel_start_pos;
        float wheel_vel = (wheel_drift - _prev_wheel_drift) / dt;
        _prev_wheel_drift = wheel_drift;

        _pos_integral += wheel_drift * dt;
        if (_pos_integral >  _pos_integral_max) _pos_integral =  _pos_integral_max;
        if (_pos_integral < -_pos_integral_max) _pos_integral = -_pos_integral_max;

        float pos_correction = _pos_kp * wheel_drift
                             + _pos_ki * _pos_integral
                             + _pos_kd * wheel_vel;

        // Balance PD with adaptive setpoint and position correction.
        // Positive drift (drove forward) → positive correction → lean backward → drive back.
        float angle_err = (_adaptive_setpoint + pos_correction) - roll_deg;
        float motor_vel = _kp * angle_err - _kd * roll_rate_dps;

        // Clamp to max drive speed
        if (motor_vel >  BALANCE_MAX_DRIVE_SPEED) motor_vel =  BALANCE_MAX_DRIVE_SPEED;
        if (motor_vel < -BALANCE_MAX_DRIVE_SPEED) motor_vel = -BALANCE_MAX_DRIVE_SPEED;

        // Accumulate position targets for back wheels
        float pos_delta = motor_vel * dt;
        _back_left_target  += pos_delta;
        _back_right_target += pos_delta;

        if (_motors->isDriveArmed()) {
            _motors->sendDrivePosition(MotorRole::BackLeft,  _back_left_target,  BALANCE_MAX_DRIVE_SPEED);
            _motors->sendDrivePosition(MotorRole::BackRight, _back_right_target, BALANCE_MAX_DRIVE_SPEED);

            _motors->sendDrivePosition(MotorRole::FrontLeft,  _front_left_hold,  0.0f);
            _motors->sendDrivePosition(MotorRole::FrontRight, _front_right_hold, 0.0f);
        }

        // Hold arms at tip for 1 second, then return -- don't lean on them
        if (!_arms_returning && (millis() - _balance_start_ms >= 1000)) {
            _arms_returning = true;
            _arm_left_goal  = _arms->getForwardLeft();
            _arm_right_goal = _arms->getForwardRight();
            _arm_ramp_speed = BALANCE_ARM_RETURN_SPEED;
            Serial.println("[Balance] Arms beginning return to forward");
        }

        if (_arms_returning) {
            _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  _arm_ramp_speed, dt);
            _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, _arm_ramp_speed, dt);
        }
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target,
                                  _arms_returning ? _arm_ramp_speed : 0.0f);

        _last_angle_err = angle_err;
        _last_motor_vel = motor_vel;

        logSample(roll_deg, roll_rate_dps);
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
// Telemetry logging -- buffered in PSRAM, flushed to LittleFS on stop
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
    s.setpoint  = (_state == BalanceState::Balancing) ? _adaptive_setpoint : _setpoint_deg;
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
    Serial.printf("  Setpoint: %.1f deg\n", _setpoint_deg);
    Serial.printf("  PD: Kp=%.4f  Kd=%.4f  adapt_rate=%.3f\n", _kp, _kd, _adapt_rate);
    Serial.printf("  Max drive speed: %.1f rad/s\n", BALANCE_MAX_DRIVE_SPEED);

    if (_state == BalanceState::Balancing) {
        float wheel_avg = (_back_left_target + _back_right_target) * 0.5f;
        float drift = wheel_avg - _wheel_start_pos;
        float pos_corr = _pos_kp * drift + _pos_ki * _pos_integral;
        Serial.printf("  Adaptive setpoint: %.2f (base: %.1f, shift: %+.2f)\n",
                      _adaptive_setpoint, _setpoint_deg, _adaptive_setpoint - _setpoint_deg);
        Serial.printf("  Cumulative error: %.2f\n", _cumulative_error);
        Serial.printf("  Wheel drift: %+.1f rad  pos_correction: %+.2f deg  (P=%+.2f I=%+.2f)\n",
                      drift, pos_corr, _pos_kp * drift, _pos_ki * _pos_integral);
        Serial.printf("  Last motor vel: %.2f rad/s\n", _last_motor_vel);
    }

    if (hasLog()) {
        Serial.printf("  Log file: %s (%u bytes)\n", BALANCE_LOG_PATH, logSize());
    } else {
        Serial.println("  Log file: none");
    }
    Serial.printf("  Logging active: %s\n", _logging ? "YES" : "no");
    Serial.println("======================");
}
