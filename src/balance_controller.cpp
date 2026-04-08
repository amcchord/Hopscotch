#include "balance_controller.h"
#include <Arduino.h>
#include <cmath>

// ---------------------------------------------------------------------------
// PID helpers
// ---------------------------------------------------------------------------

float BalanceController::pidCompute(PidState& pid, float error, float dt) {
    pid.integral += error * dt;
    if (pid.integral >  pid.i_max) pid.integral =  pid.i_max;
    if (pid.integral < -pid.i_max) pid.integral = -pid.i_max;

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid.prev_error) / dt;
    }
    pid.prev_error = error;

    float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    if (output > pid.out_max) output = pid.out_max;
    if (output < pid.out_min) output = pid.out_min;
    return output;
}

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

    _outer.kp    = BALANCE_OUTER_KP;
    _outer.ki    = BALANCE_OUTER_KI;
    _outer.kd    = BALANCE_OUTER_KD;
    _outer.i_max = BALANCE_OUTER_I_MAX;
    _outer.out_min = -180.0f;
    _outer.out_max =  180.0f;

    _inner.kp    = BALANCE_INNER_KP;
    _inner.ki    = BALANCE_INNER_KI;
    _inner.kd    = BALANCE_INNER_KD;
    _inner.i_max = 50.0f;
    _inner.out_min = -BALANCE_MAX_DRIVE_SPEED;
    _inner.out_max =  BALANCE_MAX_DRIVE_SPEED;
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
    _arm_left_goal    = fwd_l + BALANCE_ARM_TIP_LEFT;
    _arm_right_goal   = fwd_r + BALANCE_ARM_TIP_RIGHT;
    _arm_ramp_speed   = BALANCE_ARM_TIP_SPEED;

    _arms->setOverrideTargets(_arm_left_target, _arm_right_target, _arm_ramp_speed);

    Serial.printf("[Balance] TIPPING UP  arm goal: L=%.2f R=%.2f\n",
                  _arm_left_goal, _arm_right_goal);

    startLog();
}

void BalanceController::enterBalancing() {
    _state = BalanceState::Balancing;

    _outer.integral   = 0.0f;
    _outer.prev_error = 0.0f;
    _inner.integral   = 0.0f;
    _inner.prev_error = 0.0f;

    _back_left_target  = _motors->getMotor(MotorRole::BackLeft).position;
    _back_right_target = _motors->getMotor(MotorRole::BackRight).position;
    _targets_initialized = true;

    _front_left_hold  = _motors->getMotor(MotorRole::FrontLeft).position;
    _front_right_hold = _motors->getMotor(MotorRole::FrontRight).position;

    float fwd_l = _arms->getForwardLeft();
    float fwd_r = _arms->getForwardRight();
    _arm_left_goal  = fwd_l;
    _arm_right_goal = fwd_r;
    _arm_ramp_speed = BALANCE_ARM_RETURN_SPEED;

    Serial.printf("[Balance] BALANCING  setpoint=%.1f  back_pos: L=%.2f R=%.2f\n",
                  _setpoint_deg, _back_left_target, _back_right_target);
}

void BalanceController::disengage() {
    BalanceState prev = _state;
    _state = BalanceState::Idle;
    _targets_initialized = false;

    if (_arms) {
        _arms->clearOverride();
    }

    stopLog();

    if (prev != BalanceState::Idle) {
        Serial.println("[Balance] DISENGAGED -> Idle");
    }
}

// ---------------------------------------------------------------------------
// Main update (called at 50 Hz from main loop)
// ---------------------------------------------------------------------------

void BalanceController::update(float roll_deg, float roll_rate_dps,
                                bool ch7_active, bool ch11_edge, float dt) {
    if (!_motors || !_arms) return;

    // Auto-stop log after duration
    if (_logging && (millis() - _log_start_ms >= BALANCE_LOG_DURATION_MS)) {
        stopLog();
    }

    // Ch7 going LOW always disengages
    if (!ch7_active) {
        if (_state != BalanceState::Idle) {
            disengage();
        }
        return;
    }

    float angle_error = _setpoint_deg - roll_deg;

    switch (_state) {

    case BalanceState::Idle:
        // Ch7 is high and Ch11 edge triggers tip-up (only if arms are armed)
        if (ch11_edge && _motors->isArmArmed()) {
            enterTippingUp();
        }
        break;

    case BalanceState::TippingUp: {
        // Ramp arms toward tip-up goal
        _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  _arm_ramp_speed, dt);
        _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, _arm_ramp_speed, dt);
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, _arm_ramp_speed);

        // Check if roll is close enough to engage PID
        if (fabsf(angle_error) < BALANCE_ENGAGE_THRESHOLD_DEG) {
            enterBalancing();
        }

        logSample(roll_deg, roll_rate_dps);
        break;
    }

    case BalanceState::Balancing: {
        // Safety bailout
        if (fabsf(angle_error) > BALANCE_BAILOUT_THRESHOLD_DEG) {
            Serial.printf("[Balance] BAILOUT  roll=%.1f err=%.1f\n", roll_deg, angle_error);
            disengage();
            return;
        }

        // Outer PID: angle error -> desired angular rate (deg/s)
        float desired_rate = pidCompute(_outer, angle_error, dt);

        // Inner PID: rate error -> motor velocity (rad/s)
        float rate_error = desired_rate - roll_rate_dps;
        float motor_vel  = pidCompute(_inner, rate_error, dt);

        // Accumulate position targets for back wheels
        float pos_delta = motor_vel * dt;
        _back_left_target  += pos_delta;
        _back_right_target += pos_delta;

        // Send commands to back wheels
        if (_motors->isDriveArmed()) {
            _motors->sendDrivePosition(MotorRole::BackLeft,  _back_left_target,  BALANCE_MAX_DRIVE_SPEED);
            _motors->sendDrivePosition(MotorRole::BackRight, _back_right_target, BALANCE_MAX_DRIVE_SPEED);

            // Hold front wheels in place
            _motors->sendDrivePosition(MotorRole::FrontLeft,  _front_left_hold,  0.0f);
            _motors->sendDrivePosition(MotorRole::FrontRight, _front_right_hold, 0.0f);
        }

        // Ramp arms back toward forward reference
        _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  _arm_ramp_speed, dt);
        _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, _arm_ramp_speed, dt);
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, _arm_ramp_speed);

        // Store for logging
        _last_outer_err = angle_error;
        _last_outer_out = desired_rate;
        _last_inner_err = rate_error;
        _last_inner_out = motor_vel;
        _last_motor_vel = motor_vel;

        logSample(roll_deg, roll_rate_dps);
        break;
    }

    } // switch
}

// ---------------------------------------------------------------------------
// Telemetry logging
// ---------------------------------------------------------------------------

void BalanceController::startLog() {
    if (_logging) return;

    _log_file = LittleFS.open(BALANCE_LOG_PATH, "w");
    if (!_log_file) {
        Serial.println("[Balance] WARNING: could not open log file");
        return;
    }

    _log_file.println("t_ms,state,roll,roll_rate,setpoint,outer_err,outer_out,inner_err,inner_out,motor_vel,bl_pos,br_pos,bl_vel,br_vel,arm_l,arm_r");
    _log_start_ms = millis();
    _logging = true;

    _last_outer_err = 0.0f;
    _last_outer_out = 0.0f;
    _last_inner_err = 0.0f;
    _last_inner_out = 0.0f;
    _last_motor_vel = 0.0f;

    Serial.println("[Balance] Telemetry log started");
}

void BalanceController::logSample(float roll_deg, float roll_rate_dps) {
    if (!_logging || !_log_file) return;

    uint32_t t = millis() - _log_start_ms;

    float bl_pos = _motors->getMotor(MotorRole::BackLeft).position;
    float br_pos = _motors->getMotor(MotorRole::BackRight).position;
    float bl_vel = _motors->getMotor(MotorRole::BackLeft).velocity;
    float br_vel = _motors->getMotor(MotorRole::BackRight).velocity;
    float arm_l  = _motors->getMotor(MotorRole::ArmLeft).position;
    float arm_r  = _motors->getMotor(MotorRole::ArmRight).position;

    _log_file.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                     t,
                     static_cast<int>(_state),
                     roll_deg, roll_rate_dps,
                     _setpoint_deg,
                     _last_outer_err, _last_outer_out,
                     _last_inner_err, _last_inner_out,
                     _last_motor_vel,
                     bl_pos, br_pos, bl_vel, br_vel,
                     arm_l, arm_r);
}

void BalanceController::stopLog() {
    if (!_logging) return;
    _log_file.close();
    _logging = false;
    Serial.printf("[Balance] Telemetry log stopped (%lu ms recorded)\n",
                  millis() - _log_start_ms);
}

void BalanceController::dumpLog() {
    File f = LittleFS.open(BALANCE_LOG_PATH, "r");
    if (!f) {
        Serial.println("[Balance] No log file found");
        return;
    }
    Serial.printf("[Balance] --- Log dump (%u bytes) ---\n", f.size());
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
    LittleFS.remove(BALANCE_LOG_PATH);
    Serial.println("[Balance] Log file deleted");
}

bool BalanceController::hasLog() const {
    return LittleFS.exists(BALANCE_LOG_PATH);
}

size_t BalanceController::logSize() const {
    File f = LittleFS.open(BALANCE_LOG_PATH, "r");
    if (!f) return 0;
    size_t sz = f.size();
    f.close();
    return sz;
}

// ---------------------------------------------------------------------------
// Debug / status
// ---------------------------------------------------------------------------

const char* BalanceController::getStateString() const {
    switch (_state) {
        case BalanceState::Idle:      return "IDLE";
        case BalanceState::TippingUp: return "TIP_UP";
        case BalanceState::Balancing: return "BALANCE";
    }
    return "?";
}

void BalanceController::printStatus() {
    Serial.println("=== BALANCE STATUS ===");
    Serial.printf("  State: %s\n", getStateString());
    Serial.printf("  Setpoint: %.1f deg\n", _setpoint_deg);
    Serial.printf("  Outer PID: Kp=%.3f Ki=%.3f Kd=%.3f  (I_max=%.1f)\n",
                  _outer.kp, _outer.ki, _outer.kd, _outer.i_max);
    Serial.printf("  Inner PID: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  _inner.kp, _inner.ki, _inner.kd);
    Serial.printf("  Max drive speed: %.1f rad/s\n", BALANCE_MAX_DRIVE_SPEED);

    if (_state == BalanceState::Balancing) {
        Serial.printf("  Outer integral: %.2f\n", _outer.integral);
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
