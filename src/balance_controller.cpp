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

float BalanceController::clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float BalanceController::computeArmFraction() const {
    if (!_motors || !_arms) {
        return 1.0f;
    }

    float arm_delta_l = _motors->getMotor(MotorRole::ArmLeft).position - _arms->getForwardLeft();
    float arm_delta_r = _motors->getMotor(MotorRole::ArmRight).position - _arms->getForwardRight();
    float arm_avg_delta = (arm_delta_l + arm_delta_r) * 0.5f;
    float tip_avg_delta = (BALANCE_ARM_TIP_LEFT + BALANCE_ARM_TIP_RIGHT) * 0.5f;

    float arm_frac = 0.0f;
    if (tip_avg_delta > 0.01f) {
        arm_frac = arm_avg_delta / tip_avg_delta;
    }
    return clampf(arm_frac, 0.0f, 1.0f);
}

float BalanceController::computeScheduledSetpoint() const {
    float arm_frac = computeArmFraction();
    return BALANCE_SETPOINT_ARMS_FWD
         + arm_frac * (BALANCE_SETPOINT_ARMS_TIP - BALANCE_SETPOINT_ARMS_FWD);
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
    float accel_angle = atan2f(imu.accel_y, imu.accel_z) * 57.2957795f;
    float gyro_raw = imu.gyro_x;

    if (!_filter_initialized) {
        _tilt_angle = accel_angle;
        _gyro_rate = gyro_raw;
        _filter_initialized = true;
    } else {
        _tilt_angle = COMPLEMENTARY_ALPHA * (_tilt_angle + gyro_raw * dt)
                    + (1.0f - COMPLEMENTARY_ALPHA) * accel_angle;
        _gyro_rate = 0.08f * gyro_raw + 0.92f * _gyro_rate;
    }

    if (_state != BalanceState::Balancing) return;
    if (!_targets_initialized) return;

    float angle_err = _effective_setpoint - _tilt_angle;
    float motor_vel = _kp * angle_err - _kd * _gyro_rate;

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

    float scheduled_sp = computeScheduledSetpoint();
    _engage_arm_frac = computeArmFraction();
    _engage_capture_shift = clampf(current_roll - scheduled_sp,
                                   -BALANCE_CAPTURE_SHIFT_MAX_DEG,
                                   BALANCE_CAPTURE_SHIFT_MAX_DEG);
    _setpoint = 0.0f;
    _filtered_vel = 0.0f;
    float initial_base = BALANCE_USE_CAPTURE_SHIFT
                       ? scheduled_sp + _engage_capture_shift
                       : scheduled_sp;
    _smoothed_base_sp = initial_base;
    _effective_setpoint = clampf(initial_base, BALANCE_SETPOINT_MIN, BALANCE_SETPOINT_MAX);

    _back_left_target  = _motors->getMotor(MotorRole::BackLeft).position;
    _back_right_target = _motors->getMotor(MotorRole::BackRight).position;
    _targets_initialized = true;

    _wheel_start_pos = (_motors->getMotor(MotorRole::BackLeft).position
                      + _motors->getMotor(MotorRole::BackRight).position) * 0.5f;
    _pos_integral = 0.0f;
    _pos_setpoint_shift = 0.0f;

    _stuck = false;
    _stuck_start_ms = 0;
    resetSafetyTimers();

    _last_meas_drift = 0.0f;
    _last_meas_vel = 0.0f;
    _last_flags = 0;

    _front_left_hold  = _motors->getMotor(MotorRole::FrontLeft).position;
    _front_right_hold = _motors->getMotor(MotorRole::FrontRight).position;

    _arms_reached_tip = true;
    _arms_returning   = false;
    _arms_returned    = false;
    _ramp_complete    = false;
    _balance_start_ms = millis();
    _capture_stable   = false;
    _capture_stable_start_ms = 0;

    _arm_bal_frac     = 0.0f;
    _arm_bal_integral = 0.0f;
    _arm_sp_frac      = 0.0f;
    _arm_bal_active   = false;

    const ArmCalibration& cal = _arms->getCalibration();
    _arm_center_left  = cal.center_left;
    _arm_center_right = cal.center_right;

    Serial.printf("[Balance] BALANCING @200Hz  Kp=%.3f Kd=%.4f  tilt=%.1f  base=%.1f  capture=%.2f  eff=%.1f\n",
                  (float)_kp, (float)_kd, (float)_tilt_angle,
                  scheduled_sp, _engage_capture_shift, (float)_effective_setpoint);
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

void BalanceController::hardAbort(const char* reason) {
    BalanceState prev = _state;
    if (prev == BalanceState::Idle) return;

    _state = BalanceState::Idle;
    _targets_initialized = false;
    if (_arms) _arms->clearOverride();
    stopLog();

    const char* prev_str = (prev == BalanceState::Balancing) ? "BALANCE" :
                           (prev == BalanceState::TippingUp) ? "TIP_UP" : "RET_ARMS";
    Serial.printf("[Balance] HARD ABORT: %s (was %s)\n", reason, prev_str);
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

void BalanceController::resetSafetyTimers() {
    _safe_err_timing  = false;
    _safe_err_start_ms = 0;
    _safe_rate_timing = false;
    _safe_rate_start_ms = 0;
    _safe_sat_timing  = false;
    _safe_sat_start_ms = 0;
}

// ---------------------------------------------------------------------------
// State machine + outer loops (Core 1, 50Hz)
// ---------------------------------------------------------------------------

void BalanceController::update(float roll_deg, float roll_rate_dps,
                                bool ch7_active, bool ch11_edge, float dt) {
    if (!_motors || !_arms) return;

    float tilt = _filter_initialized ? (float)_tilt_angle : roll_deg;
    float rate = _filter_initialized ? (float)_gyro_rate : roll_rate_dps;

    if (_logging && (millis() - _log_start_ms >= BALANCE_LOG_DURATION_MS)) {
        _logging = false;
        Serial.printf("[Balance] Telemetry log stopped (%lu ms, %d samples) -- flush deferred\n",
                      millis() - _log_start_ms, _log_count);
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

        float tip_expected = BALANCE_SETPOINT_ARMS_TIP;
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
        // --- Measured rear-wheel odometry ---
        float meas_bl = _motors->getMotor(MotorRole::BackLeft).position;
        float meas_br = _motors->getMotor(MotorRole::BackRight).position;
        float meas_drift = (meas_bl + meas_br) * 0.5f - _wheel_start_pos;
        float meas_vel = (_motors->getMotor(MotorRole::BackLeft).velocity
                        + _motors->getMotor(MotorRole::BackRight).velocity) * 0.5f;

        _last_meas_drift = meas_drift;
        _last_meas_vel   = meas_vel;

        uint32_t now = millis();

        // ---------------------------------------------------------------
        // Safety checks (unchanged)
        // ---------------------------------------------------------------

        if (tilt < BALANCE_SAFE_TILT_MIN || tilt > BALANCE_SAFE_TILT_MAX) {
            Serial.printf("[Balance] SAFETY: tilt %.1f outside [%.0f, %.0f]\n",
                          tilt, BALANCE_SAFE_TILT_MIN, BALANCE_SAFE_TILT_MAX);
            hardAbort("tilt out of range (fallen)");
            return;
        }

        float eff_err = fabsf(_effective_setpoint - tilt);
        if (eff_err > BALANCE_SAFE_ERR_MAX_DEG) {
            if (!_safe_err_timing) {
                _safe_err_timing = true;
                _safe_err_start_ms = now;
            } else if (now - _safe_err_start_ms > BALANCE_SAFE_ERR_DURATION_MS) {
                Serial.printf("[Balance] SAFETY: error %.1f > %.0f for >%lu ms\n",
                              eff_err, BALANCE_SAFE_ERR_MAX_DEG, BALANCE_SAFE_ERR_DURATION_MS);
                disengage();
                return;
            }
        } else {
            _safe_err_timing = false;
        }

        if (fabsf(rate) > BALANCE_SAFE_RATE_MAX_DPS) {
            if (!_safe_rate_timing) {
                _safe_rate_timing = true;
                _safe_rate_start_ms = now;
            } else if (now - _safe_rate_start_ms > BALANCE_SAFE_RATE_DURATION_MS) {
                Serial.printf("[Balance] SAFETY: rate %.1f dps for >%lu ms\n",
                              fabsf(rate), BALANCE_SAFE_RATE_DURATION_MS);
                hardAbort("extreme rate");
                return;
            }
        } else {
            _safe_rate_timing = false;
        }

        float abs_cmd = fabsf((float)_last_motor_vel);
        if (abs_cmd >= BALANCE_MAX_DRIVE_SPEED * 0.95f) {
            if (!_safe_sat_timing) {
                _safe_sat_timing = true;
                _safe_sat_start_ms = now;
            } else if (now - _safe_sat_start_ms > BALANCE_SAFE_SAT_DURATION_MS) {
                Serial.printf("[Balance] SAFETY: motor saturated for >%lu ms\n",
                              BALANCE_SAFE_SAT_DURATION_MS);
                disengage();
                return;
            }
        } else {
            _safe_sat_timing = false;
        }

        if (fabsf(_effective_setpoint - tilt) > BALANCE_BAILOUT_THRESHOLD_DEG) {
            Serial.printf("[Balance] BAILOUT  tilt=%.1f sp=%.1f\n", tilt, (float)_effective_setpoint);
            disengage();
            return;
        }

        // ---------------------------------------------------------------
        // Arm-scheduled setpoint base
        //
        // Linearly interpolate between ARMS_TIP and ARMS_FWD balance
        // points based on how far the arms have returned. This gives
        // the PD the correct target at every arm position.
        // ---------------------------------------------------------------
        float scheduled_sp = BALANCE_USE_SCHEDULED_SP
                           ? computeScheduledSetpoint()
                           : BALANCE_SETPOINT_ARMS_FWD;

        float capture_shift = 0.0f;
        if (BALANCE_USE_CAPTURE_SHIFT) {
            float arm_frac = computeArmFraction();
            float capture_weight = 1.0f;
            if (_arms_returning) {
                capture_weight = (_engage_arm_frac > 0.05f)
                               ? clampf(arm_frac / _engage_arm_frac, 0.0f, 1.0f)
                               : 1.0f;
            }
            capture_shift = _engage_capture_shift * capture_weight;
        }

        float raw_base_sp = clampf(scheduled_sp + capture_shift,
                                   BALANCE_SETPOINT_MIN,
                                   BALANCE_SETPOINT_MAX);

        if (BALANCE_BASE_SP_RATE_MAX > 0.0f) {
            _smoothed_base_sp = moveToward(_smoothed_base_sp, raw_base_sp,
                                           BALANCE_BASE_SP_RATE_MAX, dt);
        } else {
            _smoothed_base_sp = raw_base_sp;
        }
        float base_effective_setpoint = _smoothed_base_sp;

        if (!_ramp_complete && _arms_returned
            && fabsf(_smoothed_base_sp - raw_base_sp) < 0.1f) {
            _ramp_complete = true;
            _arm_bal_active = true;
            _wheel_start_pos = (meas_bl + meas_br) * 0.5f;
            _arm_bal_frac = 0.0f;
            _arm_bal_integral = 0.0f;
            _arm_sp_frac = 0.0f;
            meas_drift = 0.0f;
            _last_meas_drift = 0.0f;
            Serial.printf("[Balance] Ramp complete -- arm balance active, origin reset (sp=%.1f)\n",
                          _smoothed_base_sp);
        }

        // ---------------------------------------------------------------
        // Arm return: hold tip pose until captured, then return
        // ---------------------------------------------------------------
        bool capture_ok = fabsf(_effective_setpoint - tilt) <= BALANCE_CAPTURE_ERR_MAX_DEG
                       && fabsf(rate) <= BALANCE_CAPTURE_RATE_MAX_DPS
                       && abs_cmd <= BALANCE_CAPTURE_CMD_MAX;

        if (capture_ok) {
            if (!_capture_stable) {
                _capture_stable = true;
                _capture_stable_start_ms = now;
            }
        } else {
            _capture_stable = false;
            _capture_stable_start_ms = 0;
        }

        bool capture_settled = _capture_stable
                            && (now - _capture_stable_start_ms >= BALANCE_CAPTURE_SETTLE_MS);
        bool hold_timed_out = (now - _balance_start_ms >= BALANCE_ARM_HOLD_MAX_MS);

        if (!_arms_returning && (capture_settled || hold_timed_out)) {
            _arms_returning = true;
            _arm_left_goal  = _arms->getForwardLeft();
            _arm_right_goal = _arms->getForwardRight();
            Serial.printf("[Balance] Arms beginning return to forward (%s after %lu ms)\n",
                          capture_settled ? "captured" : "settle timeout",
                          now - _balance_start_ms);
        }

        // ---------------------------------------------------------------
        // Arm balance PID (active after ramp complete only)
        // ---------------------------------------------------------------
        if (_arm_bal_active) {
            float drift_err = 0.0f;
            if (fabsf(meas_drift) > BALANCE_ARM_BAL_DEADBAND_RAD) {
                if (meas_drift > 0.0f) {
                    drift_err = meas_drift - BALANCE_ARM_BAL_DEADBAND_RAD;
                } else {
                    drift_err = meas_drift + BALANCE_ARM_BAL_DEADBAND_RAD;
                }
            }

            float pos_gate = 1.0f - clampf(fabsf(_effective_setpoint - tilt)
                                           / BALANCE_POS_GATE_ERR_DEG,
                                           0.0f, 1.0f);

            _arm_bal_integral *= BALANCE_ARM_BAL_DECAY;
            _arm_bal_integral += drift_err * pos_gate * dt;
            _arm_bal_integral = clampf(_arm_bal_integral,
                                       -BALANCE_ARM_BAL_INTEGRAL_MAX,
                                       BALANCE_ARM_BAL_INTEGRAL_MAX);

            float target_frac = (_arm_bal_kp * drift_err
                               + _arm_bal_ki * _arm_bal_integral
                               + _arm_bal_kd * meas_vel) * pos_gate;
            target_frac = clampf(target_frac, -_arm_bal_max_frac, _arm_bal_max_frac);

            _arm_bal_frac = moveToward(_arm_bal_frac, target_frac, BALANCE_ARM_BAL_FRAC_RATE, dt);

            float fwd_l = _arms->getForwardLeft();
            float fwd_r = _arms->getForwardRight();
            _arm_left_target  = fwd_l + _arm_bal_frac * _arm_center_left;
            _arm_right_target = fwd_r + _arm_bal_frac * _arm_center_right;
        }

        // ---------------------------------------------------------------
        // Arm return (only before ramp complete / arm balance takes over)
        // ---------------------------------------------------------------
        if (_arms_returning && !_arm_bal_active) {
            _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  BALANCE_ARM_RETURN_SPEED, dt);
            _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, BALANCE_ARM_RETURN_SPEED, dt);

            if (!_arms_returned) {
                float arm_err_l = fabsf(_arm_left_target - _arm_left_goal);
                float arm_err_r = fabsf(_arm_right_target - _arm_right_goal);
                if (arm_err_l < 0.08f && arm_err_r < 0.08f) {
                    _arms_returned = true;
                    Serial.printf("[Balance] Arms returned (drift=%.2f, base_sp=%.1f)\n",
                                  meas_drift, _smoothed_base_sp);
                }
            }
        }

        float arm_speed = BALANCE_ARM_RETURN_SPEED;
        if (_arm_bal_active) {
            arm_speed = BALANCE_ARM_BAL_MOTOR_SPEED;
        } else if (!_arms_returning) {
            arm_speed = 0.0f;
        }
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, arm_speed);

        // Frozen setpoint: arm CG shift creates mismatch vs fixed 92 deg target.
        // The mismatch drives sustained wheel motion (actual translation).
        _effective_setpoint = base_effective_setpoint;
        _effective_setpoint = clampf(_effective_setpoint, BALANCE_SETPOINT_MIN, BALANCE_SETPOINT_MAX);

        bool arm_correcting = fabsf(_arm_bal_frac) > 0.01f;
        _last_flags = (_stuck ? 0x01 : 0)
                    | (_safe_err_timing ? 0x02 : 0)
                    | (_safe_rate_timing ? 0x04 : 0)
                    | (_safe_sat_timing ? 0x08 : 0)
                    | (_capture_stable ? 0x10 : 0)
                    | (_arms_returning ? 0x20 : 0)
                    | (_ramp_complete ? 0x40 : 0)
                    | (arm_correcting ? 0x80 : 0);

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
    _last_meas_drift = 0.0f;
    _last_meas_vel = 0.0f;
    _last_flags = 0;

    Serial.println("[Balance] Telemetry log started (PSRAM buffer)");
}

void BalanceController::logSample(float roll_deg, float roll_rate_dps) {
    if (!_logging || !_log_buf) return;
    if (_log_count >= BALANCE_LOG_MAX_SAMPLES) return;

    BalanceSample& s = _log_buf[_log_count];
    s.t_ms           = millis() - _log_start_ms;
    s.state          = static_cast<uint8_t>(_state);
    s.roll           = roll_deg;
    s.roll_rate      = roll_rate_dps;
    s.setpoint       = (_state == BalanceState::Balancing) ? (float)_effective_setpoint : _setpoint;
    s.angle_err      = _last_angle_err;
    s.motor_vel      = _last_motor_vel;
    s.arm_bal_frac   = _arm_bal_frac;
    s.arm_sp_frac    = _arm_sp_frac;
    s.arm_bal_integ  = _arm_bal_integral;
    s.bl_pos         = _motors->getMotor(MotorRole::BackLeft).position;
    s.br_pos         = _motors->getMotor(MotorRole::BackRight).position;
    s.bl_vel         = _motors->getMotor(MotorRole::BackLeft).velocity;
    s.br_vel         = _motors->getMotor(MotorRole::BackRight).velocity;
    s.arm_l          = _motors->getMotor(MotorRole::ArmLeft).position;
    s.arm_r          = _motors->getMotor(MotorRole::ArmRight).position;
    s.arm_l_tgt      = _arm_left_target;
    s.arm_r_tgt      = _arm_right_target;
    s.meas_drift     = _last_meas_drift;
    s.meas_vel       = _last_meas_vel;
    s.flags          = _last_flags;
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

    f.println("t_ms,state,roll,roll_rate,setpoint,angle_err,motor_vel,"
              "arm_bal_frac,arm_sp_frac,arm_bal_integ,"
              "bl_pos,br_pos,bl_vel,br_vel,"
              "arm_l,arm_r,arm_l_tgt,arm_r_tgt,"
              "meas_drift,meas_vel,flags");

    for (int i = 0; i < _log_count; i++) {
        const BalanceSample& s = _log_buf[i];
        f.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,"
                 "%.4f,%.4f,%.4f,"
                 "%.2f,%.2f,%.2f,%.2f,"
                 "%.2f,%.2f,%.2f,%.2f,"
                 "%.2f,%.2f,%d\n",
                 s.t_ms, s.state,
                 s.roll, s.roll_rate, s.setpoint,
                 s.angle_err, s.motor_vel,
                 s.arm_bal_frac, s.arm_sp_frac, s.arm_bal_integ,
                 s.bl_pos, s.br_pos, s.bl_vel, s.br_vel,
                 s.arm_l, s.arm_r, s.arm_l_tgt, s.arm_r_tgt,
                 s.meas_drift, s.meas_vel, s.flags);
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

    Serial.println("# === BALANCE CONFIG ===");
    Serial.printf("# inner_kp=%.4f\n", (float)_kp);
    Serial.printf("# inner_kd=%.4f\n", (float)_kd);
    Serial.printf("# arm_bal_kp=%.4f\n", _arm_bal_kp);
    Serial.printf("# arm_bal_ki=%.4f\n", _arm_bal_ki);
    Serial.printf("# arm_bal_kd=%.4f\n", _arm_bal_kd);
    Serial.printf("# arm_bal_max_frac=%.4f\n", _arm_bal_max_frac);
    Serial.printf("# arm_bal_frac_rate=%.4f\n", BALANCE_ARM_BAL_FRAC_RATE);
    Serial.printf("# arm_bal_early_frac_rate=%.4f\n", BALANCE_ARM_BAL_EARLY_FRAC_RATE);
    Serial.printf("# arm_bal_motor_speed=%.2f\n", BALANCE_ARM_BAL_MOTOR_SPEED);
    Serial.printf("# arm_bal_deadband=%.2f\n", BALANCE_ARM_BAL_DEADBAND_RAD);
    Serial.printf("# arm_bal_integral_max=%.2f\n", BALANCE_ARM_BAL_INTEGRAL_MAX);
    Serial.printf("# arm_bal_decay=%.4f\n", BALANCE_ARM_BAL_DECAY);
    Serial.printf("# arm_sp_track_rate=%.4f\n", BALANCE_ARM_SP_TRACK_RATE);
    Serial.printf("# arm_center_left=%.4f\n", _arm_center_left);
    Serial.printf("# arm_center_right=%.4f\n", _arm_center_right);
    Serial.printf("# base_sp_fwd=%.2f\n", BALANCE_SETPOINT_ARMS_FWD);
    Serial.printf("# base_sp_tip=%.2f\n", BALANCE_SETPOINT_ARMS_TIP);
    Serial.printf("# base_sp_center=%.2f\n", BALANCE_SETPOINT_ARMS_CENTER);
    Serial.printf("# base_sp_rate_max=%.2f\n", BALANCE_BASE_SP_RATE_MAX);
    Serial.printf("# comp_alpha=%.4f\n", COMPLEMENTARY_ALPHA);
    Serial.printf("# max_drive_speed=%.2f\n", BALANCE_MAX_DRIVE_SPEED);
    Serial.printf("# capture_err=%.2f\n", BALANCE_CAPTURE_ERR_MAX_DEG);
    Serial.printf("# capture_rate=%.2f\n", BALANCE_CAPTURE_RATE_MAX_DPS);
    Serial.printf("# capture_cmd=%.2f\n", BALANCE_CAPTURE_CMD_MAX);
    Serial.printf("# capture_settle_ms=%lu\n", (unsigned long)BALANCE_CAPTURE_SETTLE_MS);
    Serial.printf("# arm_hold_max_ms=%lu\n", (unsigned long)BALANCE_ARM_HOLD_MAX_MS);
    Serial.printf("# arm_return_speed=%.2f\n", BALANCE_ARM_RETURN_SPEED);
    Serial.printf("# balance_loop_hz=%lu\n", (unsigned long)BALANCE_LOOP_HZ);
    Serial.printf("# control_loop_hz=%lu\n", (unsigned long)CONTROL_LOOP_HZ);
    Serial.printf("# pos_gate_err=%.2f\n", BALANCE_POS_GATE_ERR_DEG);

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
    Serial.printf("  PD: Kp=%.4f  Kd=%.4f\n", (float)_kp, (float)_kd);
    Serial.printf("  Arm balance: Kp=%.4f  Ki=%.4f  Kd=%.4f  max_frac=%.2f  frac=%.4f\n",
                  _arm_bal_kp, _arm_bal_ki, _arm_bal_kd, _arm_bal_max_frac, _arm_bal_frac);
    Serial.printf("  Balance loop: %d Hz (Core 0)\n", BALANCE_LOOP_HZ);
    Serial.printf("  Complementary filter: tilt=%.1f  gyro_rate=%.1f\n",
                  (float)_tilt_angle, (float)_gyro_rate);
    Serial.printf("  Max drive speed: %.1f rad/s\n", BALANCE_MAX_DRIVE_SPEED);

    if (_state == BalanceState::Balancing) {
        Serial.printf("  Setpoint: %.2f  (scheduled + trim=%+.2f)\n",
                      (float)_effective_setpoint, _setpoint);
        Serial.printf("  Measured drift: %+.1f rad  Measured vel: %+.1f rad/s\n",
                      _last_meas_drift, _last_meas_vel);
        Serial.printf("  Stuck: %s  Flags: 0x%02X\n", _stuck ? "YES" : "no", _last_flags);
        Serial.printf("  Safety: err_timer=%s  rate_timer=%s  sat_timer=%s\n",
                      _safe_err_timing ? "ACTIVE" : "off",
                      _safe_rate_timing ? "ACTIVE" : "off",
                      _safe_sat_timing ? "ACTIVE" : "off");
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
