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

// Decompose measured arm deltas onto the two calibrated motion axes:
//   tip axis    T = (TIP_LEFT, TIP_RIGHT)          -- both arms same direction
//   center axis C = (center_left, center_right)    -- mirrored (opposite signs)
// Solving d = tip_frac*T + center_frac*C keeps the two setpoint effects
// independent: tip-up motion doesn't bleed into the center term and
// arm-assist (center) motion doesn't bleed into the tip curve.
void BalanceController::armAxisFractions(float& tip_frac, float& center_frac) const {
    tip_frac = 1.0f;
    center_frac = 0.0f;
    if (!_motors || !_arms) return;

    float d_l = _motors->getMotor(MotorRole::ArmLeft).position - _arms->getForwardLeft();
    float d_r = _motors->getMotor(MotorRole::ArmRight).position - _arms->getForwardRight();

    const float T_l = BALANCE_ARM_TIP_LEFT;
    const float T_r = BALANCE_ARM_TIP_RIGHT;
    const ArmCalibration& cal = _arms->getCalibration();
    float C_l = cal.center_left;
    float C_r = cal.center_right;

    float det = T_l * C_r - T_r * C_l;
    if (fabsf(det) < 0.5f) {
        // Degenerate/missing center calibration: tip-only projection
        float tip_avg = (T_l + T_r) * 0.5f;
        tip_frac = clampf((d_l + d_r) * 0.5f / tip_avg, 0.0f, 1.0f);
        return;
    }

    tip_frac    = clampf((d_l * C_r - d_r * C_l) / det, 0.0f, 1.0f);
    // Center axis runs negative for arms forward of vertical (assist
    // braking backward motion) and positive toward the calibrated center
    // pose (arms swinging back).
    center_frac = clampf((T_l * d_r - T_r * d_l) / det, -0.5f, 1.2f);
}

float BalanceController::computeArmFraction() const {
    float tip_frac, center_frac;
    armAxisFractions(tip_frac, center_frac);
    return tip_frac;
}

float BalanceController::computeScheduledSetpoint() const {
    // Piecewise-linear interpolation over the fitted arm-position ->
    // balance-point curve (the CG shift saturates early in arm travel, so a
    // two-point lerp is wrong through most of the range), plus a linear
    // center-axis term for arm-assist excursions.
    float tip_frac, center_frac;
    armAxisFractions(tip_frac, center_frac);

    float base = BALANCE_SP_CURVE[BALANCE_SP_CURVE_LEN - 1].setpoint_deg;
    if (tip_frac <= BALANCE_SP_CURVE[0].arm_frac) {
        base = BALANCE_SP_CURVE[0].setpoint_deg;
    } else {
        for (int i = 1; i < BALANCE_SP_CURVE_LEN; i++) {
            const BalanceSpAnchor& lo = BALANCE_SP_CURVE[i - 1];
            const BalanceSpAnchor& hi = BALANCE_SP_CURVE[i];
            if (tip_frac <= hi.arm_frac) {
                float span = hi.arm_frac - lo.arm_frac;
                float t = (span > 0.0001f) ? (tip_frac - lo.arm_frac) / span : 0.0f;
                base = lo.setpoint_deg + t * (hi.setpoint_deg - lo.setpoint_deg);
                break;
            }
        }
    }

    base += center_frac * (BALANCE_SETPOINT_ARMS_CENTER - BALANCE_SETPOINT_ARMS_FWD);
    return base;
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
    (void)dt;

    // Two-stage dead-man for Core 1 stalls. Stage 1: the inner PD keeps
    // balancing on the stale setpoint but with clamped wheel authority (no
    // runaway possible, balance survives sub-second stalls -- run 173619
    // fell because v1 stopped the wheels outright). Stage 2: a long stall
    // means no safety monitors and no RC control; stop the wheels.
    uint32_t update_age = millis() - _last_update_ms;
    if (update_age > BALANCE_DEADMAN_HARD_MS) {
        if (_motors->isDriveArmed()) {
            _motors->sendDriveSpeed(MotorRole::BackLeft,  0.0f);
            _motors->sendDriveSpeed(MotorRole::BackRight, 0.0f);
        }
        return;
    }
    float cmd_max = BALANCE_MAX_DRIVE_SPEED;
    if (update_age > BALANCE_DEADMAN_SOFT_MS) {
        cmd_max = BALANCE_DEADMAN_SOFT_CMD_MAX;
    }

    float angle_err = _effective_setpoint - _tilt_angle;
    float motor_vel = _kp * angle_err - _kd * _gyro_rate;

    if (motor_vel >  cmd_max) motor_vel =  cmd_max;
    if (motor_vel < -cmd_max) motor_vel = -cmd_max;

    if (_motors->isDriveArmed()) {
        // Back wheels in Speed mode: motor_vel IS the wheel velocity command.
        // Yaw sync (computed at 50Hz on Core 1) keeps the independent speed
        // loops from integrating into heading drift.
        float yaw_corr = _yaw_corr;
        _motors->sendDriveSpeed(MotorRole::BackLeft,  motor_vel - yaw_corr);
        _motors->sendDriveSpeed(MotorRole::BackRight, motor_vel + yaw_corr);

        // Front wheels stay in CSP holding their engage position.
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
    // Switch the back wheels CSP -> Speed BEFORE anything moves. The robot
    // is static on all fours, so the blocking delays and verified param
    // writes (0.4-7.3s of Core 1 stall in every logged run when this
    // happened at engage) cost nothing here -- and a failed switch aborts
    // a standup that never started instead of hard-aborting a robot that
    // is already up on its arms. At 0 rad/s the velocity servo holds the
    // wheels against the arm push (integral action).
    bool ok_l = _motors->setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed,
                                         BALANCE_SPEED_ACC_RAD, BALANCE_SPEED_CURRENT_LIMIT_A);
    bool ok_r = _motors->setDriveRunMode(MotorRole::BackRight, RobstrideRunMode::Speed,
                                         BALANCE_SPEED_ACC_RAD, BALANCE_SPEED_CURRENT_LIMIT_A);
    if (!ok_l || !ok_r) {
        _motors->setDriveRunMode(MotorRole::BackLeft,  RobstrideRunMode::CSP);
        _motors->setDriveRunMode(MotorRole::BackRight, RobstrideRunMode::CSP);
        Serial.println("[Balance] Tip-up REFUSED: speed mode switch failed");
        return;
    }
    _speed_mode_active = true;
    _last_speed_refresh_ms = millis();

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

    Serial.printf("[Balance] TIPPING UP (wheels pre-switched to Speed)  arm goal: L=%.2f R=%.2f\n",
                  _arm_left_goal, _arm_right_goal);

    startLog();
}

void BalanceController::enterBalancing(float current_roll) {
    _state = BalanceState::Balancing;
    _targets_initialized = false;   // gate Core 0 until Speed mode is ready

    float scheduled_sp = computeScheduledSetpoint();
    _engage_arm_frac = computeArmFraction();
    // Stored equilibrium trim becomes part of the BASE setpoint so the
    // 3 deg/s base ramp delivers it smoothly -- never as a step.
    _engage_trim = clampf(_settings ? _settings->settings.balance_trim : 0.0f,
                          -BALANCE_SP_OFFSET_MAX_DEG, BALANCE_SP_OFFSET_MAX_DEG);
    _engage_capture_shift = clampf(current_roll - (scheduled_sp + _engage_trim),
                                   -BALANCE_CAPTURE_SHIFT_MAX_DEG,
                                   BALANCE_CAPTURE_SHIFT_MAX_DEG);
    float initial_base = BALANCE_USE_CAPTURE_SHIFT
                       ? scheduled_sp + _engage_trim + _engage_capture_shift
                       : scheduled_sp + _engage_trim;
    _smoothed_base_sp = initial_base;
    _effective_setpoint = clampf(initial_base, BALANCE_SETPOINT_MIN, BALANCE_SETPOINT_MAX);

    _wheel_start_pos = (_motors->getMotor(MotorRole::BackLeft).position
                      + _motors->getMotor(MotorRole::BackRight).position) * 0.5f;

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

    _vel_sp_integral    = 0.0f;
    _sp_offset          = 0.0f;
    _filtered_wheel_vel = 0.0f;
    _last_target_vel    = 0.0f;

    _yaw_corr = 0.0f;
    _yaw_lock_diff = _motors->getMotor(MotorRole::BackLeft).position
                   - _motors->getMotor(MotorRole::BackRight).position;

    _arm_assist_frac = BALANCE_ARM_ASSIST_BIAS_FRAC;
    _arm_assist_vel  = 0.0f;
    _arm_stage       = 3;      // COOLDOWN: standup transients must not deploy arms
    _arm_sign        = 0.0f;
    _arm_calm_ms     = 0.0f;
    _run_curve_shift = 0.0f;
    const ArmCalibration& cal = _arms->getCalibration();
    _arm_center_left  = cal.center_left;
    _arm_center_right = cal.center_right;

    // Normally the wheels were already switched to Speed mode at the start
    // of tip-up (setDriveRunMode is a fast no-op when the mode matches).
    // The force-engage path (no tip-up) still pays the blocking switch here
    // -- acceptable for a hand-supported testing feature.
    bool ok_l = _motors->setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed,
                                         BALANCE_SPEED_ACC_RAD, BALANCE_SPEED_CURRENT_LIMIT_A);
    bool ok_r = _motors->setDriveRunMode(MotorRole::BackRight, RobstrideRunMode::Speed,
                                         BALANCE_SPEED_ACC_RAD, BALANCE_SPEED_CURRENT_LIMIT_A);
    if (!ok_l || !ok_r) {
        // Revert any motor that DID switch so nothing is left in Speed mode
        // (setDriveRunMode is idempotent for motors already in CSP).
        _motors->setDriveRunMode(MotorRole::BackLeft,  RobstrideRunMode::CSP);
        _motors->setDriveRunMode(MotorRole::BackRight, RobstrideRunMode::CSP);
        hardAbort("speed mode switch failed");
        return;
    }
    _speed_mode_active = true;
    _targets_initialized = true;    // Core 0 may now send speed commands

    Serial.printf("[Balance] BALANCING @200Hz (Speed mode)  Kp=%.3f Kd=%.4f  tilt=%.1f  base=%.1f  capture=%.2f  eff=%.1f\n",
                  (float)_kp, (float)_kd, (float)_tilt_angle,
                  scheduled_sp, _engage_capture_shift, (float)_effective_setpoint);
}

// Blend the run's converged equilibrium estimate into the persisted trim.
// Must be called while still in Balancing state, before state cleanup.
void BalanceController::persistLearnedTrim() {
    if (!_settings) return;
    if (_state != BalanceState::Balancing || !_ramp_complete) return;
    if (millis() - _ramp_complete_ms < BALANCE_TRIM_SAVE_MIN_MS) return;

    // The run's full equilibrium estimate = trim baked in at engage + the
    // capture-measured curve shift + the residual the integrator learned.
    float learned = _engage_trim + _run_curve_shift + _vel_sp_integral;
    float old_trim = _settings->settings.balance_trim;
    float blended = old_trim + BALANCE_TRIM_BLEND * (learned - old_trim);
    if (fabsf(blended - old_trim) < BALANCE_TRIM_SAVE_DELTA_DEG) return;

    _settings->settings.balance_trim = blended;
    _settings->save();
    Serial.printf("[Balance] Equilibrium trim learned: %.2f deg (was %.2f, this run %.2f)\n",
                  blended, old_trim, learned);
}

void BalanceController::enterReturningArms() {
    persistLearnedTrim();
    _state = BalanceState::ReturningArms;
    _targets_initialized = false;
    exitSpeedMode();

    _arm_left_goal  = _arms->getForwardLeft();
    _arm_right_goal = _arms->getForwardRight();
    _arm_ramp_speed = BALANCE_ARM_RETURN_SPEED;

    stopLog();

    Serial.println("[Balance] RETURNING ARMS to forward reference");
}

// Zero the wheels and restore CSP position mode. Idempotent -- safe to call
// from any exit path, in any order, any number of times. Must be called with
// _targets_initialized already false (or _state != Balancing) so Core 0 is no
// longer sending speed commands.
void BalanceController::exitSpeedMode() {
    if (!_speed_mode_active) return;
    _speed_mode_active = false;

    if (!_motors) return;

    // Zero speed FIRST: a Speed-mode motor keeps spinning at its last
    // command, so this must precede any mode/stop transitions.
    _motors->sendDriveSpeed(MotorRole::BackLeft,  0.0f);
    _motors->sendDriveSpeed(MotorRole::BackRight, 0.0f);

    _motors->setDriveRunMode(MotorRole::BackLeft,  RobstrideRunMode::CSP);
    _motors->setDriveRunMode(MotorRole::BackRight, RobstrideRunMode::CSP);

    Serial.println("[Balance] Back wheels restored to CSP mode");
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

    // A run that balanced long enough has a converged equilibrium estimate
    // even if it ended in a fall -- keep the knowledge.
    persistLearnedTrim();

    _state = BalanceState::Idle;
    _targets_initialized = false;
    exitSpeedMode();
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
    {
        uint32_t now0 = millis();
        if (_last_update_ms != 0 && now0 - _last_update_ms > 200) {
            _loop_wake_ms = now0;   // just woke from a Core 1 stall
        }
        _last_update_ms = now0;
    }

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

        // The back wheels are already in Speed mode holding 0: refresh the
        // command every 250ms so the motor-side CAN watchdog (~1s) never
        // stops them during the ~9s tip.
        uint32_t now_tip = millis();
        if (now_tip - _last_speed_refresh_ms >= BALANCE_TIPUP_SPEED_REFRESH_MS) {
            _last_speed_refresh_ms = now_tip;
            _motors->sendDriveSpeed(MotorRole::BackLeft,  0.0f);
            _motors->sendDriveSpeed(MotorRole::BackRight, 0.0f);
        }

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

        // Stale wheel feedback = CAN bus trouble. Balancing blind in Speed
        // mode is how wheels end up spinning into a fall -- abort instead.
        // (Healthy feedback cadence is ~160 ms per motor from the scan cycle.)
        // Grace window after a Core 1 stall: feedback timestamps are updated
        // by Core 1 itself, so a loop stall is indistinguishable from dead
        // CAN until processFeedback has had a moment to refresh (a 449ms
        // stall triggered a spurious abort mid-recovery, run 222223).
        bool stale_grace = (now - _loop_wake_ms) < 500 && _loop_wake_ms != 0;
        uint32_t fb_age_l = now - _motors->getMotor(MotorRole::BackLeft).last_feedback_ms;
        uint32_t fb_age_r = now - _motors->getMotor(MotorRole::BackRight).last_feedback_ms;
        if (!stale_grace
            && (fb_age_l > BALANCE_FEEDBACK_STALE_MS || fb_age_r > BALANCE_FEEDBACK_STALE_MS)) {
            Serial.printf("[Balance] SAFETY: wheel feedback stale (L=%lums R=%lums)\n",
                          (unsigned long)fb_age_l, (unsigned long)fb_age_r);
            hardAbort("wheel feedback stale (CAN)");
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
            if (_arms_returned) {
                // Fully handed off -- arm-assist excursions must not
                // re-apply a fraction of the engage capture shift.
                capture_weight = 0.0f;
            } else if (_arms_returning) {
                capture_weight = (_engage_arm_frac > 0.05f)
                               ? clampf(arm_frac / _engage_arm_frac, 0.0f, 1.0f)
                               : 1.0f;
            }
            capture_shift = _engage_capture_shift * capture_weight;
        }

        float raw_base_sp = clampf(scheduled_sp + _engage_trim + _run_curve_shift
                                   + capture_shift,
                                   BALANCE_SETPOINT_MIN,
                                   BALANCE_SETPOINT_MAX);

        if (BALANCE_BASE_SP_RATE_MAX > 0.0f) {
            float ramp_rate = BALANCE_BASE_SP_RATE_MAX;
            if (!_ramp_complete) {
                // The ramp waits for the robot: if the wheels are already
                // moving, slow the setpoint climb instead of towing the
                // robot across the floor (standup surge, runs 164837/171838).
                // Floor at 30% so the ramp always finishes -- a fully paused
                // ramp holds the setpoint below equilibrium, which is itself
                // a source of sustained acceleration.
                float vel_gate = 1.0f - clampf(fabsf(_filtered_wheel_vel)
                                               / BALANCE_RAMP_VEL_SLOW,
                                               0.0f, 1.0f);
                if (vel_gate < 0.3f) vel_gate = 0.3f;
                ramp_rate *= vel_gate;
            }
            _smoothed_base_sp = moveToward(_smoothed_base_sp, raw_base_sp,
                                           ramp_rate, dt);
        } else {
            _smoothed_base_sp = raw_base_sp;
        }
        float base_effective_setpoint = _smoothed_base_sp;

        if (!_ramp_complete && _arms_returned
            && fabsf(_smoothed_base_sp - raw_base_sp) < 0.1f) {
            _ramp_complete = true;
            _ramp_complete_ms = now;
            // Origin stays at ENGAGE: the position loop (active only from
            // now on, so no mid-ramp fighting -- lesson 13 concern) walks
            // the robot back to where it was stood up, undoing the
            // unavoidable standup travel.
            // Stored trim is already inside the base setpoint (delivered by
            // the ramp); the integrator learns only the residual, from zero.
            // _sp_offset and _filtered_wheel_vel are NOT reset: the velocity
            // damper has been running through the ramp and both are already
            // continuous (resets here caused the 154823 setpoint step).
            _vel_sp_integral = 0.0f;
            Serial.printf("[Balance] Ramp complete -- outer cascade active (sp=%.1f, base trim=%.2f)\n",
                          _smoothed_base_sp, _engage_trim);
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
            // The forward reference IS the standing top-dead-center pose --
            // the return delivers the assist's neutral stance directly.
            _arm_left_goal  = _arms->getForwardLeft();
            _arm_right_goal = _arms->getForwardRight();

            // Per-run self-calibration: a SETTLED capture is a genuine
            // equilibrium measurement at the tip stance (err<1 deg, rate<4
            // dps, quiet 400ms). Re-zero the whole curve to it -- the curve
            // SHAPE is stable run-to-run but its absolute level wanders
            // with CG state (battery seat, surface). Replaces the fading
            // capture shift (which measured the same thing, less well, at
            // engage); the 3 deg/s base rate limiter absorbs the tiny
            // instantaneous difference between the two.
            if (capture_settled) {
                float scheduled_now = computeScheduledSetpoint();
                _run_curve_shift = clampf(tilt - (scheduled_now + _engage_trim),
                                          -6.0f, 6.0f);
                _engage_capture_shift = 0.0f;
                Serial.printf("[Balance] Capture-calibrated: tilt=%.2f, curve shift %+.2f\n",
                              tilt, _run_curve_shift);
            }
            Serial.printf("[Balance] Arms beginning return to forward/TDC (%s after %lu ms)\n",
                          capture_settled ? "captured" : "settle timeout",
                          now - _balance_start_ms);
        }

        // ---------------------------------------------------------------
        // Arm return, then arm assist ownership after the ramp completes
        // ---------------------------------------------------------------
        if (_arms_returning && !_ramp_complete) {
            // Crisis pause: don't march the CG during an outright fight
            // (run 093643). NOTE: a tighter "self-pacing" lag gate was tried
            // (run 223630 postmortem) and reverted -- it kept the robot
            // stuck in fragile mid-return stances for seconds (run 224227).
            // At 1.5 rad/s the return is slow enough to track.
            bool crisis = fabsf((float)_last_motor_vel) > 10.0f || fabsf(rate) > 30.0f;
            if (!crisis) {
                // Proportional return: scale per-arm speed by remaining
                // distance so both arms reach forward TOGETHER. Equal-rate
                // return finished the short (right) arm first; the axis
                // decomposition read the imbalance as a center excursion
                // and the equilibrium dipped ~1 deg mid-return for nothing.
                float dist_l = fabsf(_arm_left_goal  - _arm_left_target);
                float dist_r = fabsf(_arm_right_goal - _arm_right_target);
                float longest = (dist_l > dist_r) ? dist_l : dist_r;
                float speed_l = BALANCE_ARM_RETURN_SPEED;
                float speed_r = BALANCE_ARM_RETURN_SPEED;
                if (longest > 0.001f) {
                    speed_l *= dist_l / longest;
                    speed_r *= dist_r / longest;
                }
                _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  speed_l, dt);
                _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, speed_r, dt);
            }

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

        // ---------------------------------------------------------------
        // Outer cascade (Speed mode): position P -> velocity PI
        //
        //   target_vel = clamp(-drift_kp * drift)      return-to-origin
        //   vel_err    = filtered_wheel_vel - target_vel
        //   sp_offset  = kp * vel_err + integral(ki * vel_err)
        //
        // Positive drift (rolled forward) -> negative target_vel -> positive
        // vel_err -> positive sp_offset -> higher setpoint -> lean backward
        // -> the PD drives the wheels backward toward origin. In Speed mode
        // this genuinely translates the robot (the CSP equilibrium wall from
        // lessons 19-23 no longer exists). Single integrator (lesson 18),
        // gated by angle error so the catch phase is untouched.
        // ---------------------------------------------------------------
        float pos_gate = 1.0f - clampf(fabsf(_effective_setpoint - tilt)
                                       / BALANCE_POS_GATE_ERR_DEG,
                                       0.0f, 1.0f);

        // Velocity damper runs from ENGAGE (during the base ramp the PD
        // chases the rising setpoint and can surge away -- +10 rad drift in
        // run 164837). Position return and learning only after the ramp.
        _filtered_wheel_vel += BALANCE_VEL_FILTER_ALPHA * (meas_vel - _filtered_wheel_vel);

        float target_vel = 0.0f;
        if (_ramp_complete) {
            target_vel = clampf(-_drift_vel_kp * meas_drift,
                                -BALANCE_DRIFT_MAX_VEL, BALANCE_DRIFT_MAX_VEL);
        } else {
            // Early position P: oppose standup drift as it develops (low
            // gain, origin at engage) instead of repaying it all after the
            // ramp. The integrator stays ramp-gated -- this only biases the
            // velocity damper's target, it cannot wind up.
            target_vel = clampf(-BALANCE_RAMP_DRIFT_KP * meas_drift,
                                -BALANCE_RAMP_DRIFT_MAX_VEL, BALANCE_RAMP_DRIFT_MAX_VEL);
        }
        _last_target_vel = target_vel;
        float vel_err = _filtered_wheel_vel - target_vel;

        if (_ramp_complete) {
            // Glide detection: tracking the setpoint but persistently moving
            // means the equilibrium estimate is wrong -- learn faster. Only
            // when CALM: a push/tap recovery also has large vel error, but
            // boosting there corrupts the equilibrium estimate mid-recovery.
            float ki = _vel_sp_ki;
            bool calm = fabsf(rate) < BALANCE_GLIDE_RATE_MAX_DPS
                     && fabsf((float)_last_motor_vel) < BALANCE_GLIDE_CMD_MAX;
            if (calm && fabsf(vel_err) > BALANCE_GLIDE_VEL_ERR) {
                ki *= BALANCE_GLIDE_KI_BOOST;
            }

            _vel_sp_integral += ki * vel_err * pos_gate * dt;
            _vel_sp_integral = clampf(_vel_sp_integral,
                                      -BALANCE_SP_OFFSET_MAX_DEG, BALANCE_SP_OFFSET_MAX_DEG);
        }

        // Dual-slope velocity response: gentle below the knee so the
        // setpoint doesn't chase idle velocity ripple (0.3 Hz sway,
        // bal_20260702_164034), full authority above it for taps.
        // During the standup ramp only the low slope applies -- forward
        // glide is REQUIRED to stand up, and the tap-recovery slope whipped
        // the setpoint 1.5 deg past equilibrium (+13 rad tow, run 232626).
        float abs_err = fabsf(vel_err);
        float p_term;
        if (abs_err <= BALANCE_VEL_SP_KNEE || !_ramp_complete) {
            p_term = BALANCE_VEL_SP_KP_LOW * vel_err;
        } else {
            float sign = 1.0f;
            if (vel_err < 0.0f) sign = -1.0f;
            p_term = sign * (BALANCE_VEL_SP_KP_LOW * BALANCE_VEL_SP_KNEE
                             + _vel_sp_kp * (abs_err - BALANCE_VEL_SP_KNEE));
        }

        // Actuator coordination: a deployed arm is already shifting the
        // equilibrium (its center-term lowers/raises the scheduled sp).
        // Scale the wheel P authority down while arms are out so the two
        // corrections don't stack (double-counted into a -10 rad/s
        // backward overcorrection, run 232626).
        float arm_dev = fabsf(_arm_assist_frac - BALANCE_ARM_ASSIST_BIAS_FRAC);
        float arm_share = clampf(arm_dev / BALANCE_ARM_ASSIST_RANGE_POS, 0.0f, 1.0f);
        p_term *= (1.0f - 0.6f * arm_share);

        // High-velocity shed: near the wheel speed ceiling, braking-by-lean
        // self-defeats (more lean = more acceleration = saturation = crash,
        // runs 155640/164837). Fade the P authority out and accept the
        // displacement; the position loop brings it home afterward.
        float shed = 1.0f - clampf((fabsf(_filtered_wheel_vel) - BALANCE_SHED_VEL_START)
                                   / (BALANCE_SHED_VEL_FULL - BALANCE_SHED_VEL_START),
                                   0.0f, 1.0f);

        float sp_offset_target = (p_term * pos_gate * shed) + _vel_sp_integral;
        sp_offset_target = clampf(sp_offset_target,
                                  -BALANCE_SP_OFFSET_MAX_DEG, BALANCE_SP_OFFSET_MAX_DEG);
        // Rate-limited: the setpoint must never step (lesson from the
        // 154823 crash -- a 2.9 deg step pitched the robot over).
        _sp_offset = moveToward(_sp_offset, sp_offset_target,
                                BALANCE_SP_OFFSET_RATE, dt);

        // ---------------------------------------------------------------
        // Arm assist: forward runaways are braked by swinging the arms
        // toward the CENTER pose (the physically-symmetric axis -- the tip
        // deltas are per-arm asymmetric and scaling them scissors the arms
        // apart, run 170714). The balance point drops toward 83 deg, an
        // equilibrium shift needing no wheel acceleration -- exactly the
        // authority the wheels lack near saturation. High threshold + slow
        // release: fires only on genuine disturbances, no idle chatter.
        // ---------------------------------------------------------------
        if (_ramp_complete) {
            // One-shot impulse-and-relax (operator design, run 231038): the
            // arms answer the FIRST hit fast, then relax monotonically to
            // neutral. They never flip sign mid-recovery -- the counter-
            // swing belongs to the wheels. This kills the deploy/counter-
            // deploy flip-flop that kept the robot from settling.
            _arm_assist_vel += clampf(dt / BALANCE_ARM_ASSIST_VEL_TAU, 0.0f, 1.0f)
                             * (vel_err - _arm_assist_vel);
            float excess = 0.0f;
            if (_arm_assist_vel > BALANCE_ARM_ASSIST_THRESH) {
                excess = _arm_assist_vel - BALANCE_ARM_ASSIST_THRESH;
            } else if (_arm_assist_vel < -BALANCE_ARM_ASSIST_THRESH) {
                excess = _arm_assist_vel + BALANCE_ARM_ASSIST_THRESH;
            }
            float demand = clampf(BALANCE_ARM_ASSIST_GAIN * excess,
                                  -BALANCE_ARM_ASSIST_RANGE_NEG,
                                  BALANCE_ARM_ASSIST_RANGE_POS);

            // Engagement state machine. Discriminator between an external
            // bump and self-oscillation: a bump ARRIVES OUT OF CALM; an
            // oscillation never re-establishes calm (0.56 Hz arm flip-flop,
            // 89 reversals, run 095326). One event = at most two swings
            // (push + recoil handoff), then the arms hold neutral until the
            // robot has been genuinely calm -- the wheels-only loop is
            // proven stable and extinguishes any residual oscillation.
            bool calm_now = fabsf(vel_err) < BALANCE_ARM_CALM_VEL
                         && fabsf(rate) < BALANCE_ARM_CALM_RATE;
            if (calm_now) {
                _arm_calm_ms += dt * 1000.0f;
            } else {
                _arm_calm_ms = 0.0f;
            }

            float dev = _arm_assist_frac - BALANCE_ARM_ASSIST_BIAS_FRAC;
            bool near_neutral = fabsf(dev) < 0.10f;
            float dsign = 0.0f;
            if (demand > 0.0f) dsign = 1.0f;
            if (demand < 0.0f) dsign = -1.0f;

            float target = BALANCE_ARM_ASSIST_BIAS_FRAC;
            float tau = BALANCE_ARM_ASSIST_TAU_OUT;

            switch (_arm_stage) {
            case 0:  // READY: armed at neutral, full-speed response available
                if (dsign != 0.0f) {
                    _arm_sign = dsign;
                    _arm_stage = 1;
                    target = BALANCE_ARM_ASSIST_BIAS_FRAC + demand;
                    tau = BALANCE_ARM_ASSIST_TAU_IN;
                }
                break;

            case 1:  // ACTIVE
            case 2:  // HANDOFF (opposite swing of the same event)
                if (dsign == _arm_sign && fabsf(demand) > fabsf(dev)) {
                    // Deepen with the disturbance
                    target = BALANCE_ARM_ASSIST_BIAS_FRAC + demand;
                    tau = BALANCE_ARM_ASSIST_TAU_IN;
                } else if (dsign == -_arm_sign && fabsf(demand) > 0.10f
                           && _arm_stage == 1) {
                    if (near_neutral) {
                        // Recoil handoff: one opposite swing allowed
                        _arm_sign = dsign;
                        _arm_stage = 2;
                        target = BALANCE_ARM_ASSIST_BIAS_FRAC + demand;
                        tau = BALANCE_ARM_ASSIST_TAU_IN;
                    } else {
                        // Clear out fast to make the handoff possible
                        tau = BALANCE_ARM_ASSIST_TAU_IN * 2.0f;
                    }
                }
                // else: relax slow (default target/tau)
                if (near_neutral && fabsf(demand) < 0.05f) {
                    _arm_stage = 3;
                    _arm_calm_ms = 0.0f;
                }
                break;

            default: // COOLDOWN: hold neutral until genuine calm re-arms
                if (_arm_calm_ms > BALANCE_ARM_CALM_MS) {
                    _arm_stage = 0;
                }
                break;
            }

            // EMERGENCY OVERRIDE: wheels railed while velocity error is
            // still large = roll-away in progress. The wheels have nothing
            // left to give, so lifecycle rules don't apply -- throw the
            // arms to their full stop in the braking direction.
            bool wheels_railed = fabsf((float)_last_motor_vel)
                               >= BALANCE_MAX_DRIVE_SPEED * BALANCE_ARM_EMERGENCY_CMD_FRAC;
            if (wheels_railed && fabsf(vel_err) > BALANCE_ARM_ASSIST_THRESH) {
                _arm_sign  = (vel_err > 0.0f) ? 1.0f : -1.0f;
                _arm_stage = 1;   // exits as a normal ACTIVE engagement
                if (vel_err > 0.0f) {
                    target = BALANCE_ARM_ASSIST_BIAS_FRAC + BALANCE_ARM_ASSIST_RANGE_POS;
                } else {
                    target = BALANCE_ARM_ASSIST_BIAS_FRAC - BALANCE_ARM_ASSIST_RANGE_NEG;
                }
                tau = BALANCE_ARM_ASSIST_TAU_IN;
            }

            float alpha = clampf(dt / tau, 0.0f, 1.0f);
            _arm_assist_frac += alpha * (target - _arm_assist_frac);

            float fwd_l = _arms->getForwardLeft();
            float fwd_r = _arms->getForwardRight();
            _arm_left_target  = fwd_l + _arm_assist_frac * _arm_center_left;
            _arm_right_target = fwd_r + _arm_assist_frac * _arm_center_right;
        }

        float arm_speed = 0.0f;
        if (_ramp_complete) {
            arm_speed = BALANCE_ARM_ASSIST_SPEED;
        } else if (_arms_returning) {
            arm_speed = BALANCE_ARM_RETURN_SPEED;
        }
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, arm_speed);

        _effective_setpoint = base_effective_setpoint + _sp_offset;
        _effective_setpoint = clampf(_effective_setpoint, BALANCE_SETPOINT_MIN, BALANCE_SETPOINT_MAX);

        // ---------------------------------------------------------------
        // Yaw sync: hold the L/R wheel position difference at its engage
        // value. In Speed mode the two velocity loops are independent and
        // their tracking errors integrate into heading drift.
        // ---------------------------------------------------------------
        float yaw_diff = (meas_bl - meas_br) - _yaw_lock_diff;
        _yaw_corr = clampf(BALANCE_YAW_SYNC_KP * yaw_diff * 0.5f,
                           -BALANCE_YAW_SYNC_MAX, BALANCE_YAW_SYNC_MAX);

        _last_flags = (_safe_err_timing ? 0x02 : 0)
                    | (_safe_rate_timing ? 0x04 : 0)
                    | (_safe_sat_timing ? 0x08 : 0)
                    | (_capture_stable ? 0x10 : 0)
                    | (_arms_returning ? 0x20 : 0)
                    | (_ramp_complete ? 0x40 : 0);

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
    s.setpoint       = (_state == BalanceState::Balancing) ? (float)_effective_setpoint : 0.0f;
    s.angle_err      = _last_angle_err;
    s.motor_vel      = _last_motor_vel;
    s.sp_offset      = _sp_offset;
    s.target_vel     = _last_target_vel;
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
              "sp_offset,target_vel,"
              "bl_pos,br_pos,bl_vel,br_vel,"
              "arm_l,arm_r,arm_l_tgt,arm_r_tgt,"
              "meas_drift,meas_vel,flags");

    for (int i = 0; i < _log_count; i++) {
        const BalanceSample& s = _log_buf[i];
        f.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,"
                 "%.4f,%.4f,"
                 "%.2f,%.2f,%.2f,%.2f,"
                 "%.3f,%.3f,%.3f,%.3f,"
                 "%.2f,%.2f,%d\n",
                 s.t_ms, s.state,
                 s.roll, s.roll_rate, s.setpoint,
                 s.angle_err, s.motor_vel,
                 s.sp_offset, s.target_vel,
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
    Serial.println("# motor_mode=speed");
    Serial.printf("# inner_kp=%.4f\n", (float)_kp);
    Serial.printf("# inner_kd=%.4f\n", (float)_kd);
    Serial.printf("# drift_vel_kp=%.4f\n", _drift_vel_kp);
    Serial.printf("# drift_max_vel=%.2f\n", BALANCE_DRIFT_MAX_VEL);
    Serial.printf("# ramp_drift_kp=%.4f\n", BALANCE_RAMP_DRIFT_KP);
    Serial.printf("# ramp_drift_max_vel=%.2f\n", BALANCE_RAMP_DRIFT_MAX_VEL);
    Serial.printf("# vel_sp_kp=%.4f\n", _vel_sp_kp);
    Serial.printf("# vel_sp_ki=%.4f\n", _vel_sp_ki);
    Serial.printf("# sp_offset_max=%.2f\n", BALANCE_SP_OFFSET_MAX_DEG);
    Serial.printf("# vel_filter_alpha=%.4f\n", BALANCE_VEL_FILTER_ALPHA);
    Serial.printf("# stored_trim=%.4f\n", _settings ? _settings->settings.balance_trim : 0.0f);
    Serial.printf("# glide_vel_err=%.2f\n", BALANCE_GLIDE_VEL_ERR);
    Serial.printf("# glide_ki_boost=%.2f\n", BALANCE_GLIDE_KI_BOOST);
    Serial.printf("# speed_acc_rad=%.2f\n", BALANCE_SPEED_ACC_RAD);
    Serial.printf("# speed_current_limit=%.2f\n", BALANCE_SPEED_CURRENT_LIMIT_A);
    for (int i = 0; i < BALANCE_SP_CURVE_LEN; i++) {
        Serial.printf("# sp_curve_%d=%.2f:%.2f\n", i,
                      BALANCE_SP_CURVE[i].arm_frac, BALANCE_SP_CURVE[i].setpoint_deg);
    }
    Serial.printf("# base_sp_fwd=%.2f\n", BALANCE_SETPOINT_ARMS_FWD);
    Serial.printf("# base_sp_tip=%.2f\n", BALANCE_SETPOINT_ARMS_TIP);
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
    Serial.printf("  State: %s  (Speed mode %s)\n", getStateString(),
                  _speed_mode_active ? "ACTIVE" : "off");
    Serial.printf("  PD: Kp=%.4f  Kd=%.4f\n", (float)_kp, (float)_kd);
    Serial.printf("  Outer: drift_kp=%.4f  vel_kp=%.4f  vel_ki=%.4f\n",
                  _drift_vel_kp, _vel_sp_kp, _vel_sp_ki);
    Serial.printf("  Cascade: sp_offset=%.3f  integral=%.3f  target_vel=%.3f  filt_vel=%.3f\n",
                  _sp_offset, _vel_sp_integral, _last_target_vel, _filtered_wheel_vel);
    Serial.printf("  Stored trim: %.2f deg (persisted equilibrium estimate)\n",
                  _settings ? _settings->settings.balance_trim : 0.0f);
    Serial.printf("  Balance loop: %d Hz (Core 0)\n", BALANCE_LOOP_HZ);
    Serial.printf("  Complementary filter: tilt=%.1f  gyro_rate=%.1f\n",
                  (float)_tilt_angle, (float)_gyro_rate);
    Serial.printf("  Max drive speed: %.1f rad/s\n", BALANCE_MAX_DRIVE_SPEED);

    if (_state == BalanceState::Balancing) {
        Serial.printf("  Setpoint: %.2f  (base + offset=%+.2f)\n",
                      (float)_effective_setpoint, _sp_offset);
        Serial.printf("  Measured drift: %+.1f rad  Measured vel: %+.1f rad/s\n",
                      _last_meas_drift, _last_meas_vel);
        Serial.printf("  Flags: 0x%02X\n", _last_flags);
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
