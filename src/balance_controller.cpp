#include "balance_controller.h"
#include "balance_math.h"
#include "telemetry_transport.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <cmath>
#include <cstring>
#include <esp_heap_caps.h>

namespace {

static constexpr uint32_t BALANCE_LOG_MAGIC = 0x324C4142;  // "BAL2"
static constexpr uint16_t BALANCE_LOG_SCHEMA_VERSION = 3;

struct BalanceLogFileHeader {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t header_size;
    uint16_t sample_size;
    uint16_t reserved;
    uint32_t sample_count;
    uint32_t start_uptime_ms;
    uint32_t end_uptime_ms;
    uint32_t checksum;
    char build_date[12];
    char build_time[9];
    char test_note[64];
    char end_reason[48];
    BalanceLogConfigSnapshot config;
};

static_assert(sizeof(BalanceSample) == 236,
              "BalanceSample size changed; re-check LittleFS capacity");
static_assert(sizeof(BalanceSample) * BALANCE_LOG_MAX_SAMPLES < 1450000,
              "Balance telemetry no longer fits safely in LittleFS");

static uint16_t clampU16(uint32_t value) {
    return value > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(value);
}

static uint32_t fnv1aUpdate(uint32_t checksum, const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
        checksum *= 16777619u;
    }
    return checksum;
}

static uint32_t sampleChecksum(const BalanceSample* samples, size_t count) {
    return fnv1aUpdate(2166136261u,
                       reinterpret_cast<const uint8_t*>(samples),
                       count * sizeof(BalanceSample));
}

}  // namespace

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
    _log_pending_flush = false;
    _log_flush_in_progress = false;
    _log_start_ms = 0;
    _log_end_ms = 0;
    _last_log_sample_ms = 0;
    _last_log_flush_attempt_ms = 0;
    _marker_count = 0;
    _next_log_note[0] = '\0';
    _log_note[0] = '\0';
    _log_end_reason[0] = '\0';
    _trim_save_pending = false;

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
// Complementary filter + PD balance (fast task, 200Hz)
// ---------------------------------------------------------------------------

void BalanceController::balanceTick(const RawImuData& imu, float dt) {
    const uint32_t now_us = micros();
    const uint32_t age_us = imu.valid ? now_us - imu.sample_us : UINT32_MAX;
    const bool healthy = imu.valid
        && balance_math::fresh(now_us, imu.sample_us, BALANCE_IMU_STALE_US)
        && balance_math::finiteImu(imu.accel_x, imu.accel_y, imu.accel_z,
                                   imu.gyro_x, imu.gyro_y, imu.gyro_z);
    const bool missed_deadline = _filter_initialized
        && uint32_t(now_us - _last_imu_sample_us) > BALANCE_IMU_STALE_US;
    if (!healthy || missed_deadline) {
        _imu_healthy_since_ms = 0;
        if (isControllingDrive()) _inner_fault |= BAL_DIAG_IMU_STALE;
    }
    if (healthy && !_imu_healthy_since_ms) _imu_healthy_since_ms = millis();

    const uint32_t dt_us_32 = (std::isfinite(dt) && dt > 0.0f)
                           ? static_cast<uint32_t>(fminf(dt * 1000000.0f, 65535.0f)) : 65535;
    portENTER_CRITICAL(&_telemetry_mux);
    if (dt_us_32 > _inner_dt_max_us) _inner_dt_max_us = dt_us_32;
    if (_inner_ticks < 0xFF) _inner_ticks++;
    const uint8_t age_ms = age_us / 1000 > 255 ? 255 : age_us / 1000;
    if (age_ms > _imu_age_max_ms) _imu_age_max_ms = age_ms;
    _inner_diag_window |= _inner_fault;
    portEXIT_CRITICAL(&_telemetry_mux);

    // Integrate each sensor sample ONCE. Reusing a frozen gyro at 200Hz
    // fabricates rotation when the sensor or its producer stops updating.
    if (healthy && (!_filter_initialized || imu.sample_us != _last_imu_sample_us)) {
        const float accel_angle = atan2f(imu.accel_y, imu.accel_z) * 57.2957795f;
        const float gyro_raw = imu.gyro_x;
        _last_accel_angle = accel_angle;
        _last_gyro_raw = gyro_raw;
        _last_accel_norm = sqrtf(imu.accel_x * imu.accel_x
                             + imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z);
        if (!_filter_initialized || missed_deadline) {
            _tilt_angle = accel_angle;
            _gyro_rate = gyro_raw;
            _filter_initialized = true;
        } else {
            const float sensor_dt = uint32_t(imu.sample_us - _last_imu_sample_us) * 1e-6f;
            const float alpha = powf(COMPLEMENTARY_ALPHA, sensor_dt * BALANCE_LOOP_HZ);
            const float gyro_alpha = powf(0.92f, sensor_dt * BALANCE_LOOP_HZ);
            _tilt_angle = alpha * (_tilt_angle + gyro_raw * sensor_dt)
                        + (1.0f - alpha) * accel_angle;
            _gyro_rate = (1.0f - gyro_alpha) * gyro_raw + gyro_alpha * _gyro_rate;
        }
        _last_imu_sample_us = imu.sample_us;
    }

    if (_state != BalanceState::Balancing || !_targets_initialized) {
        _last_inner_diag = 0;
        _last_update_age_ms = 0;
        return;
    }

    // Two-stage dead-man for control task stalls. Stage 1: the inner PD keeps
    // balancing on the stale setpoint with reduced wheel authority. Fresh
    // sensor data is still required; reduced authority does not guarantee
    // stability. Stage 2: a long stall
    // means no safety monitors and no RC control; stop the wheels.
    uint32_t update_age = millis() - _last_update_ms;
    _last_update_age_ms = clampU16(update_age);
    if (update_age > BALANCE_DEADMAN_HARD_MS) _inner_fault |= BAL_DIAG_DEADMAN_HARD;
    if (_inner_fault) {
        _last_inner_diag = _inner_fault;
        portENTER_CRITICAL(&_telemetry_mux);
        _inner_diag_window |= _inner_fault;
        portEXIT_CRITICAL(&_telemetry_mux);
        _last_motor_vel_raw = 0.0f;
        _last_motor_vel = 0.0f;
        _last_cmd_left = 0.0f;
        _last_cmd_right = 0.0f;
        if (_motors->isDriveArmed()) {
            _motors->sendDriveSpeed(MotorRole::BackLeft,  0.0f);
            _motors->sendDriveSpeed(MotorRole::BackRight, 0.0f);
        }
        return;
    }
    float cmd_max = BALANCE_MAX_DRIVE_SPEED;
    uint16_t inner_diag = 0;
    if (update_age > BALANCE_DEADMAN_SOFT_MS) {
        cmd_max = BALANCE_DEADMAN_SOFT_CMD_MAX;
        inner_diag |= BAL_DIAG_DEADMAN_SOFT;
    }

    float angle_err = _effective_setpoint - _tilt_angle;
    // Feed forward the bounded cruising speed so the equilibrium integral need
    // not relearn a speed-dependent angle bias on every start and stop.
    float motor_vel_raw = _pilot_velocity_ff + _kp * angle_err - _kd * _gyro_rate;
    float motor_vel = motor_vel_raw;

    if (motor_vel >  cmd_max) motor_vel =  cmd_max;
    if (motor_vel < -cmd_max) motor_vel = -cmd_max;
    if (motor_vel != motor_vel_raw) {
        inner_diag |= BAL_DIAG_INNER_SATURATED;
        portENTER_CRITICAL(&_telemetry_mux);
        if (_inner_sat_ticks < 0xFF) _inner_sat_ticks++;
        portEXIT_CRITICAL(&_telemetry_mux);
    }

    const auto commands = balance_math::mix(motor_vel, _yaw_corr, cmd_max);
    float cmd_left = commands.left;
    float cmd_right = commands.right;
    if (commands.yaw != (float)_yaw_corr) inner_diag |= BAL_DIAG_YAW_CLAMPED;

    if (_motors->isDriveArmed()) {
        // Back wheels in Speed mode: motor_vel IS the wheel velocity command.
        // Yaw sync (computed at 50Hz on control task) keeps the independent speed
        // loops from integrating into heading drift.
        bool sent_l = _motors->sendDriveSpeed(MotorRole::BackLeft, cmd_left);
        bool sent_r = _motors->sendDriveSpeed(MotorRole::BackRight, cmd_right);
        if (!sent_l || !sent_r) inner_diag |= BAL_DIAG_CAN_TX_FAILED;

        // Front CSP holds are refreshed at 50 Hz; only rear balancing wheels
        // need new commands every 5 ms.
    }

    _last_angle_err = angle_err;
    _last_motor_vel_raw = motor_vel_raw;
    _last_motor_vel = motor_vel;
    _last_cmd_left = cmd_left;
    _last_cmd_right = cmd_right;
    _last_inner_diag = inner_diag;
    portENTER_CRITICAL(&_telemetry_mux);
    _inner_diag_window |= inner_diag;
    portEXIT_CRITICAL(&_telemetry_mux);
}

// ---------------------------------------------------------------------------
// State transitions (control task only)
// ---------------------------------------------------------------------------

bool BalanceController::readyToStart() const {
    if (!_motors || !_arms || !_settings || _state != BalanceState::Idle
        || !_motors->isDriveArmed() || !_motors->isArmArmed()
        || !_arms->getCalibration().calibrated || !_filter_initialized
        || !std::isfinite(_settings->settings.balance_trim)
        || !std::isfinite((float)_kp) || _kp <= 0 || _kp > 5
        || !std::isfinite((float)_kd) || _kd < 0 || _kd > 1
        || !std::isfinite(_drift_vel_kp) || _drift_vel_kp < 0 || _drift_vel_kp > 0.5f
        || !std::isfinite(_vel_sp_kp) || _vel_sp_kp < 0 || _vel_sp_kp > 5
        || !std::isfinite(_vel_sp_ki) || _vel_sp_ki < 0 || _vel_sp_ki > 2
        || !_imu_healthy_since_ms
        || millis() - _imu_healthy_since_ms < BALANCE_IMU_READY_MS
        || !balance_math::fresh(micros(), _last_imu_sample_us, BALANCE_IMU_STALE_US)) return false;
    for (int i = 0; i < _motors->motorCount(); ++i) {
        const auto& m = _motors->getMotor(i);
        if (!m.online || !m.enabled || m.has_fault || m.errors
            || millis() - m.last_feedback_ms > BALANCE_FEEDBACK_STALE_MS) return false;
    }
    return true;
}

bool BalanceController::armsAtGoal(float left, float right) const {
    const auto& l = _motors->getMotor(MotorRole::ArmLeft);
    const auto& r = _motors->getMotor(MotorRole::ArmRight);
    return l.online && r.online && !l.errors && !r.errors
        && millis() - l.last_feedback_ms <= BALANCE_FEEDBACK_STALE_MS
        && millis() - r.last_feedback_ms <= BALANCE_FEEDBACK_STALE_MS
        && fabsf(l.position - left) <= BALANCE_ARM_REACHED_RAD
        && fabsf(r.position - right) <= BALANCE_ARM_REACHED_RAD;
}

void BalanceController::enterTippingUp() {
    if (!readyToStart()) {
        Serial.println("[Balance] Tip-up REFUSED: check calibration, armed motors, faults and fresh IMU/feedback");
        return;
    }
    if (!_log_buf || _log_pending_flush || _log_flush_in_progress || _trim_save_pending) {
        Serial.println("[Balance] Tip-up REFUSED: telemetry is not ready (wait for idle save)");
        return;
    }

    // Switch the back wheels CSP -> Speed BEFORE anything moves. The robot
    // is static on all fours, so the blocking delays and verified param
    // writes (0.4-7.3s of control task stall in every logged run when this
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

    if (!startLog()) {
        _state = BalanceState::Idle;
        _targets_initialized = false;
        exitSpeedMode();
        _arms->clearOverride();
        Serial.println("[Balance] Tip-up REFUSED: telemetry start failed");
    }
}

void BalanceController::enterBalancing(float current_roll) {
    _state = BalanceState::Balancing;
    _targets_initialized = false;   // gate fast task until Speed mode is ready

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
    _qualified_trim_ms = 0;
    _trim_calm_since_ms = 0;
    _capture_stable   = false;
    _capture_stable_start_ms = 0;

    _startup_detector.reset();
    _startup_recovery.reset();
    _recoil_unwind.reset();
    _pilot.reset();
    _pilot_input_valid = false;
    _pilot_velocity_ff = 0;
    _hold_drift = 0.0f;
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
    _targets_initialized = true;    // fast task may now send speed commands

    Serial.printf("[Balance] BALANCING @200Hz (Speed mode)  Kp=%.3f Kd=%.4f  tilt=%.1f  base=%.1f  capture=%.2f  eff=%.1f\n",
                  (float)_kp, (float)_kd, (float)_tilt_angle,
                  scheduled_sp, _engage_capture_shift, (float)_effective_setpoint);
}

// Blend the run's converged equilibrium estimate into the persisted trim.
// Must be called while still in Balancing state, before state cleanup. The
// flash write itself is deferred to serviceLog() after the robot is idle.
void BalanceController::persistLearnedTrim() {
    if (!_settings) return;
    if (_state != BalanceState::Balancing || !_ramp_complete) return;
    if (millis() - _ramp_complete_ms < BALANCE_TRIM_SAVE_MIN_MS) return;

    // The run's full equilibrium estimate = trim baked in at engage + the
    // capture-measured curve shift + the residual the integrator learned.
    // Save a recently qualified calm estimate, never the final shove/fall.
    if (!_qualified_trim_ms || millis() - _qualified_trim_ms > 10000) return;
    float learned = _qualified_trim;
    float old_trim = _settings->settings.balance_trim;
    float blended = old_trim + BALANCE_TRIM_BLEND * (learned - old_trim);
    if (fabsf(blended - old_trim) < BALANCE_TRIM_SAVE_DELTA_DEG) return;

    _settings->settings.balance_trim = blended;
    _pending_trim_old = old_trim;
    _pending_trim_value = blended;
    _pending_trim_learned = learned;
    _trim_save_pending = true;
    Serial.printf("[Balance] Equilibrium trim queued: %.2f deg (was %.2f, this run %.2f)\n",
                  blended, old_trim, learned);
}

void BalanceController::enterReturningArms(const char* reason) {
    persistLearnedTrim();
    stopLog(reason);  // freeze run time before any blocking mode restoration
    _state = BalanceState::ReturningArms;
    _targets_initialized = false;
    _arm_return_start_ms = millis();
    exitSpeedMode();

    _arm_left_goal  = _arms->getForwardLeft();
    _arm_right_goal = _arms->getForwardRight();
    _arm_ramp_speed = BALANCE_ARM_RETURN_SPEED;

    stopLog(reason);

    Serial.println("[Balance] RETURNING ARMS to forward reference");
}

// Zero the wheels and restore CSP position mode. Idempotent -- safe to call
// from any exit path, in any order, any number of times. Must be called with
// _targets_initialized already false (or _state != Balancing) so fast task is no
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
    if (!readyToStart()) {
        Serial.println("[Balance] Cannot engage -- require idle, calibration, armed healthy motors and fresh IMU");
        return;
    }
    float tilt = _filter_initialized ? (float)_tilt_angle : 0.0f;
    if (!startLog()) {
        Serial.println("[Balance] FORCE ENGAGE refused: telemetry is not ready");
        return;
    }
    Serial.printf("[Balance] FORCE ENGAGE at tilt=%.1f\n", tilt);
    _arms_returning = false;
    _arms_reached_tip = true;
    enterBalancing(tilt);
}

void BalanceController::hardAbort(const char* reason) {
    BalanceState prev = _state;
    if (prev == BalanceState::Idle) return;

    // A run that balanced long enough has a converged equilibrium estimate
    // even if it ended in a fall -- keep the knowledge.
    persistLearnedTrim();

    stopLog(reason);
    _state = BalanceState::Idle;
    _targets_initialized = false;
    exitSpeedMode();
    if (_arms) _arms->clearOverride();

    const char* prev_str = (prev == BalanceState::Balancing) ? "BALANCE" :
                           (prev == BalanceState::TippingUp) ? "TIP_UP" : "RET_ARMS";
    Serial.printf("[Balance] HARD ABORT: %s (was %s)\n", reason, prev_str);
}

void BalanceController::disengage(const char* reason) {
    BalanceState prev = _state;

    if (prev == BalanceState::Idle || prev == BalanceState::ReturningArms) {
        _state = BalanceState::Idle;
        _targets_initialized = false;
        if (_arms) {
            _arms->clearOverride();
        }
        stopLog(reason);
        return;
    }

    enterReturningArms(reason);
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
// State machine + outer loops (control task, 50Hz)
// ---------------------------------------------------------------------------

void BalanceController::update(float roll_deg, float roll_rate_dps,
                                bool ch7_active, bool ch11_edge, float dt,
                                float pilot_forward, float pilot_turn, bool pilot_valid) {
    if (!_motors || !_arms) return;
    {
        uint32_t now0 = millis();
        if (_last_update_ms != 0 && now0 - _last_update_ms > 200) {
            _loop_wake_ms = now0;   // just woke from a control task stall
        }
        _last_update_ms = now0;
    }

    float tilt = _filter_initialized ? (float)_tilt_angle : roll_deg;
    float rate = _filter_initialized ? (float)_gyro_rate : roll_rate_dps;

    if (_logging && (millis() - _log_start_ms >= BALANCE_LOG_DURATION_MS)) {
        stopLog("duration_limit");
    }

    // This also runs if the fast task is blocked inside a sensor read.
    // A stopped producer cannot diagnose itself until it wakes.
    if (isControllingDrive()
        && !balance_math::fresh(micros(), _last_imu_sample_us, BALANCE_IMU_STALE_US)) {
        _inner_fault |= BAL_DIAG_IMU_STALE;
    }
    if (isControllingDrive() && _inner_fault) {
        _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
        logSample(tilt, rate);
        hardAbort((_inner_fault & BAL_DIAG_IMU_STALE) ? "IMU stale or invalid" : "control deadman hard stop");
        return;
    }

    if (!ch7_active) {
        if (_state != BalanceState::Idle) {
            if (_state == BalanceState::ReturningArms) {
                _state = BalanceState::Idle;
                _targets_initialized = false;
                if (_arms) _arms->clearOverride();
                stopLog("balance_switch_off");
            } else {
                disengage("balance_switch_off");
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
        bool arms_done = (arm_err_l < 0.05f && arm_err_r < 0.05f)
                      && armsAtGoal(_arm_tip_left_goal, _arm_tip_right_goal);
        if (now_tip - _log_start_ms > BALANCE_TIP_TIMEOUT_MS) {
            _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
            logSample(tilt, rate);
            hardAbort("tip-up timeout / arm tracking");
            return;
        }

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
        _last_outer_diag = 0;

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
            _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
            logSample(tilt, rate);
            hardAbort("tilt out of range (fallen)");
            return;
        }

        // Stale wheel feedback = CAN bus trouble. Balancing blind in Speed
        // mode is how wheels end up spinning into a fall -- abort instead.
        // (Healthy feedback cadence is ~160 ms per motor from the scan cycle.)
        // Grace window after a control task stall: feedback timestamps are updated
        // by control task itself, so a loop stall is indistinguishable from dead
        // CAN until processFeedback has had a moment to refresh (a 449ms
        // stall triggered a spurious abort mid-recovery, run 222223).
        bool stale_grace = (now - _loop_wake_ms) < 500 && _loop_wake_ms != 0;
        if (stale_grace) _last_outer_diag |= BAL_DIAG_FEEDBACK_GRACE;
        uint32_t fb_age_l = now - _motors->getMotor(MotorRole::BackLeft).last_feedback_ms;
        uint32_t fb_age_r = now - _motors->getMotor(MotorRole::BackRight).last_feedback_ms;
        if (!stale_grace
            && (fb_age_l > BALANCE_FEEDBACK_STALE_MS || fb_age_r > BALANCE_FEEDBACK_STALE_MS)) {
            Serial.printf("[Balance] SAFETY: wheel feedback stale (L=%lums R=%lums)\n",
                          (unsigned long)fb_age_l, (unsigned long)fb_age_r);
            _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
            logSample(tilt, rate);
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
                _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
                logSample(tilt, rate);
                disengage("sustained_angle_error");
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
                _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
                logSample(tilt, rate);
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
                _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
                logSample(tilt, rate);
                disengage("sustained_command_saturation");
                return;
            }
        } else {
            _safe_sat_timing = false;
        }

        if (fabsf(_effective_setpoint - tilt) > BALANCE_BAILOUT_THRESHOLD_DEG) {
            Serial.printf("[Balance] BAILOUT  tilt=%.1f sp=%.1f\n", tilt, (float)_effective_setpoint);
            _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
            logSample(tilt, rate);
            disengage("bailout_angle_error");
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
        _last_capture_shift = capture_shift;

        float raw_base_sp = clampf(scheduled_sp + _engage_trim + _run_curve_shift
                                   + capture_shift,
                                   BALANCE_SETPOINT_MIN,
                                   BALANCE_SETPOINT_MAX);
        _last_raw_base_sp = raw_base_sp;

        if (BALANCE_BASE_SP_RATE_MAX > 0.0f) {
            float ramp_rate = BALANCE_BASE_SP_RATE_MAX;
            // Once recovery owns wheel motion, retain the normal 4 deg/s
            // bound without slowing the base behind the moving equilibrium.
            if (!_ramp_complete && !_startup_recovery.triggered()) {
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
        _last_base_sp = base_effective_setpoint;

        if (!_ramp_complete && _arms_returned
            && fabsf(_smoothed_base_sp - raw_base_sp) < 0.1f) {
            _ramp_complete = true;
            _ramp_complete_ms = now;
            // A detected runaway has already started learning equilibrium.
            // Keep that correction through handoff; resetting it would remove
            // the catch just as the arms arrive. Ordinary starts retain the
            // previous zero-integral handoff and original position reference.
            if (!_startup_recovery.triggered()) _vel_sp_integral = 0.0f;
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
            _arm_return_start_ms = now;
            // The forward reference IS the standing top-dead-center pose --
            // the return delivers the assist's neutral stance directly.
            _arm_left_goal  = _arms->getForwardLeft();
            _arm_right_goal = _arms->getForwardRight();

            // Re-anchor the curve to the settled tip stance. This is a quiet
            // pose measurement, not proof of free balance: the arms may still
            // support the body. The tip-to-forward schedule remains a separate
            // physical estimate. Bound TOTAL trim before removing the stored
            // component; a relative +/-6 clamp biased the Sept 14 capture when
            // the old trim was +2.31 and the newly measured correction was -4.
            if (capture_settled) {
                float scheduled_now = computeScheduledSetpoint();
                _run_curve_shift = balance_math::captureCurveShift(
                    tilt, scheduled_now, _engage_trim, BALANCE_SP_OFFSET_MAX_DEG);
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
            if (crisis) _last_outer_diag |= BAL_DIAG_RAMP_CRISIS;
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
                if (arm_err_l < 0.08f && arm_err_r < 0.08f
                    && armsAtGoal(_arm_left_goal, _arm_right_goal)) {
                    _arms_returned = true;
                    Serial.printf("[Balance] Arms returned (drift=%.2f, base_sp=%.1f)\n",
                                  meas_drift, _smoothed_base_sp);
                }
            }
        }

        if (_arms_returning && !_arms_returned
            && now - _arm_return_start_ms > BALANCE_RETURN_TIMEOUT_MS) {
            _last_outer_diag |= BAL_DIAG_SAFETY_EXIT;
            logSample(tilt, rate);
            hardAbort("arm return timeout / tracking");
            return;
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
        // Normal learning is gated by angle error; confirmed startup runaway
        // uses the bounded early-learning path below.
        // ---------------------------------------------------------------
        float pos_gate = 1.0f - clampf(fabsf(_effective_setpoint - tilt)
                                       / BALANCE_POS_GATE_ERR_DEG,
                                       0.0f, 1.0f);
        _last_pos_gate = pos_gate;

        // Velocity damping runs from ENGAGE. Confirm sustained outward motion
        // after the measured arms start returning, using fresh wheel feedback.
        // Recovery starts learning now, while normal learning is still gated.
        _filtered_wheel_vel += BALANCE_VEL_FILTER_ALPHA * (meas_vel - _filtered_wheel_vel);

        const bool recovery_feedback_fresh =
            fb_age_l <= BALANCE_START_RECOVERY_FEEDBACK_MS
            && fb_age_r <= BALANCE_START_RECOVERY_FEEDBACK_MS;
        const balance_math::RunawayConfig recovery_detector_config = {
            BALANCE_START_RECOVERY_SPEED, BALANCE_START_RECOVERY_FORCE_SPEED,
            BALANCE_START_RECOVERY_ACCEL, BALANCE_START_RECOVERY_ACCEL_TAU,
            BALANCE_START_RECOVERY_CONFIRM_MS
        };
        if (!_startup_recovery.triggered()) {
            const bool eligible = _arms_returning && !_ramp_complete
                && computeArmFraction() <= BALANCE_START_RECOVERY_TIP_MAX
                && recovery_feedback_fresh;
            const float direction = _startup_detector.update(
                eligible, _filtered_wheel_vel, dt, recovery_detector_config);
            if (direction && _startup_recovery.start(now)) {
                Serial.printf("[Balance] Early roll recovery: v=%.2f, drift=%.2f, direction=%.0f\n",
                              _filtered_wheel_vel, meas_drift, direction);
            }
        }
        const bool recovery_calm = _ramp_complete && recovery_feedback_fresh
            && fabsf(_filtered_wheel_vel) < BALANCE_START_RECOVERY_CALM_VEL
            && fabsf(rate) < BALANCE_START_RECOVERY_CALM_RATE
            && fabsf(_effective_setpoint - tilt) < BALANCE_START_RECOVERY_CALM_ERR;
        if (_startup_recovery.settle(recovery_calm, dt, BALANCE_START_RECOVERY_CALM_MS)) {
            // Hold where recovery settled; do not immediately request a trip
            // back through all the stand-up travel. Log odometry from ENGAGE
            // unchanged so this cannot hide displacement in the evidence.
            _hold_drift = meas_drift;
            _last_outer_diag |= BAL_DIAG_RECOVERY_SETTLED;
            Serial.printf("[Balance] Early roll recovery settled: hold drift=%.2f, integral=%.2f\n",
                          _hold_drift, _vel_sp_integral);
        }
        const bool recovering = _startup_recovery.active();
        const bool recovery_boost = _startup_recovery.boosting(
            now, _ramp_complete, BALANCE_START_RECOVERY_BOOST_MS);
        if (recovering) _last_outer_diag |= BAL_DIAG_START_RECOVERY;
        if (recovery_boost) _last_outer_diag |= BAL_DIAG_RECOVERY_BOOST;

        const balance_math::PilotConfig pilot_config = {
            BALANCE_PILOT_DEADBAND, BALANCE_PILOT_MAX_VEL, BALANCE_PILOT_MAX_TURN,
            BALANCE_PILOT_ACCEL, BALANCE_PILOT_DECEL, BALANCE_PILOT_TURN_ACCEL,
            BALANCE_PILOT_READY_MS, BALANCE_PILOT_STOP_MS
        };
        _pilot_input_valid = pilot_valid;
        const bool pilot_allowed = pilot_valid && _ramp_complete && !recovering
            && recovery_feedback_fresh && _motors->isDriveArmed() && _motors->isArmArmed()
            && eff_err < BALANCE_PILOT_PAUSE_ERR && fabsf(rate) < BALANCE_PILOT_PAUSE_RATE;
        const float differential_velocity = (_motors->getMotor(MotorRole::BackLeft).velocity
                                            - _motors->getMotor(MotorRole::BackRight).velocity) * .5f;
        const bool pilot_calm = recovery_calm
            && fabsf(_filtered_wheel_vel) < BALANCE_PILOT_STOP_SPEED
            && fabsf(differential_velocity) < BALANCE_PILOT_STOP_TURN;
        const bool pilot_was_ready = _pilot.ready();
        const bool pilot_was_moving = _pilot.moving();
        _pilot.update(pilot_allowed, pilot_calm, pilot_forward, pilot_turn, dt, pilot_config);
        _pilot_velocity_ff = _pilot.moving() ? _pilot.velocity() : 0;
        if (_pilot.ready() && !pilot_was_ready)
            Serial.println("[Balance] Standing drive READY: CH1 steer, CH2 speed");
        // Move the eventual hold point with the robot while driving/braking.
        // Raw odometry still uses the original engagement position in the log.
        if (_pilot.moving() || pilot_was_moving) _hold_drift = meas_drift;

        float target_vel = 0.0f;
        if (recovering) {
            // Arrest wheel motion before requesting position correction.
        } else if (_pilot.moving()) {
            target_vel = _pilot.velocity();
        } else if (_ramp_complete) {
            target_vel = clampf(-_drift_vel_kp * (meas_drift - _hold_drift),
                                -BALANCE_DRIFT_MAX_VEL, BALANCE_DRIFT_MAX_VEL);
        } else {
            // Early position P: oppose standup drift as it develops (low
            // gain, origin at engage) instead of repaying it all after the
            // ramp. With no detected runaway, learning stays ramp-gated; this biases the
            // velocity damper's target, it cannot wind up.
            target_vel = clampf(-BALANCE_RAMP_DRIFT_KP * meas_drift,
                                -BALANCE_RAMP_DRIFT_MAX_VEL, BALANCE_RAMP_DRIFT_MAX_VEL);
        }
        _last_target_vel = target_vel;
        float vel_err = _filtered_wheel_vel - target_vel;
        _last_vel_err = vel_err;

        const balance_math::RecoilConfig recoil_config = {
            BALANCE_RECOIL_ENTER_SPEED, BALANCE_RECOIL_EXIT_SPEED,
            BALANCE_RECOIL_CONFIRM_MS, BALANCE_RECOIL_BLEND_MS, BALANCE_RECOIL_MULTIPLIER
        };
        const float unwind_multiplier = _recoil_unwind.update(
            recovering && _ramp_complete && recovery_feedback_fresh,
            vel_err, _vel_sp_integral, dt, recoil_config);

        if (recovering) {
            // One integrator, bounded gain/time/angle/rate. The initial catch
            // is not suppressed by angle-error gating; normal P damping still
            // respects that gate. End the boost at 800 ms or ramp completion.
            const float ki = recovery_boost ? BALANCE_START_RECOVERY_KI : _vel_sp_ki;
            const auto next = balance_math::recoveryIntegral(
                _vel_sp_integral, vel_err, ki, dt, _sp_offset,
                BALANCE_START_RECOVERY_LIMIT_DEG, BALANCE_START_RECOVERY_RATE_DPS,
                recovery_feedback_fresh, unwind_multiplier);
            _vel_sp_integral = next.value;
            if (next.limited) _last_outer_diag |= BAL_DIAG_RECOVERY_LIMIT;
        } else if (_ramp_complete) {
            // Glide detection: tracking the setpoint but persistently moving
            // means the equilibrium estimate is wrong -- learn faster. Only
            // when CALM: a push/tap recovery also has large vel error, but
            // boosting there corrupts the equilibrium estimate mid-recovery.
            float ki = _vel_sp_ki;
            bool calm = fabsf(rate) < BALANCE_GLIDE_RATE_MAX_DPS
                     && fabsf((float)_last_motor_vel) < BALANCE_GLIDE_CMD_MAX;
            if (!_pilot.moving() && calm && fabsf(vel_err) > BALANCE_GLIDE_VEL_ERR) {
                ki *= BALANCE_GLIDE_KI_BOOST;
                _last_outer_diag |= BAL_DIAG_GLIDE_BOOST;
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
        _last_vel_p_term = p_term;

        // High-velocity shed: near the wheel speed ceiling, braking-by-lean
        // self-defeats (more lean = more acceleration = saturation = crash,
        // runs 155640/164837). Fade the P authority out and accept the
        // displacement; the position loop brings it home afterward.
        float shed = 1.0f - clampf((fabsf(_filtered_wheel_vel) - BALANCE_SHED_VEL_START)
                                   / (BALANCE_SHED_VEL_FULL - BALANCE_SHED_VEL_START),
                                   0.0f, 1.0f);
        _last_shed = shed;

        // During the standup ramp the damper gets a much tighter clamp: the
        // robot is chasing the rising equilibrium from BELOW, where raising
        // the setpoint commands more velocity instead of braking -- the
        // damper was +4.6 of the +4.9 deg peak setpoint error behind the
        // +8..+10 rad standup tows (runs 231458/233710).
        float off_max = _ramp_complete ? BALANCE_SP_OFFSET_MAX_DEG
                                       : BALANCE_RAMP_SP_OFFSET_MAX_DEG;
        // Confirmed recovery needs room to learn before the ramp finishes;
        // it has its own bounded integral and retains the output slew limit.
        if (recovering) off_max = BALANCE_START_RECOVERY_LIMIT_DEG;
        float sp_offset_unclamped = (p_term * pos_gate * shed) + _vel_sp_integral;
        float sp_offset_target = sp_offset_unclamped;
        sp_offset_target = clampf(sp_offset_target, -off_max, off_max);
        if (sp_offset_target != sp_offset_unclamped) {
            _last_outer_diag |= BAL_DIAG_SP_CLAMPED;
            if (recovering) _last_outer_diag |= BAL_DIAG_RECOVERY_LIMIT;
        }
        _last_sp_offset_target = sp_offset_target;
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
        _last_arm_demand = 0.0f;
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
            _last_arm_demand = demand;

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
                // Exit to COOLDOWN only when the robot is actually quiet.
                // A single noisy sample below the demand threshold at the
                // exact moment the arms crossed the neutral band parked them
                // in COOLDOWN mid-event -- the forward recoil of the big
                // backward push then got wheels-only and ran away
                // (run 231458, t=44.5: demand blipped <0.05 while dev
                // crossed -0.10, handoff never fired).
                if (near_neutral && fabsf(demand) < 0.05f
                        && _arm_calm_ms > 150.0f) {
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
                _last_outer_diag |= BAL_DIAG_ARM_EMERGENCY;
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
        if (_pilot.captureHeading()) _yaw_lock_diff = meas_bl - meas_br;
        float yaw_diff = (meas_bl - meas_br) - _yaw_lock_diff;
        // mix() subtracts yaw from left and adds it to right. Negate pilot
        // steering so positive CH1 matches ground drive (left faster).
        float yaw_corr_raw = _pilot.turning() ? -_pilot.turn()
                            : BALANCE_YAW_SYNC_KP * yaw_diff * 0.5f;
        _yaw_corr = clampf(yaw_corr_raw,
                           -BALANCE_YAW_SYNC_MAX, BALANCE_YAW_SYNC_MAX);
        if ((float)_yaw_corr != yaw_corr_raw) {
            _last_outer_diag |= BAL_DIAG_YAW_CLAMPED;
        }
        _last_yaw_diff = yaw_diff;

        float tip_frac = 0.0f;
        float center_frac = 0.0f;
        armAxisFractions(tip_frac, center_frac);
        _last_arm_tip_frac = tip_frac;

        if (_pilot.moving()) _qualified_trim_ms = 0;
        const bool trim_calm = !_pilot.moving() && _ramp_complete && _arm_stage == 0
            && fabsf(meas_vel) < 0.8f * BALANCE_WHEEL_VELOCITY_SCALE && fabsf(rate) < 4.0f
            && fabsf((float)_last_angle_err) < 1.0f && abs_cmd < 2.0f
            && fabsf(_arm_assist_frac) < 0.05f && !_last_outer_diag && !_last_inner_diag;
        if (trim_calm) {
            if (!_trim_calm_since_ms) _trim_calm_since_ms = now;
            if (now - _trim_calm_since_ms >= 1000) {
                _qualified_trim = clampf(_engage_trim + _run_curve_shift + _vel_sp_integral,
                                         -BALANCE_SP_OFFSET_MAX_DEG, BALANCE_SP_OFFSET_MAX_DEG);
                _qualified_trim_ms = now;
            }
        } else {
            _trim_calm_since_ms = 0;
        }

        _last_flags = (unwind_multiplier > 1 ? 0x01 : 0)
                    | (_safe_err_timing ? 0x02 : 0)
                    | (_safe_rate_timing ? 0x04 : 0)
                    | (_safe_sat_timing ? 0x08 : 0)
                    | (_capture_stable ? 0x10 : 0)
                    | (_arms_returning ? 0x20 : 0)
                    | (_ramp_complete ? 0x40 : 0)
                    | (_startup_recovery.triggered() ? 0x80 : 0);

        const bool front_l_ok = _motors->sendDrivePosition(MotorRole::FrontLeft, _front_left_hold, 0.0f);
        const bool front_r_ok = _motors->sendDrivePosition(MotorRole::FrontRight, _front_right_hold, 0.0f);
        if (!front_l_ok || !front_r_ok) _last_outer_diag |= BAL_DIAG_CAN_TX_FAILED;

        logSample(tilt, rate);
        break;
    }

    case BalanceState::ReturningArms: {
        _arm_left_target  = moveToward(_arm_left_target,  _arm_left_goal,  _arm_ramp_speed, dt);
        _arm_right_target = moveToward(_arm_right_target, _arm_right_goal, _arm_ramp_speed, dt);
        _arms->setOverrideTargets(_arm_left_target, _arm_right_target, _arm_ramp_speed);

        float arm_err_l = fabsf(_arm_left_target - _arm_left_goal);
        float arm_err_r = fabsf(_arm_right_target - _arm_right_goal);
        if (arm_err_l < 0.05f && arm_err_r < 0.05f
            && armsAtGoal(_arm_left_goal, _arm_right_goal)) {
            _state = BalanceState::Idle;
            _arms->clearOverride();
            Serial.println("[Balance] Arms returned -> Idle");
        } else if (millis() - _arm_return_start_ms > BALANCE_RETURN_TIMEOUT_MS) {
            hardAbort("return to idle timeout");
        }
        break;
    }

    } // switch
}

// ---------------------------------------------------------------------------
// Telemetry logging
// ---------------------------------------------------------------------------

bool BalanceController::startLog() {
    if (_logging) return true;
    if (!_log_buf || _log_pending_flush || _log_flush_in_progress || _trim_save_pending) return false;

    _pilot.reset();
    _pilot_input_valid = false;

    _inner_fault = 0;
    _log_count = 0;
    _log_saved = false;
    _log_start_ms = millis();
    _log_end_ms = 0;
    _last_log_sample_ms = 0;
    _last_log_flush_attempt_ms = 0;
    _marker_count = 0;
    _logging = true;
    _log_end_reason[0] = '\0';
    snprintf(_log_note, sizeof(_log_note), "%s", _next_log_note);
    _next_log_note[0] = '\0';

    // Keep tip-up rows self-contained; none of these values owns an actuator
    // until enterBalancing(), where they are initialized again.
    _sp_offset = 0.0f;
    _vel_sp_integral = 0.0f;
    _filtered_wheel_vel = 0.0f;
    _last_target_vel = 0.0f;
    _arm_assist_frac = BALANCE_ARM_ASSIST_BIAS_FRAC;
    _arm_assist_vel = 0.0f;
    _arm_calm_ms = 0.0f;
    _arm_stage = 3;
    _run_curve_shift = 0.0f;
    _yaw_corr = 0.0f;

    _last_angle_err = 0.0f;
    _last_motor_vel_raw = 0.0f;
    _last_motor_vel = 0.0f;
    _last_cmd_left = 0.0f;
    _last_cmd_right = 0.0f;
    _last_meas_drift = 0.0f;
    _last_meas_vel = 0.0f;
    _last_base_sp = 0.0f;
    _last_raw_base_sp = 0.0f;
    _last_capture_shift = 0.0f;
    _last_sp_offset_target = 0.0f;
    _last_vel_err = 0.0f;
    _last_vel_p_term = 0.0f;
    _last_pos_gate = 0.0f;
    _last_shed = 0.0f;
    _last_arm_tip_frac = 0.0f;
    _last_arm_demand = 0.0f;
    _last_yaw_diff = 0.0f;
    _last_inner_diag = 0;
    _last_outer_diag = 0;
    portENTER_CRITICAL(&_telemetry_mux);
    _inner_dt_max_us = 0;
    _inner_ticks = 0;
    _inner_sat_ticks = 0;
    _imu_age_max_ms = 0;
    _inner_diag_window = 0;
    portEXIT_CRITICAL(&_telemetry_mux);
    _last_flags = 0;

    memset(&_log_config, 0, sizeof(_log_config));
    _log_config.inner_kp = _kp;
    _log_config.inner_kd = _kd;
    _log_config.drift_vel_kp = _drift_vel_kp;
    _log_config.drift_max_vel = BALANCE_DRIFT_MAX_VEL;
    _log_config.ramp_drift_kp = BALANCE_RAMP_DRIFT_KP;
    _log_config.ramp_drift_max_vel = BALANCE_RAMP_DRIFT_MAX_VEL;
    _log_config.vel_sp_kp = _vel_sp_kp;
    _log_config.vel_sp_kp_low = BALANCE_VEL_SP_KP_LOW;
    _log_config.vel_sp_knee = BALANCE_VEL_SP_KNEE;
    _log_config.vel_sp_ki = _vel_sp_ki;
    _log_config.sp_offset_max = BALANCE_SP_OFFSET_MAX_DEG;
    _log_config.ramp_sp_offset_max = BALANCE_RAMP_SP_OFFSET_MAX_DEG;
    _log_config.sp_offset_rate = BALANCE_SP_OFFSET_RATE;
    _log_config.vel_filter_alpha = BALANCE_VEL_FILTER_ALPHA;
    _log_config.pos_gate_err = BALANCE_POS_GATE_ERR_DEG;
    _log_config.shed_vel_start = BALANCE_SHED_VEL_START;
    _log_config.shed_vel_full = BALANCE_SHED_VEL_FULL;
    _log_config.stored_trim = _settings ? _settings->settings.balance_trim : 0.0f;
    _log_config.glide_vel_err = BALANCE_GLIDE_VEL_ERR;
    _log_config.glide_ki_boost = BALANCE_GLIDE_KI_BOOST;
    _log_config.speed_acc_rad = BALANCE_SPEED_ACC_RAD;
    _log_config.speed_current_limit = BALANCE_SPEED_CURRENT_LIMIT_A;
    _log_config.base_sp_fwd = BALANCE_SETPOINT_ARMS_FWD;
    _log_config.base_sp_tip = BALANCE_SETPOINT_ARMS_TIP;
    _log_config.base_sp_center = BALANCE_SETPOINT_ARMS_CENTER;
    _log_config.base_sp_rate_max = BALANCE_BASE_SP_RATE_MAX;
    _log_config.ramp_vel_slow = BALANCE_RAMP_VEL_SLOW;
    _log_config.comp_alpha = COMPLEMENTARY_ALPHA;
    _log_config.max_drive_speed = BALANCE_MAX_DRIVE_SPEED;
    _log_config.arm_return_speed = BALANCE_ARM_RETURN_SPEED;
    _log_config.arm_assist_thresh = BALANCE_ARM_ASSIST_THRESH;
    _log_config.arm_assist_gain = BALANCE_ARM_ASSIST_GAIN;
    _log_config.arm_range_pos = BALANCE_ARM_ASSIST_RANGE_POS;
    _log_config.arm_range_neg = BALANCE_ARM_ASSIST_RANGE_NEG;
    _log_config.arm_tau_in = BALANCE_ARM_ASSIST_TAU_IN;
    _log_config.arm_tau_out = BALANCE_ARM_ASSIST_TAU_OUT;
    _log_config.arm_emergency_cmd_frac = BALANCE_ARM_EMERGENCY_CMD_FRAC;
    _log_config.yaw_sync_kp = BALANCE_YAW_SYNC_KP;
    _log_config.yaw_sync_max = BALANCE_YAW_SYNC_MAX;
    _log_config.capture_settle_ms = BALANCE_CAPTURE_SETTLE_MS;
    _log_config.arm_hold_max_ms = BALANCE_ARM_HOLD_MAX_MS;
    _log_config.log_duration_ms = BALANCE_LOG_DURATION_MS;
    _log_config.balance_loop_hz = BALANCE_LOOP_HZ;
    _log_config.control_loop_hz = CONTROL_LOOP_HZ;
    _log_config.reserved[0] = static_cast<uint8_t>(BALANCE_START_RECOVERY_SPEED * 10 + .5f);
    _log_config.reserved[1] = static_cast<uint8_t>(BALANCE_START_RECOVERY_FORCE_SPEED * 10 + .5f);
    _log_config.reserved[2] = BALANCE_START_RECOVERY_CONFIRM_MS / 10;
    _log_config.curve_len = (BALANCE_SP_CURVE_LEN < BALANCE_LOG_MAX_CURVE_POINTS)
                          ? BALANCE_SP_CURVE_LEN : BALANCE_LOG_MAX_CURVE_POINTS;
    for (int i = 0; i < _log_config.curve_len; i++) {
        _log_config.curve_frac[i] = BALANCE_SP_CURVE[i].arm_frac;
        _log_config.curve_sp[i] = BALANCE_SP_CURVE[i].setpoint_deg;
    }

    Serial.printf("[Balance] Telemetry v%u started (capacity %.1fs, note='%s')\n",
                  BALANCE_LOG_SCHEMA_VERSION,
                  BALANCE_LOG_MAX_SAMPLES / (float)CONTROL_LOOP_HZ,
                  _log_note[0] ? _log_note : "none");
    return true;
}

void BalanceController::logSample(float roll_deg, float roll_rate_dps) {
    if (!_logging || !_log_buf) return;
    if (_log_count >= BALANCE_LOG_MAX_SAMPLES) {
        stopLog("buffer_full");
        return;
    }

    uint32_t now = millis();
    const MotorState& bl = _motors->getMotor(MotorRole::BackLeft);
    const MotorState& br = _motors->getMotor(MotorRole::BackRight);
    const MotorState& al = _motors->getMotor(MotorRole::ArmLeft);
    const MotorState& ar = _motors->getMotor(MotorRole::ArmRight);
    uint16_t inner_dt_max_us;
    uint8_t inner_ticks;
    uint8_t inner_sat_ticks;
    uint8_t imu_age_ms;
    uint16_t inner_diag;
    portENTER_CRITICAL(&_telemetry_mux);
    inner_dt_max_us = _inner_dt_max_us;
    inner_ticks = _inner_ticks;
    inner_sat_ticks = _inner_sat_ticks;
    imu_age_ms = _imu_age_max_ms;
    inner_diag = _inner_diag_window;
    _imu_age_max_ms = 0;
    _inner_diag_window = 0;
    _inner_dt_max_us = 0;
    _inner_ticks = 0;
    _inner_sat_ticks = 0;
    portEXIT_CRITICAL(&_telemetry_mux);

    BalanceSample& s = _log_buf[_log_count];
    s.t_ms           = now - _log_start_ms;
    s.sample_dt_ms   = _last_log_sample_ms ? clampU16(now - _last_log_sample_ms) : 0;
    s.inner_dt_max_us = inner_dt_max_us;
    s.feedback_age_l_ms = clampU16(now - bl.last_feedback_ms);
    s.feedback_age_r_ms = clampU16(now - br.last_feedback_ms);
    s.update_age_ms  = _last_update_age_ms;
    s.marker         = _marker_count;
    s.diag_flags     = inner_diag | _last_inner_diag | _last_outer_diag;
    s.state          = static_cast<uint8_t>(_state);
    s.flags          = _last_flags;
    s.arm_stage      = _arm_stage;
    s.inner_ticks    = inner_ticks;
    s.inner_sat_ticks = inner_sat_ticks;
    s.imu_age_ms     = imu_age_ms;
    s.roll           = roll_deg;
    s.roll_rate      = roll_rate_dps;
    s.accel_angle    = _last_accel_angle;
    s.gyro_raw       = _last_gyro_raw;
    s.accel_norm     = _last_accel_norm;
    s.setpoint       = (_state == BalanceState::Balancing) ? (float)_effective_setpoint : 0.0f;
    s.angle_err      = _last_angle_err;
    s.base_sp        = _last_base_sp;
    s.raw_base_sp    = _last_raw_base_sp;
    s.capture_shift  = _last_capture_shift;
    s.run_curve_shift = _run_curve_shift;
    s.motor_vel_raw  = _last_motor_vel_raw;
    s.motor_vel      = _last_motor_vel;
    s.cmd_left       = _last_cmd_left;
    s.cmd_right      = _last_cmd_right;
    s.sp_offset      = _sp_offset;
    s.sp_offset_target = _last_sp_offset_target;
    s.target_vel     = _last_target_vel;
    s.filtered_vel   = _filtered_wheel_vel;
    s.vel_err        = _last_vel_err;
    s.vel_integral   = _vel_sp_integral;
    s.vel_p_term     = _last_vel_p_term;
    s.pos_gate       = _last_pos_gate;
    s.shed           = _last_shed;
    s.bl_pos         = bl.position;
    s.br_pos         = br.position;
    s.bl_vel         = bl.velocity;
    s.br_vel         = br.velocity;
    s.bl_torque      = bl.torque;
    s.br_torque      = br.torque;
    s.arm_l          = al.position;
    s.arm_r          = ar.position;
    s.arm_l_tgt      = _arm_left_target;
    s.arm_r_tgt      = _arm_right_target;
    s.arm_l_vel      = al.velocity;
    s.arm_r_vel      = ar.velocity;
    s.arm_l_torque   = al.torque;
    s.arm_r_torque   = ar.torque;
    s.meas_drift     = _last_meas_drift;
    s.meas_vel       = _last_meas_vel;
    float center_frac = 0.0f;
    armAxisFractions(s.arm_tip_frac, center_frac);
    s.arm_assist_frac = _arm_assist_frac;
    s.arm_assist_vel = _arm_assist_vel;
    s.arm_demand     = _last_arm_demand;
    s.arm_calm_ms    = _arm_calm_ms;
    s.yaw_diff       = _last_yaw_diff;
    s.yaw_corr       = _yaw_corr;
    s.bus_voltage    = _motors->getBusVoltage();
    s.total_current  = _motors->getTotalCurrent();
    s.pilot_forward = _pilot.forwardStick();
    s.pilot_steering = _pilot.turnStick();
    s.pilot_turn = _pilot.turn();
    s.pilot_flags = (_pilot.ready() ? 1 : 0) | (_pilot.moving() ? 2 : 0)
                  | (_pilot.turning() ? 4 : 0) | (_pilot_input_valid ? 8 : 0);
    _log_count++;
    _last_log_sample_ms = now;

}

void BalanceController::stopLog(const char* reason) {
    if (!_logging) return;
    _logging = false;
    _log_end_ms = millis();
    snprintf(_log_end_reason, sizeof(_log_end_reason), "%s",
             (reason && reason[0]) ? reason : "unspecified");
    _log_pending_flush = _log_count > 0;

    Serial.printf("[Balance] Telemetry stopped (%lu ms, %d samples, reason=%s) -- save deferred until idle\n",
                  _log_end_ms - _log_start_ms, _log_count, _log_end_reason);
}

void BalanceController::flushLogToFile() {
    if (_log_count == 0 || !_log_buf) return;

    BalanceLogFileHeader header = {};
    header.magic = BALANCE_LOG_MAGIC;
    header.schema_version = BALANCE_LOG_SCHEMA_VERSION;
    header.header_size = sizeof(BalanceLogFileHeader);
    header.sample_size = sizeof(BalanceSample);
    header.reserved = 127;  // prior extensions plus standing drive v1
    header.sample_count = _log_count;
    header.start_uptime_ms = _log_start_ms;
    header.end_uptime_ms = _log_end_ms ? _log_end_ms : millis();
    header.checksum = sampleChecksum(_log_buf, _log_count);
    snprintf(header.build_date, sizeof(header.build_date), "%s", __DATE__);
    snprintf(header.build_time, sizeof(header.build_time), "%s", __TIME__);
    snprintf(header.test_note, sizeof(header.test_note), "%s", _log_note);
    snprintf(header.end_reason, sizeof(header.end_reason), "%s", _log_end_reason);
    header.config = _log_config;

    Serial.printf("[Balance] Saving %d binary samples (%u bytes) to %s...\n",
                  _log_count,
                  (unsigned)(sizeof(header) + _log_count * sizeof(BalanceSample)),
                  BALANCE_LOG_PATH);
    uint32_t start = millis();

    // A legacy CSV can occupy ~350 KB; remove it before allocating the new
    // 1.42 MB binary file or LittleFS may run out of space mid-write.
    LittleFS.remove(BALANCE_LEGACY_LOG_PATH);
    File f = LittleFS.open(BALANCE_LOG_PATH, "w");
    if (!f) {
        Serial.println("[Balance] WARNING: could not open log file for writing");
        return;
    }

    bool ok = f.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header)) == sizeof(header);
    const uint8_t* data = reinterpret_cast<const uint8_t*>(_log_buf);
    size_t remaining = _log_count * sizeof(BalanceSample);
    while (ok && remaining > 0) {
        size_t chunk = remaining > 4096 ? 4096 : remaining;
        size_t written = f.write(data, chunk);
        if (written != chunk) {
            ok = false;
            break;
        }
        data += chunk;
        remaining -= chunk;
    }

    f.flush();
    f.close();
    if (!ok) {
        LittleFS.remove(BALANCE_LOG_PATH);
        Serial.println("[Balance] WARNING: telemetry save was incomplete; will retry while idle");
        return;
    }

    _log_saved = true;
    _log_pending_flush = false;
    Serial.printf("[Balance] Log saved and checksummed (%lu ms)\n", millis() - start);
}

void BalanceController::serviceLog() {
    if ((!_log_pending_flush && !_trim_save_pending)
        || _log_flush_in_progress || _logging || isActive()
        || _motors->isDriveArmed() || _motors->isArmArmed() || _motors->isArming()) return;
    uint32_t now = millis();
    if (_last_log_flush_attempt_ms != 0 && now - _last_log_flush_attempt_ms < 5000) return;
    _last_log_flush_attempt_ms = now;
    _log_flush_in_progress = true;
    if (_trim_save_pending && _settings) {
        if (_settings->save()) {
            Serial.printf("[Balance] Equilibrium trim saved while idle: %.2f deg (was %.2f, run %.2f)\n",
                          _pending_trim_value, _pending_trim_old, _pending_trim_learned);
            _trim_save_pending = false;
        } else {
            Serial.println("[Balance] WARNING: equilibrium trim save failed; will retry while idle");
        }
    }
    if (_log_pending_flush) flushLogToFile();
    _log_flush_in_progress = false;
}

void BalanceController::setLogNote(const char* note) {
    char* destination = _logging ? _log_note : _next_log_note;
    size_t destination_size = _logging ? sizeof(_log_note) : sizeof(_next_log_note);
    snprintf(destination, destination_size, "%s", note ? note : "");
    Serial.printf("[Balance] %s test note: %s\n",
                  _logging ? "Current" : "Next",
                  destination[0] ? destination : "(cleared)");
}

void BalanceController::markEvent() {
    if (_logging && _marker_count < 0xFFFF) _marker_count++;
}

void BalanceController::dumpLog() {
    if (isActive() || _logging || _motors->isDriveArmed() || _motors->isArmArmed() || _motors->isArming()) {
        Serial.println("[Balance] Log dump REFUSED while balance mode is active");
        return;
    }
    _last_log_flush_attempt_ms = 0;  // an explicit download requests an immediate retry
    serviceLog();
    if (_log_pending_flush) {
        Serial.println("[Balance] Log is still pending save; check LittleFS space and retry");
        return;
    }

    TelemetryTransport out;
    File f = LittleFS.open(BALANCE_LOG_PATH, "r");
    if (!f) {
        // Preserve access to a pre-v2 CSV that survived a firmware-only flash.
        File legacy = LittleFS.open(BALANCE_LEGACY_LOG_PATH, "r");
        if (!legacy) {
            out.println("[Balance] No log file found");
            return;
        }
        out.printf("[Balance] --- Log dump (%u bytes, legacy schema) ---\n", legacy.size());
        out.println("# telemetry_schema=1");
        out.println("# legacy_file=1");
        while (legacy.available()) out.write(legacy.read());
        legacy.close();
        out.println("[Balance] --- End of log ---");
        return;
    }

    BalanceLogFileHeader header = {};
    if (f.read(reinterpret_cast<uint8_t*>(&header), sizeof(header)) != sizeof(header)
        || header.magic != BALANCE_LOG_MAGIC
        || !balance_log::supported(header.schema_version, header.sample_size)
        || header.header_size != sizeof(BalanceLogFileHeader)
        || header.sample_count > BALANCE_LOG_MAX_SAMPLES) {
        out.println("[Balance] Telemetry file is invalid or uses an unsupported schema");
        f.close();
        return;
    }

    size_t expected_size = sizeof(header) + header.sample_count * header.sample_size;
    uint32_t checksum = 2166136261u;
    uint8_t verify_buf[512];
    size_t remaining = header.sample_count * header.sample_size;
    while (remaining > 0) {
        size_t chunk = remaining > sizeof(verify_buf) ? sizeof(verify_buf) : remaining;
        int got = f.read(verify_buf, chunk);
        if (got != (int)chunk) break;
        checksum = fnv1aUpdate(checksum, verify_buf, chunk);
        remaining -= chunk;
    }
    bool checksum_valid = remaining == 0 && checksum == header.checksum
                       && f.size() == expected_size;

    out.printf("[Balance] --- Log dump (%u bytes, %lu samples) ---\n",
                  f.size(), (unsigned long)header.sample_count);

    out.resetChecksum();
    out.println("# === BALANCE CONFIG ===");
    out.println("# transport_checksum=fnv1a32");
    out.printf("# telemetry_features=%u\n", header.reserved);
    if (header.reserved & 64) {
        out.println("# standing_drive=ch1_ch2_velocity_feedforward_v1");
        out.println("# standing_drive_limits=velocity_rad_s:1 turn_rad_s:0.5 accel:0.5 decel:0.75 turn_accel:0.75");
        out.println("# standing_drive_gates=deadband:0.06 neutral_calm_ms:400 stop_calm_ms:400 rc_fresh_ms:100");
        out.println("# standing_drive_flags=ready:1 moving_or_braking:2 turning:4 fresh_input:8");
    }
    if (header.reserved & 32) {
        // Describe the stored algorithm version, not the downloader's settings.
        out.println("# startup_recoil=confirmed_unwind_v1");
        out.println("# startup_recoil_multiplier=2 enter_rad_s=0.35 exit_rad_s=0.15 confirm_ms=60 blend_ms=120");
        out.println("# startup_recoil_active_flag=0x01");
    }
    if (header.reserved & 16) {
        out.println("# startup_recovery=wheel_velocity_learning_v1");
        out.printf("# startup_recovery_speed=%.1f\n", header.config.reserved[0] * .1f);
        out.printf("# startup_recovery_force_speed=%.1f\n", header.config.reserved[1] * .1f);
        out.printf("# startup_recovery_confirm_ms=%u\n", header.config.reserved[2] * 10);
        // Versioned constants: these describe v1, never the running config of
        // firmware that happens to download an older stored log.
        out.println("# startup_recovery_ki=1.0 boost_ms=800 integral_rate_dps=6 limit_deg=6");
        out.println("# startup_recovery_hold=settled_position calm_ms=400");
    }
    if (header.reserved & 8) {
        out.println("# capture_trim_bounds=absolute");
    }
    if (header.reserved & 2) {
        out.println("# wheel_feedback_velocity_range_rad_s=50");
        out.println("# wheel_feedback_torque_range_nm=5.5");
    }
    if (header.reserved & 4) {
        out.println("# can_receive_hz=200");
        out.println("# front_hold_hz=50");
    }
    out.printf("# telemetry_schema=%u\n", header.schema_version);
    out.printf("# sample_size_bytes=%u\n", header.sample_size);
    out.printf("# sample_count=%lu\n", (unsigned long)header.sample_count);
    out.printf("# checksum=0x%08lX\n", (unsigned long)header.checksum);
    out.printf("# checksum_valid=%d\n", checksum_valid ? 1 : 0);
    out.printf("# build_date=%s\n", header.build_date);
    out.printf("# build_time=%s\n", header.build_time);
    out.printf("# run_start_uptime_ms=%lu\n", (unsigned long)header.start_uptime_ms);
    out.printf("# run_end_uptime_ms=%lu\n", (unsigned long)header.end_uptime_ms);
    out.printf("# run_duration_ms=%lu\n",
                  (unsigned long)(header.end_uptime_ms - header.start_uptime_ms));
    out.printf("# end_reason=%s\n", header.end_reason[0] ? header.end_reason : "unknown");
    out.printf("# test_note=%s\n", header.test_note[0] ? header.test_note : "none");
    out.println("# motor_mode=speed");
    const BalanceLogConfigSnapshot& c = header.config;
    out.printf("# inner_kp=%.4f\n", c.inner_kp);
    out.printf("# inner_kd=%.4f\n", c.inner_kd);
    out.printf("# drift_vel_kp=%.4f\n", c.drift_vel_kp);
    out.printf("# drift_max_vel=%.2f\n", c.drift_max_vel);
    out.printf("# ramp_drift_kp=%.4f\n", c.ramp_drift_kp);
    out.printf("# ramp_drift_max_vel=%.2f\n", c.ramp_drift_max_vel);
    out.printf("# vel_sp_kp=%.4f\n", c.vel_sp_kp);
    out.printf("# vel_sp_kp_low=%.4f\n", c.vel_sp_kp_low);
    out.printf("# vel_sp_knee=%.4f\n", c.vel_sp_knee);
    out.printf("# vel_sp_ki=%.4f\n", c.vel_sp_ki);
    out.printf("# sp_offset_max=%.2f\n", c.sp_offset_max);
    out.printf("# ramp_sp_offset_max=%.2f\n", c.ramp_sp_offset_max);
    out.printf("# sp_offset_rate=%.2f\n", c.sp_offset_rate);
    out.printf("# vel_filter_alpha=%.4f\n", c.vel_filter_alpha);
    out.printf("# stored_trim=%.4f\n", c.stored_trim);
    out.printf("# glide_vel_err=%.2f\n", c.glide_vel_err);
    out.printf("# glide_ki_boost=%.2f\n", c.glide_ki_boost);
    out.printf("# speed_acc_rad=%.2f\n", c.speed_acc_rad);
    out.printf("# speed_current_limit=%.2f\n", c.speed_current_limit);
    for (int i = 0; i < c.curve_len; i++) {
        out.printf("# sp_curve_%d=%.2f:%.2f\n", i,
                      c.curve_frac[i], c.curve_sp[i]);
    }
    out.printf("# base_sp_fwd=%.2f\n", c.base_sp_fwd);
    out.printf("# base_sp_tip=%.2f\n", c.base_sp_tip);
    out.printf("# base_sp_center=%.2f\n", c.base_sp_center);
    out.printf("# base_sp_rate_max=%.2f\n", c.base_sp_rate_max);
    out.printf("# ramp_vel_slow=%.2f\n", c.ramp_vel_slow);
    out.printf("# comp_alpha=%.4f\n", c.comp_alpha);
    out.printf("# max_drive_speed=%.2f\n", c.max_drive_speed);
    out.printf("# capture_settle_ms=%lu\n", (unsigned long)c.capture_settle_ms);
    out.printf("# arm_hold_max_ms=%lu\n", (unsigned long)c.arm_hold_max_ms);
    out.printf("# arm_return_speed=%.2f\n", c.arm_return_speed);
    out.printf("# arm_assist_thresh=%.2f\n", c.arm_assist_thresh);
    out.printf("# arm_assist_gain=%.2f\n", c.arm_assist_gain);
    out.printf("# arm_range_pos=%.2f\n", c.arm_range_pos);
    out.printf("# arm_range_neg=%.2f\n", c.arm_range_neg);
    out.printf("# arm_tau_in=%.3f\n", c.arm_tau_in);
    out.printf("# arm_tau_out=%.3f\n", c.arm_tau_out);
    out.printf("# arm_emergency_cmd_frac=%.2f\n", c.arm_emergency_cmd_frac);
    out.printf("# yaw_sync_kp=%.2f\n", c.yaw_sync_kp);
    out.printf("# yaw_sync_max=%.2f\n", c.yaw_sync_max);
    out.printf("# log_duration_ms=%lu\n", (unsigned long)c.log_duration_ms);
    out.printf("# balance_loop_hz=%u\n", c.balance_loop_hz);
    out.printf("# control_loop_hz=%u\n", c.control_loop_hz);
    out.printf("# pos_gate_err=%.2f\n", c.pos_gate_err);
    out.printf("# shed_vel_start=%.2f\n", c.shed_vel_start);
    out.printf("# shed_vel_full=%.2f\n", c.shed_vel_full);

    if (!checksum_valid) { f.close(); return; }
    f.seek(sizeof(header));
    out.println("t_ms,sample_dt_ms,inner_dt_max_us,inner_ticks,inner_sat_ticks,state,flags,diag_flags,marker,"
                   "roll,roll_rate,accel_angle,gyro_raw,accel_norm,setpoint,angle_err,base_sp,raw_base_sp,"
                   "capture_shift,run_curve_shift,motor_vel_raw,motor_vel,cmd_left,cmd_right,sp_offset,"
                   "sp_offset_target,target_vel,filtered_vel,vel_err,vel_integral,vel_p_term,pos_gate,shed,"
                   "bl_pos,br_pos,bl_vel,br_vel,bl_torque,br_torque,feedback_age_l_ms,feedback_age_r_ms,"
                   "arm_l,arm_r,arm_l_tgt,arm_r_tgt,arm_l_vel,arm_r_vel,arm_l_torque,arm_r_torque,"
                   "arm_tip_frac,arm_assist_frac,arm_assist_vel,arm_demand,arm_calm_ms,arm_stage,"
                   "meas_drift,meas_vel,yaw_diff,yaw_corr,update_age_ms,bus_voltage,total_current,imu_age_ms,"
                   "pilot_forward,pilot_steering,pilot_turn,pilot_flags");
    for (uint32_t i = 0; i < header.sample_count && !out.failed(); i++) {
        BalanceSample s = {};
        if (f.read(reinterpret_cast<uint8_t*>(&s), header.sample_size) != header.sample_size) break;
        out.printf("%lu,%u,%u,%u,%u,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.4f,",
                      (unsigned long)s.t_ms, s.sample_dt_ms, s.inner_dt_max_us,
                      s.inner_ticks, s.inner_sat_ticks, s.state, s.flags,
                      s.diag_flags, s.marker, s.roll, s.roll_rate,
                      s.accel_angle, s.gyro_raw, s.accel_norm);
        out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
                      s.setpoint, s.angle_err, s.base_sp, s.raw_base_sp,
                      s.capture_shift, s.run_curve_shift, s.motor_vel_raw,
                      s.motor_vel, s.cmd_left, s.cmd_right, s.sp_offset,
                      s.sp_offset_target, s.target_vel, s.filtered_vel,
                      s.vel_err, s.vel_integral, s.vel_p_term, s.pos_gate, s.shed);
        out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,",
                      s.bl_pos, s.br_pos, s.bl_vel, s.br_vel,
                      s.bl_torque, s.br_torque,
                      s.feedback_age_l_ms, s.feedback_age_r_ms);
        out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.1f,%u,%.3f,%.3f,%.3f,%.3f,%u,%.2f,%.2f,",
                      s.arm_l, s.arm_r, s.arm_l_tgt, s.arm_r_tgt,
                      s.arm_l_vel, s.arm_r_vel, s.arm_l_torque, s.arm_r_torque,
                      s.arm_tip_frac, s.arm_assist_frac, s.arm_assist_vel,
                      s.arm_demand, s.arm_calm_ms, s.arm_stage,
                      s.meas_drift, s.meas_vel, s.yaw_diff, s.yaw_corr,
                      s.update_age_ms, s.bus_voltage, s.total_current);
        if (header.reserved & 1) out.printf("%u", s.imu_age_ms);
        if (header.schema_version >= 3 && (header.reserved & 64))
            out.printf(",%.4f,%.4f,%.4f,%lu", s.pilot_forward, s.pilot_steering,
                       s.pilot_turn, (unsigned long)s.pilot_flags);
        else out.printf(",,,,"); // historical files have unknown pilot fields
        out.println();
    }
    f.close();
    if (out.failed()) return;
    const uint32_t transfer_checksum = out.checksum();
    out.printf("# transport_fnv1a=0x%08lX\n", (unsigned long)transfer_checksum);
    out.println("[Balance] --- End of log ---");
}

void BalanceController::clearLog() {
    if (isActive() || _logging || _log_flush_in_progress
        || _motors->isDriveArmed() || _motors->isArmArmed() || _motors->isArming()) {
        Serial.println("[Balance] Log clear REFUSED: end balance and disarm drive AND arms first");
        return;
    }
    _log_count = 0;
    _log_saved = false;
    _log_pending_flush = false;
    LittleFS.remove(BALANCE_LOG_PATH);
    LittleFS.remove(BALANCE_LEGACY_LOG_PATH);
    Serial.println("[Balance] Log file deleted");
}

bool BalanceController::hasLog() const {
    if (_log_count > 0) return true;
    return LittleFS.exists(BALANCE_LOG_PATH) || LittleFS.exists(BALANCE_LEGACY_LOG_PATH);
}

size_t BalanceController::logSize() const {
    if (LittleFS.exists(BALANCE_LOG_PATH)) {
        File f = LittleFS.open(BALANCE_LOG_PATH, "r");
        if (!f) return 0;
        size_t sz = f.size();
        f.close();
        return sz;
    }
    if (LittleFS.exists(BALANCE_LEGACY_LOG_PATH)) {
        File f = LittleFS.open(BALANCE_LEGACY_LOG_PATH, "r");
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
    Serial.printf("  Balance loop: %d Hz (control core, direct IMU)\n", BALANCE_LOOP_HZ);
    Serial.printf("  IMU age: %lu us, healthy since: %lu ms, latched fault: 0x%04X\n",
                  (unsigned long)(micros() - _last_imu_sample_us),
                  (unsigned long)_imu_healthy_since_ms, (unsigned)_inner_fault);
    Serial.printf("  Complementary filter: tilt=%.1f  gyro_rate=%.1f\n",
                  (float)_tilt_angle, (float)_gyro_rate);
    Serial.printf("  Max drive speed: %.1f rad/s\n", BALANCE_MAX_DRIVE_SPEED);
    Serial.printf("  Standing drive: CH1 steer / CH2 speed, limit %.2f rad/s, turn %.2f rad/s, %s\n",
                  BALANCE_PILOT_MAX_VEL, BALANCE_PILOT_MAX_TURN,
                  _state == BalanceState::Balancing && _pilot.ready() ? "READY" : "waiting for centered calm balance");

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

    // Status is allowed during a run: never open/stat LittleFS from that
    // path (including missing-file error logging in the VFS implementation).
    if (isActive() || _logging || _log_pending_flush) {
        Serial.printf("  Log buffer: %d samples, schema v%u (file inspection deferred)\n",
                      _log_count, BALANCE_LOG_SCHEMA_VERSION);
    } else if (hasLog()) {
        Serial.printf("  Log: %u bytes, %d buffered samples, schema v%u\n",
                      logSize(), _log_count, BALANCE_LOG_SCHEMA_VERSION);
    } else {
        Serial.println("  Log file: none");
    }
    Serial.printf("  Logging: %s  pending save: %s  note: %s\n",
                  _logging ? "ACTIVE" : "off",
                  isLogPendingFlush() ? "YES" : "no",
                  (_logging ? _log_note : _next_log_note)[0]
                      ? (_logging ? _log_note : _next_log_note) : "none");
    Serial.println("======================");
}
