#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "motor_manager.h"
#include "arm_controller.h"
#include "settings.h"
#include "config.h"
#include "balance_math.h"
#include "balance_pilot.h"
#include "balance_drive.h"
#include "balance_telemetry.h"

enum class BalanceState : uint8_t {
    Idle           = 0,
    TippingUp      = 1,
    Balancing      = 2,
    ReturningArms  = 3,
};

struct RawImuData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    uint32_t sample_us = 0;  // successful accel + gyro read, never a polling timestamp
    bool valid = false;
};

enum BalanceDiagFlag : uint16_t {
    BAL_DIAG_INNER_SATURATED = 0x0001,
    BAL_DIAG_DEADMAN_SOFT    = 0x0002,
    BAL_DIAG_DEADMAN_HARD    = 0x0004,
    BAL_DIAG_FEEDBACK_GRACE  = 0x0008,
    BAL_DIAG_GLIDE_BOOST     = 0x0010,
    BAL_DIAG_RAMP_CRISIS     = 0x0020,
    BAL_DIAG_ARM_EMERGENCY   = 0x0040,
    BAL_DIAG_SP_CLAMPED      = 0x0080,
    BAL_DIAG_YAW_CLAMPED     = 0x0100,
    BAL_DIAG_SAFETY_EXIT     = 0x0200,
    BAL_DIAG_IMU_STALE       = 0x0400,
    BAL_DIAG_CAN_TX_FAILED   = 0x0800,
    BAL_DIAG_START_RECOVERY  = 0x1000,
    BAL_DIAG_RECOVERY_BOOST  = 0x2000,
    BAL_DIAG_RECOVERY_LIMIT  = 0x4000,
    BAL_DIAG_RECOVERY_SETTLED = 0x8000,
};

class BalanceController {
public:
    void begin(MotorManager* motors, ArmController* arms);
    void setSettingsManager(SettingsManager* mgr) { _settings = mgr; }

    // Called from fast task at 200Hz -- fast PD balance loop
    void balanceTick(const RawImuData& imu, float dt);

    // Called from control task at 50Hz -- state machine, arms, velocity-integrating setpoint
    void update(float roll_deg, float roll_rate_dps,
                bool ch7_active, bool ch11_edge, float dt,
                float pilot_forward = 0, float pilot_turn = 0, bool pilot_valid = false);

    BalanceState getState() const { return _state; }
    bool isActive() const { return _state != BalanceState::Idle; }
    bool isControllingDrive() const {
        return _state == BalanceState::TippingUp || _state == BalanceState::Balancing;
    }

    const char* getStateString() const;

    void forceEngage();
    void hardAbort(const char* reason);

    float getTiltAngle() const { return _tilt_angle; }
    float getGyroRate() const { return _gyro_rate; }

    // Gain setters for serial tuning
    void setKp(float v) { _kp = v; }
    void setKd(float v) { _kd = v; }
    void setDriftVelKp(float v) { _drift_vel_kp = v; }
    void setVelSpKp(float v) { _vel_sp_kp = v; }
    void setVelSpKi(float v) { _vel_sp_ki = v; }

    float getKp() const { return _kp; }
    float getKd() const { return _kd; }
    float getEffectiveSetpoint() const { return _effective_setpoint; }

    // Telemetry log. serviceLog() is called only from the low-priority loop
    // after balance mode is fully idle; it is the only path that writes flash.
    void dumpLog();
    void clearLog();
    void serviceLog();
    void setLogNote(const char* note);
    void markEvent();
    bool hasLog() const;
    size_t logSize() const;
    bool isLogging() const { return _logging; }
    bool isLogPendingFlush() const {
        return _log_pending_flush || _log_flush_in_progress || _trim_save_pending;
    }
    uint32_t getLogStartMs() const { return _log_start_ms; }
    uint32_t getLogEndMs() const { return _log_end_ms; }

    void printStatus();

private:
    MotorManager*    _motors   = nullptr;
    ArmController*   _arms     = nullptr;
    SettingsManager* _settings = nullptr;

    volatile BalanceState _state = BalanceState::Idle;

    // --- Complementary filter (fast task) ---
    volatile float _tilt_angle = 0.0f;
    volatile float _gyro_rate  = 0.0f;
    balance_math::DriveRateFilter _drive_rate_filter; // fast-task owned
    volatile float _drive_gyro_rate = 0.0f;
    volatile float _last_accel_angle = 0.0f;
    volatile float _last_gyro_raw    = 0.0f;
    volatile float _last_accel_norm  = 0.0f;
    bool _filter_initialized = false;
    volatile uint32_t _last_imu_sample_us = 0;
    volatile uint32_t _imu_healthy_since_ms = 0;
    volatile uint16_t _inner_fault = 0;  // cleared only for a deliberate new attempt
    uint8_t _imu_age_max_ms = 0;
    uint16_t _inner_diag_window = 0;

    // --- PD gains (read by fast task, written by control task for tuning) ---
    volatile float _kp = BALANCE_KP;
    volatile float _kd = BALANCE_KD;

    // --- Effective setpoint (written by control task, read by fast task) ---
    volatile float _effective_setpoint = BALANCE_SETPOINT_ARMS_TIP;

    // --- Yaw sync: differential wheel correction (control task writes, fast task reads) ---
    // Half-difference applied as left = cmd - corr, right = cmd + corr.
    volatile float _yaw_corr = 0.0f;
    float _yaw_lock_diff = 0.0f;   // left-right wheel position diff at engage (rad)

    // --- Speed-mode gate (control task writes, fast task reads) ---
    volatile bool  _targets_initialized = false;
    bool _speed_mode_active = false;
    uint32_t _last_speed_refresh_ms = 0;   // 0-speed keepalive during tip-up

    // Dead-man: control task stamps this every update(); if it goes stale while
    // balancing, fast task stops the wheels instead of driving blind on a
    // frozen setpoint (a stalled control task also cannot process the RC switch).
    volatile uint32_t _last_update_ms = 0;
    uint32_t _loop_wake_ms = 0;   // when control task last woke from a stall

    // Front wheel hold positions
    float _front_left_hold  = 0.0f;
    float _front_right_hold = 0.0f;

    // --- Setpoint handoff state (control task only) ---
    float _wheel_start_pos    = 0.0f;
    float _hold_drift         = 0.0f; // settled hold relative to engage; raw travel remains logged
    float _engage_capture_shift = 0.0f;
    float _smoothed_base_sp = 0.0f;
    float _engage_arm_frac = 1.0f;
    float _engage_trim = 0.0f;   // stored trim captured at engage, part of base sp
    float _run_curve_shift = 0.0f;  // per-run curve re-zero measured at capture

    // --- Outer cascade: position P -> velocity PI (control task only) ---
    float _drift_vel_kp       = BALANCE_DRIFT_VEL_KP;
    float _vel_sp_kp          = BALANCE_VEL_SP_KP;
    float _vel_sp_ki          = BALANCE_VEL_SP_KI;
    float _arm_assist_frac    = 0.0f;   // arm excursion toward center (0..MAX_FRAC)
    float _arm_assist_vel     = 0.0f;   // dedicated slow LPF of vel_err for the assist
    // Arm engagement state machine: 0=READY 1=ACTIVE 2=HANDOFF 3=COOLDOWN
    uint8_t _arm_stage        = 3;
    float   _arm_sign         = 0.0f;   // engagement direction (+1/-1)
    float   _arm_calm_ms      = 0.0f;   // accumulated calm time for re-arming
    float _arm_center_left    = 0.0f;   // calibrated center-axis deltas from forward
    float _arm_center_right   = 0.0f;
    balance_math::RunawayDetector _startup_detector;
    balance_math::StartupRecovery _startup_recovery;
    balance_math::RecoilUnwind _recoil_unwind;
    balance_math::BalancePilot _pilot;
    bool _pilot_input_valid = false;
    volatile float _pilot_velocity_ff = 0; // requested speed for the 200 Hz driving controller
    volatile float _pilot_measured_vel = 0; // 50 Hz filtered feedback snapshot
    volatile bool _pilot_driving = false;
    balance_math::BalanceDrive _pilot_drive;
    balance_math::DriveArmRecovery _drive_arm_recovery; // control-task owned
    volatile bool _pilot_drive_active = false;
    float _pilot_arm_applied = 0;
    float _vel_sp_integral    = 0.0f;   // deg (the single integrator = equilibrium estimate)
    float _sp_offset          = 0.0f;   // deg, added to base setpoint
    float _filtered_wheel_vel = 0.0f;   // rad/s
    float _last_target_vel    = 0.0f;   // rad/s, for telemetry
    uint32_t _ramp_complete_ms = 0;     // when the outer cascade activated

    // --- Safety abort timers (control task only) ---
    uint32_t _safe_err_start_ms  = 0;
    bool     _safe_err_timing    = false;
    uint32_t _safe_rate_start_ms = 0;
    bool     _safe_rate_timing   = false;
    uint32_t _safe_sat_start_ms  = 0;
    bool     _safe_sat_timing    = false;

    // --- Last values for telemetry (volatile: written by fast task, read by control task) ---
    volatile float _last_angle_err = 0.0f;
    volatile float _last_motor_vel_raw = 0.0f;
    volatile float _last_motor_vel = 0.0f;
    volatile float _last_cmd_left  = 0.0f;
    volatile float _last_cmd_right = 0.0f;
    volatile uint16_t _last_inner_diag = 0;
    volatile uint16_t _inner_dt_max_us = 0;
    volatile uint8_t  _inner_ticks = 0;
    volatile uint8_t  _inner_sat_ticks = 0;
    portMUX_TYPE _telemetry_mux = portMUX_INITIALIZER_UNLOCKED;
    volatile uint16_t _last_update_age_ms = 0;
    float _last_meas_drift = 0.0f;
    float _last_meas_vel   = 0.0f;
    float _last_base_sp = 0.0f;
    float _last_raw_base_sp = 0.0f;
    float _last_capture_shift = 0.0f;
    float _last_sp_offset_target = 0.0f;
    float _last_vel_err = 0.0f;
    float _last_vel_p_term = 0.0f;
    float _last_pos_gate = 0.0f;
    float _last_shed = 0.0f;
    float _last_arm_tip_frac = 0.0f;
    float _last_arm_demand = 0.0f;
    float _last_yaw_diff = 0.0f;
    uint16_t _last_outer_diag = 0;
    uint8_t _last_flags    = 0;

    // Arm ramp state (control task only)
    float _arm_left_target  = 0.0f;
    float _arm_right_target = 0.0f;
    float _arm_left_goal    = 0.0f;
    float _arm_right_goal   = 0.0f;
    float _arm_ramp_speed   = 0.0f;
    float _arm_tip_left_goal  = 0.0f;
    float _arm_tip_right_goal = 0.0f;
    bool  _arms_reached_tip   = false;
    bool  _arms_returning     = false;
    bool  _arms_returned      = false;
    bool  _ramp_complete      = false;
    uint32_t _balance_start_ms = 0;
    bool     _capture_stable   = false;
    uint32_t _capture_stable_start_ms = 0;

    // Telemetry logging
    BalanceSample* _log_buf    = nullptr;
    int            _log_count  = 0;
    bool           _logging    = false;
    uint32_t       _log_start_ms = 0;
    uint32_t       _log_end_ms = 0;
    uint32_t       _last_log_sample_ms = 0;
    uint32_t       _last_log_flush_attempt_ms = 0;
    bool           _log_saved  = false;
    volatile bool  _log_pending_flush = false;
    volatile bool  _log_flush_in_progress = false;
    uint16_t       _marker_count = 0;
    char           _next_log_note[64] = {};
    char           _log_note[64] = {};
    char           _log_end_reason[48] = {};
    BalanceLogConfigSnapshot _log_config = {};
    bool           _trim_save_pending = false;
    float          _pending_trim_old = 0.0f;
    float          _pending_trim_value = 0.0f;
    float          _pending_trim_learned = 0.0f;
    float          _qualified_trim = 0.0f;
    uint32_t       _trim_calm_since_ms = 0;
    uint32_t       _qualified_trim_ms = 0;
    uint32_t       _arm_return_start_ms = 0;

    static float clampf(float value, float min_value, float max_value);
    static float moveToward(float current, float target, float rate, float dt);
    void armAxisFractions(float& tip_frac, float& center_frac) const;
    float computeArmFraction() const;
    float computeScheduledSetpoint() const;
    void enterTippingUp();
    void enterBalancing(float current_roll);
    void enterReturningArms(const char* reason);
    void disengage(const char* reason);
    void exitSpeedMode();
    void persistLearnedTrim();
    void resetSafetyTimers();
    bool readyToStart() const;
    bool armsAtGoal(float left, float right) const;
    bool startLog();
    void logSample(float roll_deg, float roll_rate_dps);
    void stopLog(const char* reason);
    void flushLogToFile();
};
