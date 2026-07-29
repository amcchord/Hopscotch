#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "motor_manager.h"
#include "arm_controller.h"
#include "settings.h"
#include "config.h"

enum class BalanceState : uint8_t {
    Idle           = 0,
    TippingUp      = 1,
    Balancing      = 2,
    ReturningArms  = 3,
};

struct RawImuData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

struct BalanceSample {
    uint32_t t_ms;
    uint16_t sample_dt_ms;
    uint16_t inner_dt_max_us;
    uint16_t feedback_age_l_ms;
    uint16_t feedback_age_r_ms;
    uint16_t update_age_ms;
    uint16_t marker;
    uint16_t diag_flags;
    uint8_t  state;
    uint8_t  flags;
    uint8_t  arm_stage;
    uint8_t  inner_ticks;
    uint8_t  inner_sat_ticks;
    uint8_t  reserved;
    float    roll;
    float    roll_rate;
    float    accel_angle;
    float    gyro_raw;
    float    accel_norm;
    float    setpoint;
    float    angle_err;
    float    base_sp;
    float    raw_base_sp;
    float    capture_shift;
    float    run_curve_shift;
    float    motor_vel_raw;
    float    motor_vel;
    float    cmd_left;
    float    cmd_right;
    float    sp_offset;
    float    sp_offset_target;
    float    target_vel;
    float    filtered_vel;
    float    vel_err;
    float    vel_integral;
    float    vel_p_term;
    float    pos_gate;
    float    shed;
    float    bl_pos, br_pos;
    float    bl_vel, br_vel;
    float    bl_torque, br_torque;
    float    arm_l, arm_r;
    float    arm_l_tgt, arm_r_tgt;
    float    arm_l_vel, arm_r_vel;
    float    arm_l_torque, arm_r_torque;
    float    meas_drift;
    float    meas_vel;
    float    arm_tip_frac;
    float    arm_assist_frac;
    float    arm_assist_vel;
    float    arm_demand;
    float    arm_calm_ms;
    float    yaw_diff;
    float    yaw_corr;
    float    bus_voltage;
    float    total_current;
};

// One 50 Hz sample for the complete 120-second capture. The in-memory and
// on-flash representations are binary so this expanded forensic schema fits
// comfortably in PSRAM and the 1.5 MB LittleFS partition.
static constexpr int BALANCE_LOG_MAX_SAMPLES = 6000;

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
};

static constexpr int BALANCE_LOG_MAX_CURVE_POINTS = 6;

struct BalanceLogConfigSnapshot {
    float inner_kp;
    float inner_kd;
    float drift_vel_kp;
    float drift_max_vel;
    float ramp_drift_kp;
    float ramp_drift_max_vel;
    float vel_sp_kp;
    float vel_sp_kp_low;
    float vel_sp_knee;
    float vel_sp_ki;
    float sp_offset_max;
    float ramp_sp_offset_max;
    float sp_offset_rate;
    float vel_filter_alpha;
    float pos_gate_err;
    float shed_vel_start;
    float shed_vel_full;
    float stored_trim;
    float glide_vel_err;
    float glide_ki_boost;
    float speed_acc_rad;
    float speed_current_limit;
    float base_sp_fwd;
    float base_sp_tip;
    float base_sp_center;
    float base_sp_rate_max;
    float ramp_vel_slow;
    float comp_alpha;
    float max_drive_speed;
    float arm_return_speed;
    float arm_assist_thresh;
    float arm_assist_gain;
    float arm_range_pos;
    float arm_range_neg;
    float arm_tau_in;
    float arm_tau_out;
    float arm_emergency_cmd_frac;
    float yaw_sync_kp;
    float yaw_sync_max;
    uint32_t capture_settle_ms;
    uint32_t arm_hold_max_ms;
    uint32_t log_duration_ms;
    uint16_t balance_loop_hz;
    uint16_t control_loop_hz;
    uint8_t curve_len;
    uint8_t reserved[3];
    float curve_frac[BALANCE_LOG_MAX_CURVE_POINTS];
    float curve_sp[BALANCE_LOG_MAX_CURVE_POINTS];
};

class BalanceController {
public:
    void begin(MotorManager* motors, ArmController* arms);
    void setSettingsManager(SettingsManager* mgr) { _settings = mgr; }

    // Called from Core 0 at 200Hz -- fast PD balance loop
    void balanceTick(const RawImuData& imu, float dt);

    // Called from Core 1 at 50Hz -- state machine, arms, velocity-integrating setpoint
    void update(float roll_deg, float roll_rate_dps,
                bool ch7_active, bool ch11_edge, float dt);

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

    // --- Complementary filter (Core 0) ---
    volatile float _tilt_angle = 0.0f;
    volatile float _gyro_rate  = 0.0f;
    volatile float _last_accel_angle = 0.0f;
    volatile float _last_gyro_raw    = 0.0f;
    volatile float _last_accel_norm  = 0.0f;
    bool _filter_initialized = false;

    // --- PD gains (read by Core 0, written by Core 1 for tuning) ---
    volatile float _kp = BALANCE_KP;
    volatile float _kd = BALANCE_KD;

    // --- Effective setpoint (written by Core 1, read by Core 0) ---
    volatile float _effective_setpoint = BALANCE_SETPOINT_ARMS_TIP;

    // --- Yaw sync: differential wheel correction (Core 1 writes, Core 0 reads) ---
    // Half-difference applied as left = cmd - corr, right = cmd + corr.
    volatile float _yaw_corr = 0.0f;
    float _yaw_lock_diff = 0.0f;   // left-right wheel position diff at engage (rad)

    // --- Speed-mode gate (Core 1 writes, Core 0 reads) ---
    volatile bool  _targets_initialized = false;
    bool _speed_mode_active = false;
    uint32_t _last_speed_refresh_ms = 0;   // 0-speed keepalive during tip-up

    // Dead-man: Core 1 stamps this every update(); if it goes stale while
    // balancing, Core 0 stops the wheels instead of driving blind on a
    // frozen setpoint (a stalled Core 1 also cannot process the RC switch).
    volatile uint32_t _last_update_ms = 0;
    uint32_t _loop_wake_ms = 0;   // when Core 1 last woke from a stall

    // Front wheel hold positions
    float _front_left_hold  = 0.0f;
    float _front_right_hold = 0.0f;

    // --- Setpoint handoff state (Core 1 only) ---
    float _wheel_start_pos    = 0.0f;
    float _engage_capture_shift = 0.0f;
    float _smoothed_base_sp = 0.0f;
    float _engage_arm_frac = 1.0f;
    float _engage_trim = 0.0f;   // stored trim captured at engage, part of base sp
    float _run_curve_shift = 0.0f;  // per-run curve re-zero measured at capture

    // --- Outer cascade: position P -> velocity PI (Core 1 only) ---
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
    float _vel_sp_integral    = 0.0f;   // deg (the single integrator = equilibrium estimate)
    float _sp_offset          = 0.0f;   // deg, added to base setpoint
    float _filtered_wheel_vel = 0.0f;   // rad/s
    float _last_target_vel    = 0.0f;   // rad/s, for telemetry
    uint32_t _ramp_complete_ms = 0;     // when the outer cascade activated

    // --- Safety abort timers (Core 1 only) ---
    uint32_t _safe_err_start_ms  = 0;
    bool     _safe_err_timing    = false;
    uint32_t _safe_rate_start_ms = 0;
    bool     _safe_rate_timing   = false;
    uint32_t _safe_sat_start_ms  = 0;
    bool     _safe_sat_timing    = false;

    // --- Last values for telemetry (volatile: written by Core 0, read by Core 1) ---
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

    // Arm ramp state (Core 1 only)
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
    bool startLog();
    void logSample(float roll_deg, float roll_rate_dps);
    void stopLog(const char* reason);
    void flushLogToFile();
};
