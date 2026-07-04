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
    uint8_t  state;
    float    roll;
    float    roll_rate;
    float    setpoint;
    float    angle_err;
    float    motor_vel;
    float    sp_offset;
    float    target_vel;
    float    bl_pos, br_pos;
    float    bl_vel, br_vel;
    float    arm_l, arm_r;
    float    arm_l_tgt, arm_r_tgt;
    float    meas_drift;
    float    meas_vel;
    uint8_t  flags;
};

static constexpr int BALANCE_LOG_MAX_SAMPLES = 3000;

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

    // Telemetry log
    void dumpLog();
    void clearLog();
    bool hasLog() const;
    size_t logSize() const;

    void printStatus();

private:
    MotorManager*    _motors   = nullptr;
    ArmController*   _arms     = nullptr;
    SettingsManager* _settings = nullptr;

    volatile BalanceState _state = BalanceState::Idle;

    // --- Complementary filter (Core 0) ---
    volatile float _tilt_angle = 0.0f;
    volatile float _gyro_rate  = 0.0f;
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
    volatile float _last_motor_vel = 0.0f;
    float _last_meas_drift = 0.0f;
    float _last_meas_vel   = 0.0f;
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
    bool           _log_saved  = false;

    static float clampf(float value, float min_value, float max_value);
    static float moveToward(float current, float target, float rate, float dt);
    void armAxisFractions(float& tip_frac, float& center_frac) const;
    float computeArmFraction() const;
    float computeScheduledSetpoint() const;
    void enterTippingUp();
    void enterBalancing(float current_roll);
    void enterReturningArms();
    void disengage();
    void exitSpeedMode();
    void persistLearnedTrim();
    void resetSafetyTimers();
    void startLog();
    void logSample(float roll_deg, float roll_rate_dps);
    void stopLog();
    void flushLogToFile();
};
