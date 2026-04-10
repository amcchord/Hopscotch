#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "motor_manager.h"
#include "arm_controller.h"
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
    float    vel_integrator;
    float    bl_pos, br_pos;
    float    bl_vel, br_vel;
    float    arm_l, arm_r;
    float    meas_drift;
    float    meas_vel;
    float    pos_shift;
    uint8_t  flags;
};

static constexpr int BALANCE_LOG_MAX_SAMPLES = 3000;

class BalanceController {
public:
    void begin(MotorManager* motors, ArmController* arms);

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
    void setVelGain(float v) { _vel_gain = v; }
    void setPosKp(float v) { _pos_kp = v; }
    void setPosKi(float v) { _pos_ki = v; }
    void setPosKd(float v) { _pos_kd = v; }

    float getKp() const { return _kp; }
    float getKd() const { return _kd; }
    float getVelGain() const { return _vel_gain; }
    float getEffectiveSetpoint() const { return _effective_setpoint; }

    // Telemetry log
    void dumpLog();
    void clearLog();
    bool hasLog() const;
    size_t logSize() const;

    void printStatus();

private:
    MotorManager*   _motors = nullptr;
    ArmController*  _arms   = nullptr;

    volatile BalanceState _state = BalanceState::Idle;

    // --- Complementary filter (Core 0) ---
    volatile float _tilt_angle = 0.0f;
    volatile float _gyro_rate  = 0.0f;
    bool _filter_initialized = false;

    // --- PD gains (read by Core 0, written by Core 1 for tuning) ---
    volatile float _kp = BALANCE_KP;
    volatile float _kd = BALANCE_KD;

    // --- Velocity-integrating setpoint (Core 1 writes, Core 0 reads) ---
    float _setpoint = 0.0f;
    float _vel_gain = BALANCE_VEL_GAIN;
    float _filtered_vel = 0.0f;

    // --- Effective setpoint (written by Core 1, read by Core 0) ---
    volatile float _effective_setpoint = BALANCE_SETPOINT_ARMS_TIP;

    // --- Back wheel targets (written by Core 0 for position commands) ---
    volatile float _back_left_target  = 0.0f;
    volatile float _back_right_target = 0.0f;
    volatile bool  _targets_initialized = false;

    // Front wheel hold positions
    float _front_left_hold  = 0.0f;
    float _front_right_hold = 0.0f;

    // --- Position return as bounded setpoint shift (Core 1, measured odometry) ---
    float _wheel_start_pos    = 0.0f;
    float _pos_kp             = BALANCE_POS_KP;
    float _pos_ki             = BALANCE_POS_KI;
    float _pos_kd             = BALANCE_POS_KD;
    float _pos_integral       = 0.0f;
    float _pos_setpoint_shift = 0.0f;

    // --- Stuck / wall detection (Core 1 only) ---
    bool     _stuck           = false;
    uint32_t _stuck_start_ms  = 0;

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
    bool  _arms_settled       = false;
    uint32_t _balance_start_ms = 0;

    // Telemetry logging
    BalanceSample* _log_buf    = nullptr;
    int            _log_count  = 0;
    bool           _logging    = false;
    uint32_t       _log_start_ms = 0;
    bool           _log_saved  = false;

    static float moveToward(float current, float target, float rate, float dt);
    void enterTippingUp();
    void enterBalancing(float current_roll);
    void enterReturningArms();
    void disengage();
    void resetSafetyTimers();
    void startLog();
    void logSample(float roll_deg, float roll_rate_dps);
    void stopLog();
    void flushLogToFile();
};
