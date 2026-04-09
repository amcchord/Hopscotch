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
    float    integral;
    float    bl_pos, br_pos;
    float    bl_vel, br_vel;
    float    arm_l, arm_r;
};

static constexpr int BALANCE_LOG_MAX_SAMPLES = 1500;

class BalanceController {
public:
    void begin(MotorManager* motors, ArmController* arms);

    // Called from Core 0 at 200Hz -- fast PD balance loop
    void balanceTick(const RawImuData& imu, float dt);

    // Called from Core 1 at 50Hz -- state machine, arms, adaptive setpoint, position PID
    void update(float roll_deg, float roll_rate_dps,
                bool ch7_active, bool ch11_edge, float dt);

    BalanceState getState() const { return _state; }
    bool isActive() const { return _state != BalanceState::Idle; }
    bool isControllingDrive() const {
        return _state == BalanceState::TippingUp || _state == BalanceState::Balancing;
    }

    const char* getStateString() const;

    // Force-engage balance from current position (skips tip-up)
    void forceEngage();

    // Complementary filter outputs (written by Core 0, read by Core 1)
    float getTiltAngle() const { return _tilt_angle; }
    float getGyroRate() const { return _gyro_rate; }

    // Gain setters for serial tuning
    void setKp(float v) { _kp = v; }
    void setKd(float v) { _kd = v; }
    void setFfGain(float v) { _ff_gain = v; }
    void setBaseDeg(float v) { _base_deg = v; }
    void setAdaptRate(float v) { _adapt_rate = v; }
    void setPosKp(float v) { _pos_kp = v; }
    void setPosKi(float v) { _pos_ki = v; }
    void setPosKd(float v) { _pos_kd = v; }

    float getKp() const { return _kp; }
    float getKd() const { return _kd; }
    float getFfGain() const { return _ff_gain; }
    float getBaseDeg() const { return _base_deg; }
    float getAdaptRate() const { return _adapt_rate; }
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

    // --- Feedforward from arm position ---
    float _base_deg = BALANCE_BASE_DEG;
    float _ff_gain  = BALANCE_FF_GAIN;
    float _ff_setpoint = BALANCE_BASE_DEG;  // computed each tick from arm position

    // --- Complementary filter (Core 0) ---
    volatile float _tilt_angle = 0.0f;
    volatile float _gyro_rate  = 0.0f;
    bool _filter_initialized = false;

    // --- PD gains (read by Core 0, written by Core 1 for tuning) ---
    volatile float _kp = BALANCE_KP;
    volatile float _kd = BALANCE_KD;

    // --- Effective setpoint (written by Core 1, read by Core 0) ---
    volatile float _effective_setpoint = BALANCE_BASE_DEG;

    // --- Velocity bias from position PID (written by Core 1, read by Core 0) ---
    volatile float _velocity_bias = 0.0f;

    // --- Back wheel targets (written by Core 0, read by Core 1 for position PID) ---
    volatile float _back_left_target  = 0.0f;
    volatile float _back_right_target = 0.0f;
    volatile bool  _targets_initialized = false;

    // Front wheel hold positions
    float _front_left_hold  = 0.0f;
    float _front_right_hold = 0.0f;

    // --- Adaptive fine-tuning (Core 1 only, on top of feedforward) ---
    float _adaptive_adjustment = 0.0f;
    float _cumulative_error    = 0.0f;
    float _adapt_rate = 0.05f; // very gentle fine-tuning only

    // --- Position return PID (Core 1 only) ---
    float _wheel_start_pos    = 0.0f;
    float _pos_kp             = 1.0f;   // rad/s per rad of drift (velocity bias)
    float _pos_ki             = 0.3f;   // rad/s per rad*sec of accumulated drift
    float _pos_kd             = 0.2f;   // rad/s per rad/s of wheel velocity
    float _pos_integral       = 0.0f;
    float _pos_integral_max   = 100.0f;
    float _prev_wheel_drift   = 0.0f;

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
    uint32_t _balance_start_ms = 0;

    // Telemetry logging
    BalanceSample* _log_buf    = nullptr;
    int            _log_count  = 0;
    bool           _logging    = false;
    uint32_t       _log_start_ms = 0;
    bool           _log_saved  = false;

    // Last values for logging
    volatile float _last_angle_err = 0.0f;
    volatile float _last_motor_vel = 0.0f;

    static float moveToward(float current, float target, float rate, float dt);
    void enterTippingUp();
    void enterBalancing(float current_roll);
    void enterReturningArms();
    void disengage();
    void startLog();
    void logSample(float roll_deg, float roll_rate_dps);
    void stopLog();
    void flushLogToFile();
};
