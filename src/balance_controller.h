#pragma once

#include <cstdint>
#include "motor_manager.h"
#include "arm_controller.h"
#include "config.h"

enum class BalanceState : uint8_t {
    Idle           = 0,
    TippingUp      = 1,
    Balancing      = 2,
    ReturningArms  = 3,
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

static constexpr int BALANCE_LOG_MAX_SAMPLES = 1500;  // 30s at 50Hz

class BalanceController {
public:
    void begin(MotorManager* motors, ArmController* arms);

    void update(float roll_deg, float roll_rate_dps,
                bool ch7_active, bool ch11_edge, float dt);

    BalanceState getState() const { return _state; }
    bool isActive() const { return _state != BalanceState::Idle; }
    bool isControllingDrive() const {
        return _state == BalanceState::TippingUp || _state == BalanceState::Balancing;
    }

    const char* getStateString() const;

    // PD gain setters for serial tuning
    void setKp(float v) { _kp = v; }
    void setKd(float v) { _kd = v; }
    void setExpo(float v) { _expo = v; }
    void setAdaptRate(float v) { _adapt_rate = v; }
    void setPosKp(float v) { _pos_kp = v; }
    void setPosKi(float v) { _pos_ki = v; }
    void setPosKd(float v) { _pos_kd = v; }
    void setSetpoint(float v) { _setpoint_deg = v; _adaptive_setpoint = v; }

    float getKp() const { return _kp; }
    float getKd() const { return _kd; }
    float getAdaptRate() const { return _adapt_rate; }
    float getSetpoint() const { return _setpoint_deg; }
    float getAdaptiveSetpoint() const { return _adaptive_setpoint; }

    // Telemetry log
    void dumpLog();
    void clearLog();
    bool hasLog() const;
    size_t logSize() const;

    void printStatus();

private:
    MotorManager*   _motors = nullptr;
    ArmController*  _arms   = nullptr;

    BalanceState _state = BalanceState::Idle;
    float _setpoint_deg = BALANCE_SETPOINT_DEG;

    // PD gains (tunable at runtime via serial)
    float _kp = BALANCE_KP;
    float _kd = BALANCE_KD;
    float _expo = 1.5f;  // error exponent: 1.0=linear, 1.5=aggressive for large errors

    // Adaptive setpoint: tracks the true balance point via cumulative error
    float _adaptive_setpoint = BALANCE_SETPOINT_DEG;
    float _cumulative_error  = 0.0f;
    float _adapt_rate = 0.5f;  // deg of setpoint shift per degree*second of cumulative error

    // Back wheel position targets (accumulated)
    float _back_left_target  = 0.0f;
    float _back_right_target = 0.0f;
    bool  _targets_initialized = false;

    // Wheel position PID -- drives robot back toward starting position
    float _wheel_start_pos    = 0.0f;
    float _pos_kp             = 0.3f;   // deg lean per rad of drift
    float _pos_ki             = 0.2f;   // deg lean per rad*sec of accumulated drift
    float _pos_kd             = 0.05f;  // deg lean per rad/s of wheel velocity
    float _pos_integral       = 0.0f;
    float _pos_integral_max   = 100.0f;
    float _prev_wheel_drift   = 0.0f;

    // Front wheel hold positions
    float _front_left_hold  = 0.0f;
    float _front_right_hold = 0.0f;

    // Arm ramp state
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

    // Telemetry logging -- buffered in PSRAM, written to flash on stop
    BalanceSample* _log_buf    = nullptr;
    int            _log_count  = 0;
    bool           _logging    = false;
    uint32_t       _log_start_ms = 0;
    bool           _log_saved  = false;

    // Last values for logging
    float _last_angle_err = 0.0f;
    float _last_motor_vel = 0.0f;

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
