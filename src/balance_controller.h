#pragma once

#include <cstdint>
#include <LittleFS.h>
#include "motor_manager.h"
#include "arm_controller.h"
#include "config.h"

enum class BalanceState : uint8_t {
    Idle      = 0,
    TippingUp = 1,
    Balancing = 2,
};

struct PidState {
    float kp       = 0.0f;
    float ki       = 0.0f;
    float kd       = 0.0f;
    float integral  = 0.0f;
    float prev_error = 0.0f;
    float i_max     = 100.0f;
    float out_min   = -100.0f;
    float out_max   =  100.0f;
};

class BalanceController {
public:
    void begin(MotorManager* motors, ArmController* arms);

    void update(float roll_deg, float roll_rate_dps,
                bool ch7_active, bool ch11_edge, float dt);

    BalanceState getState() const { return _state; }
    bool isActive() const { return _state != BalanceState::Idle; }

    const char* getStateString() const;

    // PID gain setters for serial tuning
    void setOuterKp(float v) { _outer.kp = v; }
    void setOuterKi(float v) { _outer.ki = v; }
    void setOuterKd(float v) { _outer.kd = v; }
    void setInnerKp(float v) { _inner.kp = v; }
    void setInnerKi(float v) { _inner.ki = v; }
    void setInnerKd(float v) { _inner.kd = v; }
    void setSetpoint(float v) { _setpoint_deg = v; }

    float getOuterKp() const { return _outer.kp; }
    float getOuterKi() const { return _outer.ki; }
    float getOuterKd() const { return _outer.kd; }
    float getInnerKp() const { return _inner.kp; }
    float getInnerKi() const { return _inner.ki; }
    float getInnerKd() const { return _inner.kd; }
    float getSetpoint() const { return _setpoint_deg; }

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

    // Cascaded PID controllers
    PidState _outer;
    PidState _inner;

    // Back wheel position targets (accumulated)
    float _back_left_target  = 0.0f;
    float _back_right_target = 0.0f;
    bool  _targets_initialized = false;

    // Front wheel hold positions
    float _front_left_hold  = 0.0f;
    float _front_right_hold = 0.0f;

    // Arm ramp state
    float _arm_left_target  = 0.0f;
    float _arm_right_target = 0.0f;
    float _arm_left_goal    = 0.0f;
    float _arm_right_goal   = 0.0f;
    float _arm_ramp_speed   = 0.0f;

    // Telemetry logging
    File  _log_file;
    bool  _logging = false;
    uint32_t _log_start_ms = 0;

    // Last PID outputs for logging
    float _last_outer_err = 0.0f;
    float _last_outer_out = 0.0f;
    float _last_inner_err = 0.0f;
    float _last_inner_out = 0.0f;
    float _last_motor_vel = 0.0f;

    static float pidCompute(PidState& pid, float error, float dt);
    static float moveToward(float current, float target, float rate, float dt);
    void enterTippingUp();
    void enterBalancing();
    void disengage();
    void startLog();
    void logSample(float roll_deg, float roll_rate_dps);
    void stopLog();
};
