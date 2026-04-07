#pragma once

#include "motor_manager.h"

class ArmController {
public:
    void begin(MotorManager* motors);

    // Update arm positions from RC channels.
    // left_input / right_input are normalized -1..+1
    void update(float left_input, float right_input, float dt_sec);

    // Hold arms at current position (called on signal loss)
    void holdPosition();

    // Configurable parameters
    void setMaxArmSpeed(float rad_s) { _max_arm_speed = rad_s; }
    void setArmRange(float rad) { _arm_range = rad; }

    float getTargetPosition(MotorRole role) const;

private:
    MotorManager* _motors = nullptr;
    float _max_arm_speed = 5.0f;   // rad/s max arm travel speed
    float _arm_range = 3.14f;      // +/- range in radians
    float _target_left = 0.0f;
    float _target_right = 0.0f;
    bool  _initialized = false;
};
