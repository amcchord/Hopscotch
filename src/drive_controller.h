#pragma once

#include "motor_manager.h"
#include "crsf.h"

class DriveController {
public:
    void begin(MotorManager* motors);

    // Update drive targets from RC input. Call at control loop rate.
    // throttle/steering are normalized -1..+1 (deadband already applied)
    void update(float throttle, float steering, float dt_sec);

    // Emergency stop: freeze all position targets at current motor positions
    void emergencyStop();

    // Configurable parameters
    void setMaxSpeed(float rad_s) { _max_speed = rad_s; }
    void setPositionHorizon(float sec) { _horizon_sec = sec; }
    float getMaxSpeed() const { return _max_speed; }
    float getPositionHorizon() const { return _horizon_sec; }

    // Target positions for telemetry
    float getTargetPosition(MotorRole role) const;
    float getCommandedSpeed(MotorRole role) const;

private:
    MotorManager* _motors = nullptr;
    float _max_speed = 10.0f;     // rad/s
    float _horizon_sec = 2.0f;    // how far ahead to set target position

    // Per-motor rolling position targets
    float _target_pos[NUM_DRIVE_MOTORS] = {0};
    float _cmd_speed[NUM_DRIVE_MOTORS] = {0};
    bool  _initialized = false;
};
