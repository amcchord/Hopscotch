#pragma once

#include "motor_manager.h"
#include "crsf.h"

static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * 3.14159265f);

enum class DriveMotorState : uint8_t {
    Idle,       // stick at zero, motor stopped, holding position
    Driving,    // stick active, motor running
    Braking,    // stick returned to zero, decelerating
};

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

    // Telemetry getters
    float getTargetPosition(MotorRole role) const;
    float getCommandedSpeed(MotorRole role) const;
    float getActualSpeed(MotorRole role) const;
    DriveMotorState getMotorState(MotorRole role) const;

    // Print per-motor debug info to Serial
    void printDebug();

private:
    MotorManager* _motors = nullptr;
    float _max_speed = 33.0f;    // rad/s (RS05 max)
    float _horizon_sec = 3.0f;   // how far ahead to set target position

    // Per-motor state
    float _target_pos[NUM_DRIVE_MOTORS] = {0};
    float _cmd_speed[NUM_DRIVE_MOTORS] = {0};
    float _actual_speed[NUM_DRIVE_MOTORS] = {0};
    float _actual_pos[NUM_DRIVE_MOTORS] = {0};
    float _brake_speed_limit[NUM_DRIVE_MOTORS] = {0};
    DriveMotorState _state[NUM_DRIVE_MOTORS] = {};
    bool  _initialized = false;

    static constexpr float STOP_THRESHOLD = 0.3f;       // rad/s: below this, consider motor stopped
    static constexpr float HOLD_SPEED_LIMIT = 0.5f;     // rad/s: speed limit when holding position
    static constexpr float DRIVE_COMMAND_THRESHOLD = 0.01f; // rad/s: below this, stick counts as centered
};
