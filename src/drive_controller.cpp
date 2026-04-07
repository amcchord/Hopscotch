#include "drive_controller.h"
#include <Arduino.h>
#include <cmath>

void DriveController::begin(MotorManager* motors) {
    _motors = motors;
    _initialized = false;
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        _target_pos[i] = 0.0f;
        _cmd_speed[i] = 0.0f;
    }
}

void DriveController::update(float throttle, float steering, float dt_sec) {
    if (!_motors || !_motors->isDriveArmed()) return;

    // On first update after arming, sync targets to current motor positions
    if (!_initialized) {
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
            MotorRole role = static_cast<MotorRole>(i);
            _target_pos[i] = _motors->getMotor(role).position;
        }
        _initialized = true;
    }

    // Tank-style arcade mixing:
    //   left_speed  = throttle + steering
    //   right_speed = throttle - steering
    // Clamp to [-1, 1]
    float left_norm = throttle + steering;
    float right_norm = throttle - steering;

    if (left_norm > 1.0f) left_norm = 1.0f;
    if (left_norm < -1.0f) left_norm = -1.0f;
    if (right_norm > 1.0f) right_norm = 1.0f;
    if (right_norm < -1.0f) right_norm = -1.0f;

    // Convert normalized speed to rad/s
    float left_speed = left_norm * _max_speed;
    float right_speed = right_norm * _max_speed;

    // Assign speeds to motors:
    //   [0] FrontRight = right_speed
    //   [1] BackRight  = right_speed
    //   [2] BackLeft   = left_speed
    //   [3] FrontLeft  = left_speed
    float speeds[NUM_DRIVE_MOTORS] = {
        right_speed,  // FrontRight
        right_speed,  // BackRight
        left_speed,   // BackLeft
        left_speed,   // FrontLeft
    };

    // Rolling position horizon failsafe:
    // Set the target position to be (horizon_sec * speed) ahead of current position.
    // The speed_limit on the motor controls actual speed.
    // If we stop updating, the motor reaches the target and stops.
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        MotorRole role = static_cast<MotorRole>(i);
        float current_pos = _motors->getMotor(role).position;

        float abs_speed = fabsf(speeds[i]);
        _cmd_speed[i] = speeds[i];

        if (abs_speed < 0.01f) {
            // Near zero: hold current position
            _target_pos[i] = current_pos;
            _motors->sendDrivePosition(role, _target_pos[i], 1.0f);
        } else {
            // Set target position horizon_sec ahead in the direction of travel
            _target_pos[i] = current_pos + speeds[i] * _horizon_sec;
            _motors->sendDrivePosition(role, _target_pos[i], abs_speed);
        }
    }
}

void DriveController::emergencyStop() {
    if (!_motors) return;

    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        MotorRole role = static_cast<MotorRole>(i);
        float current_pos = _motors->getMotor(role).position;
        _target_pos[i] = current_pos;
        _cmd_speed[i] = 0.0f;
        _motors->sendDrivePosition(role, current_pos, 0.5f);
    }
    _initialized = false;
}

float DriveController::getTargetPosition(MotorRole role) const {
    int idx = static_cast<int>(role);
    if (idx < NUM_DRIVE_MOTORS) return _target_pos[idx];
    return 0.0f;
}

float DriveController::getCommandedSpeed(MotorRole role) const {
    int idx = static_cast<int>(role);
    if (idx < NUM_DRIVE_MOTORS) return _cmd_speed[idx];
    return 0.0f;
}
