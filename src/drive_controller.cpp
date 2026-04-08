#include "drive_controller.h"
#include <Arduino.h>
#include <cmath>

void DriveController::begin(MotorManager* motors) {
    _motors = motors;
    _initialized = false;
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        _target_pos[i] = 0.0f;
        _cmd_speed[i] = 0.0f;
        _actual_speed[i] = 0.0f;
        _actual_pos[i] = 0.0f;
        _brake_speed_limit[i] = 0.0f;
        _state[i] = DriveMotorState::Idle;
    }
}

void DriveController::update(float throttle, float steering, float dt_sec) {
    if (!_motors || !_motors->isDriveArmed()) return;

    // On first update after arming, sync targets to current motor positions
    if (!_initialized) {
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
            MotorRole role = static_cast<MotorRole>(i);
            _target_pos[i] = _motors->getMotor(role).position;
            _actual_pos[i] = _target_pos[i];
            _actual_speed[i] = 0.0f;
            _state[i] = DriveMotorState::Idle;
        }
        _initialized = true;
        Serial.println("[Drive] Initialized -- synced targets to current positions");
    }

    // Tank-style arcade mixing
    float left_norm = throttle + steering;
    float right_norm = throttle - steering;

    if (left_norm > 1.0f) left_norm = 1.0f;
    if (left_norm < -1.0f) left_norm = -1.0f;
    if (right_norm > 1.0f) right_norm = 1.0f;
    if (right_norm < -1.0f) right_norm = -1.0f;

    float left_speed = left_norm * _max_speed;
    float right_speed = right_norm * _max_speed;

    float speeds[NUM_DRIVE_MOTORS] = {
        right_speed,  // FrontRight
        right_speed,  // BackRight
        left_speed,   // BackLeft
        left_speed,   // FrontLeft
    };

    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        MotorRole role = static_cast<MotorRole>(i);
        float current_pos = _motors->getMotor(role).position;
        float current_vel = _motors->getMotor(role).velocity;

        _actual_pos[i] = current_pos;
        _actual_speed[i] = current_vel;
        _cmd_speed[i] = speeds[i];

        float abs_cmd = fabsf(speeds[i]);
        float abs_actual = fabsf(current_vel);

        if (abs_cmd >= DRIVE_COMMAND_THRESHOLD) {
            // --- DRIVING: stick is active ---
            if (_state[i] != DriveMotorState::Driving) {
                Serial.printf("[Drive] Motor %d: DRIVING (cmd=%.1f rad/s = %.0f RPM)\n",
                              i, speeds[i], speeds[i] * RAD_S_TO_RPM);
            }
            _state[i] = DriveMotorState::Driving;

            // Horizon distance = commanded_speed * horizon_sec.
            // The speed_limit parameter on the motor caps the actual velocity,
            // so the target only needs to be far enough ahead that the motor
            // doesn't reach it and decelerate prematurely.
            float horizon_distance = abs_cmd * _horizon_sec;

            float direction = (speeds[i] > 0) ? 1.0f : -1.0f;
            _target_pos[i] = current_pos + direction * horizon_distance;

            _motors->sendDrivePosition(role, _target_pos[i], abs_cmd);

        } else if (_state[i] == DriveMotorState::Driving) {
            // --- TRANSITION: stick just went to center, hard stop ---
            _state[i] = DriveMotorState::Braking;

            Serial.printf("[Drive] Motor %d: BRAKING from %.1f rad/s (%.0f RPM)\n",
                          i, current_vel, current_vel * RAD_S_TO_RPM);

            // Capture a FIXED stop position and send it ONCE with the full
            // position command. Then on subsequent ticks, only write
            // SPEED_LIMIT to avoid re-triggering the motor at the old speed.
            _target_pos[i] = current_pos;
            _motors->sendDrivePosition(role, _target_pos[i], HOLD_SPEED_LIMIT);

        } else if (_state[i] == DriveMotorState::Braking) {
            // --- BRAKING: keep sending fixed target with zero speed limit ---
            // Speed limit 0 tells the motor to not move at all.
            _motors->sendDrivePosition(role, _target_pos[i], 0.0f);

            if (abs_actual < STOP_THRESHOLD) {
                _state[i] = DriveMotorState::Idle;
                _target_pos[i] = current_pos;
                Serial.printf("[Drive] Motor %d: STOPPED at pos=%.2f rad\n", i, current_pos);
            }

        } else {
            // --- IDLE: hold the captured position with zero speed limit ---
            // Speed limit 0 locks the motor at the target position with no
            // allowed movement. The target was captured when transitioning
            // to Idle from Braking or on init.
            _motors->sendDrivePosition(role, _target_pos[i], 0.0f);
        }
    }
}

void DriveController::emergencyStop() {
    if (!_motors) return;

    Serial.println("[Drive] EMERGENCY STOP");
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        MotorRole role = static_cast<MotorRole>(i);
        float current_pos = _motors->getMotor(role).position;
        _target_pos[i] = current_pos;
        _cmd_speed[i] = 0.0f;
        _actual_speed[i] = 0.0f;
        _brake_speed_limit[i] = 0.0f;
        _state[i] = DriveMotorState::Idle;
        _motors->sendDrivePosition(role, current_pos, 0.0f);
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

float DriveController::getActualSpeed(MotorRole role) const {
    int idx = static_cast<int>(role);
    if (idx < NUM_DRIVE_MOTORS) return _actual_speed[idx];
    return 0.0f;
}

DriveMotorState DriveController::getMotorState(MotorRole role) const {
    int idx = static_cast<int>(role);
    if (idx < NUM_DRIVE_MOTORS) return _state[idx];
    return DriveMotorState::Idle;
}

void DriveController::printDebug() {
    if (!_motors || !_motors->isDriveArmed()) return;

    static const char* state_names[] = {"IDLE", "DRIVE", "BRAKE"};

    Serial.println("[Drive] --- Per-Motor Status ---");
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        float cmd_rpm = _cmd_speed[i] * RAD_S_TO_RPM;
        float act_rpm = _actual_speed[i] * RAD_S_TO_RPM;
        float speed_err = _cmd_speed[i] - _actual_speed[i];
        float pos_err = _target_pos[i] - _actual_pos[i];

        Serial.printf("[Drive] M%d %s | cmd=%6.1f rad/s (%5.0f RPM) | act=%6.1f rad/s (%5.0f RPM) | "
                      "spd_err=%+.1f | tgt_pos=%7.2f | act_pos=%7.2f | pos_err=%+.2f",
                      i, state_names[static_cast<int>(_state[i])],
                      _cmd_speed[i], cmd_rpm,
                      _actual_speed[i], act_rpm,
                      speed_err,
                      _target_pos[i], _actual_pos[i], pos_err);

        if (_state[i] == DriveMotorState::Braking) {
            Serial.printf(" | brake_lim=%.1f", _brake_speed_limit[i]);
        }
        Serial.println();
    }
}
