#include "arm_controller.h"
#include <Arduino.h>
#include <cmath>

void ArmController::begin(MotorManager* motors) {
    _motors = motors;
    _target_left = 0.0f;
    _target_right = 0.0f;
    _initialized = false;
}

void ArmController::update(float left_input, float right_input, float dt_sec) {
    if (!_motors || !_motors->isArmArmed()) return;

    // On first call, sync to current positions
    if (!_initialized) {
        _target_left = _motors->getMotor(MotorRole::ArmLeft).position;
        _target_right = _motors->getMotor(MotorRole::ArmRight).position;
        _initialized = true;
    }

    // RC input controls arm velocity (rate mode):
    // stick input * max_speed = velocity, integrate into position
    _target_left += left_input * _max_arm_speed * dt_sec;
    _target_right += right_input * _max_arm_speed * dt_sec;

    // Clamp to range
    if (_target_left > _arm_range) _target_left = _arm_range;
    if (_target_left < -_arm_range) _target_left = -_arm_range;
    if (_target_right > _arm_range) _target_right = _arm_range;
    if (_target_right < -_arm_range) _target_right = -_arm_range;

    float speed_left = fabsf(left_input) * _max_arm_speed;
    float speed_right = fabsf(right_input) * _max_arm_speed;

    // Minimum speed so the motor can still reach the position
    if (speed_left < 0.5f) speed_left = 0.5f;
    if (speed_right < 0.5f) speed_right = 0.5f;

    _motors->sendArmPosition(MotorRole::ArmLeft, _target_left, speed_left);
    _motors->sendArmPosition(MotorRole::ArmRight, _target_right, speed_right);
}

void ArmController::holdPosition() {
    if (!_motors) return;

    // Hold at current actual position
    _target_left = _motors->getMotor(MotorRole::ArmLeft).position;
    _target_right = _motors->getMotor(MotorRole::ArmRight).position;

    _motors->sendArmPosition(MotorRole::ArmLeft, _target_left, 0.5f);
    _motors->sendArmPosition(MotorRole::ArmRight, _target_right, 0.5f);

    _initialized = false;
}

float ArmController::getTargetPosition(MotorRole role) const {
    if (role == MotorRole::ArmLeft) return _target_left;
    if (role == MotorRole::ArmRight) return _target_right;
    return 0.0f;
}
