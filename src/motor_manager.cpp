#include "motor_manager.h"
#include <Arduino.h>

void MotorManager::begin(Robstride* can_bus) {
    _can = can_bus;

    // Initialize motor states with defaults
    _motors[0] = { DEFAULT_MOTOR_ID_FRONT_RIGHT, MotorRole::FrontRight, false, false, false, false, 0,0,0,0,0,0, RobstrideRunMode::Position };
    _motors[1] = { DEFAULT_MOTOR_ID_BACK_RIGHT,  MotorRole::BackRight,  false, false, false, false, 0,0,0,0,0,0, RobstrideRunMode::Position };
    _motors[2] = { DEFAULT_MOTOR_ID_BACK_LEFT,   MotorRole::BackLeft,   true,  false, false, false, 0,0,0,0,0,0, RobstrideRunMode::Position };
    _motors[3] = { DEFAULT_MOTOR_ID_FRONT_LEFT,  MotorRole::FrontLeft,  true,  false, false, false, 0,0,0,0,0,0, RobstrideRunMode::Position };
    _motors[4] = { DEFAULT_MOTOR_ID_ARM_LEFT,    MotorRole::ArmLeft,    false, false, false, false, 0,0,0,0,0,0, RobstrideRunMode::Position };
    _motors[5] = { DEFAULT_MOTOR_ID_ARM_RIGHT,   MotorRole::ArmRight,   false, false, false, false, 0,0,0,0,0,0, RobstrideRunMode::Position };

    _drive_armed = false;
    _arm_armed = false;

    Serial.println("[Motors] Manager initialized with 6 motors");
}

void MotorManager::setMotorId(MotorRole role, uint8_t can_id) {
    int idx = static_cast<int>(role);
    if (idx < NUM_MOTORS) {
        _motors[idx].can_id = can_id;
    }
}

uint8_t MotorManager::getMotorId(MotorRole role) const {
    int idx = static_cast<int>(role);
    if (idx < NUM_MOTORS) {
        return _motors[idx].can_id;
    }
    return 0;
}

int MotorManager::findMotorByCanId(uint8_t can_id) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (_motors[i].can_id == can_id) return i;
    }
    return -1;
}

bool MotorManager::enableAndConfigureMotor(int idx, RobstrideRunMode mode) {
    if (!_can || idx < 0 || idx >= NUM_MOTORS) return false;

    MotorState& m = _motors[idx];

    // Stop first to clear state
    _can->stopMotor(m.can_id, CAN_HOST_ID, true);
    delay(10);

    // Set run mode to Position
    _can->setRunMode(m.can_id, CAN_HOST_ID, mode);
    delay(10);

    // Enable
    bool ok = _can->enableMotor(m.can_id, CAN_HOST_ID);
    if (ok) {
        m.enabled = true;
        m.run_mode = mode;
        Serial.printf("[Motors] Enabled motor ID=%d in mode %d\n", m.can_id, static_cast<int>(mode));
    }
    return ok;
}

bool MotorManager::armDriveMotors() {
    if (!_can) return false;

    bool all_ok = true;
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        if (!enableAndConfigureMotor(i, RobstrideRunMode::Position)) {
            all_ok = false;
        }
        delay(20);
    }
    _drive_armed = all_ok;
    Serial.printf("[Motors] Drive motors armed: %s\n", all_ok ? "OK" : "PARTIAL");
    return all_ok;
}

bool MotorManager::disarmDriveMotors() {
    if (!_can) return false;

    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        _can->stopMotor(_motors[i].can_id, CAN_HOST_ID, false);
        _motors[i].enabled = false;
        delay(5);
    }
    _drive_armed = false;
    Serial.println("[Motors] Drive motors disarmed");
    return true;
}

bool MotorManager::armArmMotors() {
    if (!_can) return false;

    bool all_ok = true;
    for (int i = NUM_DRIVE_MOTORS; i < NUM_MOTORS; i++) {
        if (!enableAndConfigureMotor(i, RobstrideRunMode::Position)) {
            all_ok = false;
        }
        delay(20);
    }
    _arm_armed = all_ok;
    Serial.printf("[Motors] Arm motors armed: %s\n", all_ok ? "OK" : "PARTIAL");
    return all_ok;
}

bool MotorManager::disarmArmMotors() {
    if (!_can) return false;

    for (int i = NUM_DRIVE_MOTORS; i < NUM_MOTORS; i++) {
        _can->stopMotor(_motors[i].can_id, CAN_HOST_ID, false);
        _motors[i].enabled = false;
        delay(5);
    }
    _arm_armed = false;
    Serial.println("[Motors] Arm motors disarmed");
    return true;
}

bool MotorManager::disarmAll() {
    bool d = disarmDriveMotors();
    bool a = disarmArmMotors();
    return d && a;
}

bool MotorManager::sendDrivePosition(MotorRole role, float position_rad, float speed_limit_rad_s) {
    int idx = static_cast<int>(role);
    if (idx >= NUM_DRIVE_MOTORS || !_can || !_drive_armed) return false;

    MotorState& m = _motors[idx];
    float pos = m.reversed ? -position_rad : position_rad;
    float spd = fabsf(speed_limit_rad_s);

    return _can->sendPositionCommand(m.can_id, CAN_HOST_ID, pos, spd);
}

bool MotorManager::sendArmPosition(MotorRole role, float position_rad, float speed_limit_rad_s) {
    int idx = static_cast<int>(role);
    if (idx < NUM_DRIVE_MOTORS || idx >= NUM_MOTORS || !_can || !_arm_armed) return false;

    MotorState& m = _motors[idx];
    return _can->sendPositionCommand(m.can_id, CAN_HOST_ID, position_rad, speed_limit_rad_s);
}

void MotorManager::processFeedback() {
    if (!_can) return;

    RobstrideFeedback fb;
    // Drain up to 16 messages per call
    for (int i = 0; i < 16; i++) {
        if (!_can->receiveFeedback(fb, 0)) break;

        int idx = findMotorByCanId(fb.motor_id);
        if (idx < 0) continue;

        MotorState& m = _motors[idx];
        float sign = m.reversed ? -1.0f : 1.0f;
        m.position = fb.position * sign;
        m.velocity = fb.velocity * sign;
        m.torque = fb.torque * sign;
        m.temperature = fb.temperature;
        m.errors = fb.errors;
        m.has_fault = fb.has_fault;
        m.online = true;
        m.last_feedback_ms = millis();
    }
}

void MotorManager::checkTimeouts(uint32_t timeout_ms) {
    uint32_t now = millis();
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (_motors[i].online && (now - _motors[i].last_feedback_ms > timeout_ms)) {
            _motors[i].online = false;
        }
    }
}

void MotorManager::scanNextMotor() {
    if (!_can) return;

    // Round-robin: send motion control ping (all zeros) to each motor.
    // This provokes a type-0x02 feedback response without commanding movement.
    MotorState& m = _motors[_scan_index];
    _can->sendMotionPing(m.can_id);

    _scan_index = (_scan_index + 1) % NUM_MOTORS;
}

bool MotorManager::changeMotorCanIdOnBus(uint8_t old_id, uint8_t new_id) {
    if (!_can) return false;
    return _can->changeMotorCanId(old_id, CAN_HOST_ID, new_id);
}
