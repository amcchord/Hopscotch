#include "motor_manager.h"
#include <Arduino.h>

static MotorState makeMotor(uint8_t can_id, MotorRole role, bool reversed) {
    MotorState m = {};
    m.can_id = can_id;
    m.role = role;
    m.reversed = reversed;
    m.online = false;
    m.enabled = false;
    m.has_fault = false;
    m.position = 0;
    m.raw_position = 0;
    m.prev_raw_position = 0;
    m.unwrap_offset = 0;
    m._abs_pos_offset = 0;
    m.has_first_feedback = false;
    m.velocity = 0;
    m.torque = 0;
    m.temperature = 0;
    m.errors = 0;
    m.last_feedback_ms = 0;
    m.run_mode = RobstrideRunMode::CSP;
    return m;
}

void MotorManager::begin(Robstride* can_bus) {
    _can = can_bus;

    _motors[0] = makeMotor(DEFAULT_MOTOR_ID_FRONT_RIGHT, MotorRole::FrontRight, false);
    _motors[1] = makeMotor(DEFAULT_MOTOR_ID_BACK_RIGHT,  MotorRole::BackRight,  false);
    _motors[2] = makeMotor(DEFAULT_MOTOR_ID_BACK_LEFT,   MotorRole::BackLeft,   true);
    _motors[3] = makeMotor(DEFAULT_MOTOR_ID_FRONT_LEFT,  MotorRole::FrontLeft,  true);
    _motors[4] = makeMotor(DEFAULT_MOTOR_ID_ARM_LEFT,    MotorRole::ArmLeft,    false);
    _motors[5] = makeMotor(DEFAULT_MOTOR_ID_ARM_RIGHT,   MotorRole::ArmRight,   false);

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

    // Per RS00 User Manual, the sequence for CSP mode is:
    //   1. Stop (type 4, clear fault)
    //   2. Set zero position (type 6) -- works in CSP and MIT modes, blocked in PP
    //   3. Set run_mode (type 18 param write)
    //   4. Enable (type 3)
    //   5. Write limit_spd and loc_ref

    _can->stopMotor(m.can_id, CAN_HOST_ID, true);
    delay(100);

    // Zero the position -- per manual section 4.2.6, this is supported in
    // CSP and Motion Control modes (blocked in PP). Since we're about to
    // set CSP mode, and the motor defaults to MIT after stop, zeroing works.
    _can->setZeroPosition(m.can_id, CAN_HOST_ID);
    delay(50);

    _can->setRunMode(m.can_id, CAN_HOST_ID, mode);
    delay(10);

    bool ok = _can->enableMotor(m.can_id, CAN_HOST_ID);
    delay(10);

    if (ok) {
        if (mode == RobstrideRunMode::CSP) {
            // CSP mode: set speed limit then hold at position 0
            _can->writeFloatParam(m.can_id, CAN_HOST_ID,
                                  RobstrideParam::SPEED_LIMIT, SPEC_RS05.max_speed);
            delay(5);
            _can->writeFloatParam(m.can_id, CAN_HOST_ID,
                                  RobstrideParam::TARGET_POSITION, 0.0f);
            delay(5);
        }

        m.enabled = true;
        m.run_mode = mode;
        m.position = 0.0f;
        m.raw_position = 0.0f;
        m.prev_raw_position = 0.0f;
        m.unwrap_offset = 0.0f;
        m._abs_pos_offset = 0.0f;
        m.has_first_feedback = false;
        Serial.printf("[Motors] Enabled motor ID=%d in CSP mode (zeroed)\n", m.can_id);
    }
    return ok;
}

bool MotorManager::armDriveMotors() {
    if (!_can) return false;

    bool all_ok = true;
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        if (!enableAndConfigureMotor(i, RobstrideRunMode::CSP)) {
            all_ok = false;
        }
        delay(20);
    }
    _drive_armed = all_ok;
    Serial.printf("[Motors] Drive motors armed (CSP): %s\n", all_ok ? "OK" : "PARTIAL");
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
        if (!enableAndConfigureMotor(i, RobstrideRunMode::CSP)) {
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

bool MotorManager::sendDriveSpeedLimit(MotorRole role, float speed_limit_rad_s) {
    int idx = static_cast<int>(role);
    if (idx >= NUM_DRIVE_MOTORS || !_can || !_drive_armed) return false;

    float spd = fabsf(speed_limit_rad_s);
    return _can->writeFloatParam(_motors[idx].can_id, CAN_HOST_ID,
                                 RobstrideParam::SPEED_LIMIT, spd);
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

        // Unwrap position: the feedback encodes position in [-4pi, +4pi]
        // and wraps when the motor rotates beyond that range. We detect
        // wraps by looking for jumps larger than half the range and
        // accumulate an offset to produce a continuous position.
        float raw = fb.position;
        m.raw_position = raw;

        if (!m.has_first_feedback) {
            m.prev_raw_position = raw;
            // Compute initial unwrap offset so that:
            //   raw + unwrap_offset = abs_pos_offset (the motor's true position)
            // This aligns our coordinate system with the motor's internal counter.
            m.unwrap_offset = m._abs_pos_offset - raw;
            m.has_first_feedback = true;
        } else {
            float delta = raw - m.prev_raw_position;
            if (delta > POSITION_FEEDBACK_HALF) {
                m.unwrap_offset -= POSITION_FEEDBACK_RANGE;
            } else if (delta < -POSITION_FEEDBACK_HALF) {
                m.unwrap_offset += POSITION_FEEDBACK_RANGE;
            }
            m.prev_raw_position = raw;
        }

        m.position = (raw + m.unwrap_offset) * sign;
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
