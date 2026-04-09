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
    _arming = {};
    _arming_just_completed = false;
    _arming_completed_drive = false;

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

void MotorManager::configureMotorAfterEnable(int idx, float motor_pos) {
    MotorState& m = _motors[idx];
    m.enabled = true;
    m.run_mode = RobstrideRunMode::CSP;
    m.position = motor_pos;
    m.raw_position = 0.0f;
    m.prev_raw_position = 0.0f;
    m.unwrap_offset = 0.0f;
    m._abs_pos_offset = motor_pos;
    m.has_first_feedback = false;
    Serial.printf("[Motors] Enabled motor ID=%d in CSP mode (pos=%.3f)\n",
                  m.can_id, motor_pos);
}

// ---------------------------------------------------------------------------
// Non-blocking arming: request
// ---------------------------------------------------------------------------

void MotorManager::requestArmDrive() {
    if (!_can || isArming()) return;

    _arming_just_completed = false;
    _arming.step = ArmingStep::StopMotor;
    _arming.first_idx = 0;
    _arming.last_idx = NUM_DRIVE_MOTORS;
    _arming.current_idx = 0;
    _arming.step_start_ms = millis();
    _arming.all_ok = true;
    _arming.is_drive = true;
    _arming.pos_received = false;
    _arming.read_pos = 0.0f;

    Serial.println("[Motors] Arming drive motors (non-blocking)...");
}

void MotorManager::requestArmArms() {
    if (!_can || isArming()) return;

    _arming_just_completed = false;
    _arming.step = ArmingStep::StopMotor;
    _arming.first_idx = NUM_DRIVE_MOTORS;
    _arming.last_idx = NUM_MOTORS;
    _arming.current_idx = NUM_DRIVE_MOTORS;
    _arming.step_start_ms = millis();
    _arming.all_ok = true;
    _arming.is_drive = false;
    _arming.pos_received = false;
    _arming.read_pos = 0.0f;

    Serial.println("[Motors] Arming arm motors (non-blocking)...");
}

// ---------------------------------------------------------------------------
// Non-blocking arming: cancel (stops any already-enabled motors in this batch)
// ---------------------------------------------------------------------------

void MotorManager::cancelArming() {
    if (!isArming()) return;

    Serial.println("[Motors] Arming cancelled");

    for (int i = _arming.first_idx; i < _arming.current_idx; i++) {
        if (_motors[i].enabled) {
            _can->stopMotor(_motors[i].can_id, CAN_HOST_ID, false);
            _motors[i].enabled = false;
        }
    }

    _arming.step = ArmingStep::Idle;
}

// ---------------------------------------------------------------------------
// Non-blocking arming: advance one step per call
// ---------------------------------------------------------------------------

void MotorManager::updateArming() {
    if (!_can) return;

    _arming_just_completed = false;
    uint32_t now = millis();
    uint32_t elapsed = now - _arming.step_start_ms;
    int idx = _arming.current_idx;

    switch (_arming.step) {

    case ArmingStep::Idle:
    case ArmingStep::Complete:
    case ArmingStep::Failed:
        return;

    case ArmingStep::StopMotor: {
        MotorState& m = _motors[idx];
        _can->stopMotor(m.can_id, CAN_HOST_ID, true);
        _arming.step = ArmingStep::WaitStop;
        _arming.step_start_ms = now;
        break;
    }

    case ArmingStep::WaitStop:
        if (elapsed >= ARMING_STOP_DELAY_MS) {
            _arming.step = ArmingStep::SetMode;
            _arming.step_start_ms = now;
        }
        break;

    case ArmingStep::SetMode: {
        MotorState& m = _motors[idx];
        _can->setRunMode(m.can_id, CAN_HOST_ID, RobstrideRunMode::CSP);
        _arming.step = ArmingStep::WaitMode;
        _arming.step_start_ms = now;
        break;
    }

    case ArmingStep::WaitMode:
        if (elapsed >= ARMING_MODE_DELAY_MS) {
            _arming.step = ArmingStep::Enable;
            _arming.step_start_ms = now;
        }
        break;

    case ArmingStep::Enable: {
        MotorState& m = _motors[idx];
        bool ok = _can->enableMotor(m.can_id, CAN_HOST_ID);
        if (!ok) {
            Serial.printf("[Motors] Failed to enable motor ID=%d\n", m.can_id);
            _arming.all_ok = false;
            _arming.step = ArmingStep::NextMotor;
            _arming.step_start_ms = now;
        } else {
            _arming.step = ArmingStep::WaitEnable;
            _arming.step_start_ms = now;
        }
        break;
    }

    case ArmingStep::WaitEnable:
        if (elapsed >= ARMING_ENABLE_DELAY_MS) {
            _arming.step = ArmingStep::ReadPos;
            _arming.step_start_ms = now;
        }
        break;

    case ArmingStep::ReadPos: {
        MotorState& m = _motors[idx];
        _arming.pos_received = false;
        _arming.read_pos = 0.0f;
        _can->readParam(m.can_id, CAN_HOST_ID, RobstrideParam::MECH_POS);
        _arming.step = ArmingStep::WaitReadPos;
        _arming.step_start_ms = now;
        break;
    }

    case ArmingStep::WaitReadPos: {
        if (_arming.pos_received) {
            float motor_pos = _arming.read_pos;
            configureMotorAfterEnable(idx, motor_pos);

            _can->writeFloatParam(_motors[idx].can_id, CAN_HOST_ID,
                                  RobstrideParam::SPEED_LIMIT, 0.0f);
            _can->writeFloatParam(_motors[idx].can_id, CAN_HOST_ID,
                                  RobstrideParam::TARGET_POSITION, motor_pos);

            _arming.step = ArmingStep::WaitConfigure;
            _arming.step_start_ms = now;
        } else if (elapsed >= ARMING_READ_POS_TIMEOUT_MS) {
            Serial.printf("[Motors] WARNING: MECH_POS timeout for motor %d, assuming 0\n",
                          _motors[idx].can_id);
            configureMotorAfterEnable(idx, 0.0f);

            _can->writeFloatParam(_motors[idx].can_id, CAN_HOST_ID,
                                  RobstrideParam::SPEED_LIMIT, 0.0f);
            _can->writeFloatParam(_motors[idx].can_id, CAN_HOST_ID,
                                  RobstrideParam::TARGET_POSITION, 0.0f);

            _arming.step = ArmingStep::WaitConfigure;
            _arming.step_start_ms = now;
        }
        break;
    }

    case ArmingStep::WaitConfigure:
        if (elapsed >= ARMING_CONFIGURE_DELAY_MS) {
            _arming.step = ArmingStep::NextMotor;
            _arming.step_start_ms = now;
        }
        break;

    case ArmingStep::NextMotor: {
        _arming.current_idx++;
        if (_arming.current_idx >= _arming.last_idx) {
            if (_arming.is_drive) {
                _drive_armed = _arming.all_ok;
                Serial.printf("[Motors] Drive motors armed: %s\n",
                              _arming.all_ok ? "OK" : "PARTIAL");
            } else {
                _arm_armed = _arming.all_ok;
                Serial.printf("[Motors] Arm motors armed: %s\n",
                              _arming.all_ok ? "OK" : "PARTIAL");
            }
            _arming.step = ArmingStep::Complete;
            _arming_just_completed = true;
            _arming_completed_drive = _arming.is_drive;
        } else {
            if (elapsed >= ARMING_INTER_MOTOR_DELAY_MS) {
                _arming.step = ArmingStep::StopMotor;
                _arming.step_start_ms = now;
            }
        }
        break;
    }

    } // switch
}

// ---------------------------------------------------------------------------
// Disarm (always sends stop commands, regardless of armed state)
// ---------------------------------------------------------------------------

void MotorManager::disarmDriveMotors() {
    if (!_can) return;

    for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
        _can->stopMotor(_motors[i].can_id, CAN_HOST_ID, false);
        _motors[i].enabled = false;
    }
    _drive_armed = false;
    Serial.println("[Motors] Drive motors disarmed");
}

void MotorManager::disarmArmMotors() {
    if (!_can) return;

    for (int i = NUM_DRIVE_MOTORS; i < NUM_MOTORS; i++) {
        _can->stopMotor(_motors[i].can_id, CAN_HOST_ID, false);
        _motors[i].enabled = false;
    }
    _arm_armed = false;
    Serial.println("[Motors] Arm motors disarmed");
}

void MotorManager::disarmAll() {
    disarmDriveMotors();
    disarmArmMotors();
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

        // Handle param-read responses
        if (fb.is_param_response) {
            if (fb.param_addr == RobstrideParam::MECH_POS &&
                _arming.step == ArmingStep::WaitReadPos &&
                findMotorByCanId(fb.motor_id) == _arming.current_idx) {
                _arming.read_pos = fb.param_value;
                _arming.pos_received = true;
            } else if (fb.param_addr == RobstrideParam::VBUS) {
                if (_bus_voltage == 0.0f && fb.param_value > 0.0f) {
                    Serial.printf("[Motors] First VBUS reading: %.1fV\n", fb.param_value);
                }
                _bus_voltage = fb.param_value;
            } else if (fb.param_addr == RobstrideParam::IQ_FILT) {
                int idx = findMotorByCanId(fb.motor_id);
                if (idx >= 0) {
                    _motor_current[idx] = fb.param_value;
                }
            }
            continue;
        }

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

    // Scan cycle: 6 motion pings + 1 VBUS read + 1 IQ_FILT read = 8 slots
    static constexpr int SLOT_VBUS = NUM_MOTORS;
    static constexpr int SLOT_IQ   = NUM_MOTORS + 1;
    static constexpr int TOTAL_SLOTS = NUM_MOTORS + 2;

    if (_scan_index < NUM_MOTORS) {
        MotorState& m = _motors[_scan_index];
        _can->sendMotionPing(m.can_id);
    } else if (_scan_index == SLOT_VBUS) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (_motors[i].online) {
                _can->readParam(_motors[i].can_id, CAN_HOST_ID, RobstrideParam::VBUS);
                break;
            }
        }
    } else if (_scan_index == SLOT_IQ) {
        // Read IQ_FILT from one motor per cycle, rotating through all motors
        for (int attempt = 0; attempt < NUM_MOTORS; attempt++) {
            int idx = (_iq_scan_index + attempt) % NUM_MOTORS;
            if (_motors[idx].online) {
                _can->readParam(_motors[idx].can_id, CAN_HOST_ID, RobstrideParam::IQ_FILT);
                _iq_scan_index = (idx + 1) % NUM_MOTORS;
                break;
            }
        }
    }

    _scan_index = (_scan_index + 1) % TOTAL_SLOTS;
}

float MotorManager::getTotalCurrent() const {
    float sum = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (_motors[i].online) {
            float abs_current = _motor_current[i];
            if (abs_current < 0.0f) abs_current = -abs_current;
            sum += abs_current;
        }
    }
    return sum;
}

bool MotorManager::changeMotorCanIdOnBus(uint8_t old_id, uint8_t new_id) {
    if (!_can) return false;
    return _can->changeMotorCanId(old_id, CAN_HOST_ID, new_id);
}
