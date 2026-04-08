#include "arm_controller.h"
#include "config.h"
#include <Arduino.h>
#include <cmath>

static const char* posName(ArmPosition p) {
    switch (p) {
        case ArmPosition::Forward:  return "FORWARD";
        case ArmPosition::Center:   return "CENTER";
        case ArmPosition::Backward: return "BACKWARD";
        case ArmPosition::Jump:     return "JUMP";
    }
    return "?";
}

static const char* calStepName(CalStep s) {
    switch (s) {
        case CalStep::Idle:         return "IDLE";
        case CalStep::WaitForward:  return "WAIT_FORWARD";
        case CalStep::WaitCenter:   return "WAIT_CENTER";
        case CalStep::WaitBackward: return "WAIT_BACKWARD";
    }
    return "?";
}

void ArmController::begin(MotorManager* motors) {
    _motors = motors;
    _target_left = 0.0f;
    _target_right = 0.0f;
    _initialized = false;
    _moving = false;
    _forward_left = 0.0f;
    _forward_right = 0.0f;
    _current_position = ArmPosition::Forward;
    _cycle_direction_forward = true;
    _cal_mode = false;
    _cal_step = CalStep::Idle;
    _override_active = false;
}

// Return absolute motor targets for a given arm position.
// Forward = forward_ref, Center/Backward = forward_ref + calibrated delta.
void ArmController::getCalPositions(ArmPosition pos, float& left, float& right) const {
    switch (pos) {
        case ArmPosition::Forward:
            left = _forward_left;
            right = _forward_right;
            break;
        case ArmPosition::Center:
            left = _forward_left + _cal.center_left;
            right = _forward_right + _cal.center_right;
            break;
        case ArmPosition::Backward:
            left = _forward_left + _cal.backward_left;
            right = _forward_right + _cal.backward_right;
            break;
        case ArmPosition::Jump:
            left = _forward_left + ARM_JUMP_DELTA_LEFT;
            right = _forward_right + ARM_JUMP_DELTA_RIGHT;
            break;
    }
}

void ArmController::printCalTable() const {
    Serial.println("=== ARM CALIBRATION (deltas from Forward) ===");
    Serial.printf("  %-10s  %10s  %10s\n", "Position", "Left (rad)", "Right (rad)");
    Serial.printf("  %-10s  %10.3f  %10.3f\n", "Forward", 0.0f, 0.0f);
    Serial.printf("  %-10s  %10.3f  %10.3f\n", "Center", _cal.center_left, _cal.center_right);
    Serial.printf("  %-10s  %10.3f  %10.3f\n", "Backward", _cal.backward_left, _cal.backward_right);
    Serial.printf("  Calibrated: %s\n", _cal.calibrated ? "YES" : "NO");
    Serial.printf("  Forward ref: left=%.3f right=%.3f\n", _forward_left, _forward_right);
    Serial.println("=============================================");
}

// ---------------------------------------------------------------------------
// Record current motor positions as the Forward reference (software offset).
// Called AFTER armArmMotors() -- no CAN zero needed.
// ---------------------------------------------------------------------------
void ArmController::setForwardReference() {
    if (!_motors) return;

    _forward_left = _motors->getMotor(MotorRole::ArmLeft).position;
    _forward_right = _motors->getMotor(MotorRole::ArmRight).position;

    _target_left = _forward_left;
    _target_right = _forward_right;
    _moving = false;
    _current_position = ArmPosition::Forward;
    _cycle_direction_forward = true;
    _initialized = false;

    Serial.printf("[Arm] Forward reference set: left=%.3f right=%.3f\n",
                  _forward_left, _forward_right);
}

// ---------------------------------------------------------------------------
// Calibration mode entry/exit
// ---------------------------------------------------------------------------
bool ArmController::enterCalMode() {
    if (_motors && _motors->isArmArmed()) {
        Serial.println("[Arm] Cannot enter calibration mode while arms are armed");
        return false;
    }
    _cal_mode = true;
    _cal_step = CalStep::WaitForward;
    Serial.println("[Arm] === CALIBRATION MODE ===");
    Serial.println("[Arm] Step 1/3: Move arms to FORWARD position, then press Ch11");
    return true;
}

void ArmController::exitCalMode() {
    if (_cal_mode) {
        _cal_mode = false;
        _cal_step = CalStep::Idle;
        Serial.println("[Arm] Calibration mode exited");
    }
}

// ---------------------------------------------------------------------------
// Advance to the next position in the cycle:
// Forward -> Center -> Backward -> Center -> Forward -> ...
// ---------------------------------------------------------------------------
ArmPosition ArmController::advancePosition() {
    if (_current_position == ArmPosition::Forward) {
        _cycle_direction_forward = true;
        return ArmPosition::Center;
    }
    if (_current_position == ArmPosition::Backward) {
        _cycle_direction_forward = false;
        return ArmPosition::Center;
    }
    if (_current_position == ArmPosition::Jump) {
        _cycle_direction_forward = false;
        return ArmPosition::Center;
    }
    if (_cycle_direction_forward) {
        return ArmPosition::Backward;
    }
    return ArmPosition::Forward;
}

// ---------------------------------------------------------------------------
// Main update -- called every control tick
// ---------------------------------------------------------------------------
void ArmController::update(const ArmInput& input, float dt_sec) {
    if (!_motors) return;

    // --- Calibration mode (disarmed only) ---
    if (_cal_mode) {
        if (_motors->isArmArmed()) {
            Serial.println("[Arm] Arms armed during calibration -- exiting cal mode");
            exitCalMode();
            return;
        }

        if (input.cal_trigger) {
            float left_pos = _motors->getMotor(MotorRole::ArmLeft).position;
            float right_pos = _motors->getMotor(MotorRole::ArmRight).position;

            switch (_cal_step) {
                case CalStep::WaitForward:
                    _cal_fwd_left = left_pos;
                    _cal_fwd_right = right_pos;
                    Serial.printf("[Arm] FORWARD recorded: left=%.3f right=%.3f\n",
                                  left_pos, right_pos);
                    _cal_step = CalStep::WaitCenter;
                    Serial.println("[Arm] Step 2/3: Move arms to CENTER position, then press Ch11");
                    break;

                case CalStep::WaitCenter:
                    _cal.center_left = left_pos - _cal_fwd_left;
                    _cal.center_right = right_pos - _cal_fwd_right;
                    Serial.printf("[Arm] CENTER delta: left=%.3f right=%.3f\n",
                                  _cal.center_left, _cal.center_right);
                    _cal_step = CalStep::WaitBackward;
                    Serial.println("[Arm] Step 3/3: Move arms to BACKWARD position, then press Ch11");
                    break;

                case CalStep::WaitBackward:
                    _cal.backward_left = left_pos - _cal_fwd_left;
                    _cal.backward_right = right_pos - _cal_fwd_right;
                    _cal.calibrated = true;
                    Serial.printf("[Arm] BACKWARD delta: left=%.3f right=%.3f\n",
                                  _cal.backward_left, _cal.backward_right);

                    if (_settings) {
                        _settings->settings.arm_cal = _cal;
                        _settings->save();
                        Serial.println("[Arm] Calibration saved to flash");
                    }
                    printCalTable();
                    exitCalMode();
                    Serial.println("[Arm] Calibration complete!");
                    break;

                case CalStep::Idle:
                    break;
            }
        }
        return;
    }

    // --- Movement and hold only when armed ---
    if (!_motors->isArmArmed()) return;

    // --- External override (balance controller owns the arms) ---
    if (_override_active) {
        _motors->sendArmPosition(MotorRole::ArmLeft,  _override_left,  _override_speed);
        _motors->sendArmPosition(MotorRole::ArmRight, _override_right, _override_speed);
        _initialized = false;
        return;
    }

    if (!_initialized) {
        _target_left = _motors->getMotor(MotorRole::ArmLeft).position;
        _target_right = _motors->getMotor(MotorRole::ArmRight).position;
        _initialized = true;
        Serial.printf("[Arm] Init: left=%.3f right=%.3f\n", _target_left, _target_right);
    }

    // --- Speed selection from Ch5 ---
    float effective_speed = ARM_MAX_SPEED_RAD_S;
    if (input.speed_channel < 0.0f) {
        effective_speed = ARM_SLOW_SPEED_RAD_S;
    }

    // --- Move trigger (Ch12): advance to next position ---
    if (input.move_trigger) {
        if (!_cal.calibrated) {
            Serial.println("[Arm] Cannot move -- not calibrated. Run 'cal start' first.");
        } else {
            ArmPosition next = advancePosition();
            float goal_left, goal_right;
            getCalPositions(next, goal_left, goal_right);
            _target_left = goal_left;
            _target_right = goal_right;
            _current_position = next;
            _moving = true;
            Serial.printf("[Arm] Moving to %s: left=%.3f right=%.3f (fwd_ref: %.3f, %.3f)\n",
                          posName(next), goal_left, goal_right,
                          _forward_left, _forward_right);
        }
    }

    // --- Jump trigger (Ch11): jump from Center ---
    if (input.jump_trigger && _current_position == ArmPosition::Center && !_moving) {
        float goal_left, goal_right;
        getCalPositions(ArmPosition::Jump, goal_left, goal_right);
        _target_left = goal_left;
        _target_right = goal_right;
        _current_position = ArmPosition::Jump;
        _moving = true;
        Serial.printf("[Arm] JUMP: left=%.3f right=%.3f (fwd_ref: %.3f, %.3f)\n",
                      goal_left, goal_right, _forward_left, _forward_right);
    }

    // --- Nudge offset from Ch4 (mirrored: +left, -right) ---
    float nudge_offset = input.nudge_channel * ARM_NUDGE_MAX_RAD;

    // --- Send position commands (always use effective_speed so Ch5 applies at all times) ---
    float send_left = _target_left + nudge_offset;
    float send_right = _target_right - nudge_offset;

    _motors->sendArmPosition(MotorRole::ArmLeft, send_left, effective_speed);
    _motors->sendArmPosition(MotorRole::ArmRight, send_right, effective_speed);

    if (_moving) {
        float err_l = fabsf(_motors->getMotor(MotorRole::ArmLeft).position - send_left);
        float err_r = fabsf(_motors->getMotor(MotorRole::ArmRight).position - send_right);

        if (err_l < 0.1f && err_r < 0.1f) {
            _moving = false;
            Serial.printf("[Arm] Reached %s\n", posName(_current_position));
        }
    }
}

void ArmController::holdPosition() {
    if (!_motors) return;
    _moving = false;

    _target_left = _motors->getMotor(MotorRole::ArmLeft).position;
    _target_right = _motors->getMotor(MotorRole::ArmRight).position;

    _motors->sendArmPosition(MotorRole::ArmLeft, _target_left, 0.0f);
    _motors->sendArmPosition(MotorRole::ArmRight, _target_right, 0.0f);

    _initialized = false;
}

// ---------------------------------------------------------------------------
// External override (used by balance controller)
// ---------------------------------------------------------------------------

void ArmController::setOverrideTargets(float left, float right, float speed) {
    _override_active = true;
    _override_left   = left;
    _override_right  = right;
    _override_speed  = speed;
}

void ArmController::clearOverride() {
    if (_override_active) {
        _override_active = false;
        _initialized = false;
        Serial.println("[Arm] Override cleared, returning to normal control");
    }
}

float ArmController::getTargetPosition(MotorRole role) const {
    if (role == MotorRole::ArmLeft) return _target_left;
    if (role == MotorRole::ArmRight) return _target_right;
    return 0.0f;
}

const char* ArmController::getStateString() const {
    if (!_motors || !_motors->isArmArmed()) {
        if (_cal_mode) {
            snprintf(_state_buf, sizeof(_state_buf), "CAL:%s", calStepName(_cal_step));
            return _state_buf;
        }
        return "DISARM";
    }
    if (_moving) {
        snprintf(_state_buf, sizeof(_state_buf), "->%s", posName(_current_position));
    } else {
        snprintf(_state_buf, sizeof(_state_buf), "HOLD");
    }
    return _state_buf;
}
