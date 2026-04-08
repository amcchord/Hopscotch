#pragma once

#include <cstdint>
#include "motor_manager.h"
#include "settings.h"

enum class ArmPosition : uint8_t {
    Forward = 0,
    Center  = 1,
    Backward = 2,
    Jump = 3,
};

enum class CalStep : uint8_t {
    Idle,
    WaitForward,
    WaitCenter,
    WaitBackward,
};

struct ArmInput {
    bool cal_trigger;     // rising edge on calibration channel (only used in cal mode)
    bool move_trigger;    // rising edge from Ch12
    bool jump_trigger;    // rising edge on Ch11 while armed at center -> jump position
    float speed_channel;  // Ch5 normalized: < 0 = slow (30 RPM), > 0 = max speed
    float nudge_channel;  // Ch4 normalized: -1..+1 maps to +/- 1 rotation offset (mirrored)
};

class ArmController {
public:
    void begin(MotorManager* motors);

    void update(const ArmInput& input, float dt_sec);

    void holdPosition();

    // Record current motor positions as the Forward reference.
    // Call AFTER armArmMotors() with arms physically at Forward.
    void setForwardReference();

    void setCalibration(const ArmCalibration& cal) { _cal = cal; }
    const ArmCalibration& getCalibration() const { return _cal; }

    void setSettingsManager(SettingsManager* mgr) { _settings = mgr; }

    float getTargetLeft() const { return _target_left; }
    float getTargetRight() const { return _target_right; }
    float getForwardLeft() const { return _forward_left; }
    float getForwardRight() const { return _forward_right; }
    bool isMoving() const { return _moving; }
    ArmPosition getCurrentPosition() const { return _current_position; }

    void setMaxArmSpeed(float rad_s) { _max_arm_speed = rad_s; }
    void setArmRange(float rad) { (void)rad; }

    float getTargetPosition(MotorRole role) const;
    const char* getStateString() const;
    void printCalTable() const;

    // Calibration mode (disarmed only)
    bool enterCalMode();
    void exitCalMode();
    bool isInCalMode() const { return _cal_mode; }
    CalStep getCalStep() const { return _cal_step; }

    // External override (used by balance controller)
    void setOverrideTargets(float left, float right, float speed);
    void clearOverride();
    bool isOverridden() const { return _override_active; }

private:
    MotorManager* _motors = nullptr;
    SettingsManager* _settings = nullptr;

    ArmCalibration _cal = {};
    float _target_left = 0.0f;
    float _target_right = 0.0f;
    float _max_arm_speed = 5.0f;
    bool _initialized = false;
    bool _moving = false;

    // Forward reference: absolute motor positions when arms are at Forward.
    // All movement targets are computed as forward_ref + calibrated delta.
    float _forward_left = 0.0f;
    float _forward_right = 0.0f;

    // Calibration forward reference (recorded during cal step 1)
    float _cal_fwd_left = 0.0f;
    float _cal_fwd_right = 0.0f;

    // Sequential position cycling: Fwd -> Ctr -> Bwd -> Ctr -> Fwd ...
    ArmPosition _current_position = ArmPosition::Forward;
    bool _cycle_direction_forward = true;

    // Calibration mode
    bool _cal_mode = false;
    CalStep _cal_step = CalStep::Idle;

    // External override
    bool  _override_active = false;
    float _override_left   = 0.0f;
    float _override_right  = 0.0f;
    float _override_speed  = 0.0f;

    mutable char _state_buf[24] = {};

    void getCalPositions(ArmPosition pos, float& left, float& right) const;
    ArmPosition advancePosition();
};
