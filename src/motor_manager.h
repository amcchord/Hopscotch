#pragma once

#include <cstdint>
#include "robstride.h"
#include "config.h"

static constexpr int NUM_DRIVE_MOTORS = 4;
static constexpr int NUM_ARM_MOTORS   = 2;
static constexpr int NUM_MOTORS       = NUM_DRIVE_MOTORS + NUM_ARM_MOTORS;

enum class MotorRole : uint8_t {
    FrontRight = 0,
    BackRight  = 1,
    BackLeft   = 2,
    FrontLeft  = 3,
    ArmLeft    = 4,
    ArmRight   = 5,
};

struct MotorState {
    uint8_t  can_id;
    MotorRole role;
    bool     reversed;       // left-side drive motors are reversed
    bool     online;
    bool     enabled;
    bool     has_fault;
    float    position;       // rad
    float    velocity;       // rad/s
    float    torque;         // Nm
    float    temperature;    // C
    uint16_t errors;
    uint32_t last_feedback_ms;
    RobstrideRunMode run_mode;
};

class MotorManager {
public:
    void begin(Robstride* can_bus);

    // Configure CAN IDs from settings
    void setMotorId(MotorRole role, uint8_t can_id);
    uint8_t getMotorId(MotorRole role) const;

    // Arm / disarm
    bool armDriveMotors();
    bool disarmDriveMotors();
    bool armArmMotors();
    bool disarmArmMotors();
    bool disarmAll();

    bool isDriveArmed() const { return _drive_armed; }
    bool isArmArmed() const { return _arm_armed; }

    // Send position command to a drive motor (handles reversal)
    bool sendDrivePosition(MotorRole role, float position_rad, float speed_limit_rad_s);

    // Send position command to an arm motor
    bool sendArmPosition(MotorRole role, float position_rad, float speed_limit_rad_s);

    // Process incoming CAN feedback (call frequently)
    void processFeedback();

    // Mark motors offline if no feedback within timeout
    void checkTimeouts(uint32_t timeout_ms = 500);

    // Change a motor's CAN ID on the bus
    bool changeMotorCanIdOnBus(uint8_t old_id, uint8_t new_id);

    // Access motor state
    const MotorState& getMotor(MotorRole role) const { return _motors[static_cast<int>(role)]; }
    const MotorState& getMotor(int index) const { return _motors[index]; }
    int motorCount() const { return NUM_MOTORS; }

private:
    Robstride* _can = nullptr;
    MotorState _motors[NUM_MOTORS];
    bool _drive_armed = false;
    bool _arm_armed = false;

    int findMotorByCanId(uint8_t can_id);
    bool enableAndConfigureMotor(int idx, RobstrideRunMode mode);
};
