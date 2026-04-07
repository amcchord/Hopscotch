#pragma once

#include <cstdint>
#include "driver/twai.h"

// Robstride private protocol communication types (encoded in CAN ID bits [28:24])
enum class RobstrideCommType : uint8_t {
    ObtainID    = 0,
    Control     = 1,   // MIT-style composite control
    Feedback    = 2,
    Enable      = 3,
    Stop        = 4,
    SetZero     = 6,
    SetID       = 7,
    ParamRead   = 17,
    ParamWrite  = 18,
    Fault       = 21,
};

// Motor run modes reported in feedback
enum class RobstrideRunMode : uint8_t {
    MIT       = 0,
    Position  = 1,
    Speed     = 2,
    Current   = 3,
};

// Common parameter addresses
namespace RobstrideParam {
    static constexpr uint16_t RUN_MODE        = 0x7005;
    static constexpr uint16_t SPEED_LIMIT     = 0x7017;
    static constexpr uint16_t CURRENT_LIMIT   = 0x7018;
    static constexpr uint16_t TARGET_POSITION = 0x7016;
    static constexpr uint16_t TARGET_SPEED    = 0x700A;
    static constexpr uint16_t TARGET_CURRENT  = 0x7006;
    static constexpr uint16_t POSITION_KP     = 0x7019;
    static constexpr uint16_t SPEED_KP        = 0x701A;
    static constexpr uint16_t SPEED_KI        = 0x701B;
    static constexpr uint16_t CAN_ID          = 0x7014;
    static constexpr uint16_t MOTOR_POSITION  = 0x7000;
    static constexpr uint16_t MOTOR_SPEED     = 0x7001;
    static constexpr uint16_t MOTOR_TORQUE    = 0x7002;
    static constexpr uint16_t MOTOR_TEMP      = 0x7003;
}

// Parsed feedback from a motor
struct RobstrideFeedback {
    uint8_t motor_id;
    float   position;     // radians
    float   velocity;     // rad/s
    float   torque;       // Nm
    float   temperature;  // Celsius
    uint8_t mode;
    uint16_t errors;
    bool    has_fault;
};

class Robstride {
public:
    bool begin(int tx_pin, int rx_pin);
    void end();

    // Build the 29-bit extended CAN ID
    static uint32_t makeCanId(RobstrideCommType type, uint16_t data16, uint8_t target_id, uint8_t host_id);

    // High-level commands
    bool enableMotor(uint8_t motor_id, uint8_t host_id);
    bool stopMotor(uint8_t motor_id, uint8_t host_id, bool clear_fault = false);

    // MIT-style composite control: position + velocity + kp + kd + torque
    bool sendMITControl(uint8_t motor_id, uint8_t host_id,
                        float position, float velocity,
                        float kp, float kd, float torque);

    // Parameter read/write (for position mode, speed mode, etc.)
    bool writeFloatParam(uint8_t motor_id, uint8_t host_id, uint16_t param_addr, float value);
    bool writeU8Param(uint8_t motor_id, uint8_t host_id, uint16_t param_addr, uint8_t value);

    // Set motor run mode (0=MIT, 1=Position, 2=Speed, 3=Current)
    bool setRunMode(uint8_t motor_id, uint8_t host_id, RobstrideRunMode mode);

    // Position mode convenience: sets target position and speed limit
    bool sendPositionCommand(uint8_t motor_id, uint8_t host_id,
                             float target_pos_rad, float speed_limit_rad_s);

    // Speed mode convenience
    bool sendSpeedCommand(uint8_t motor_id, uint8_t host_id,
                          float target_speed_rad_s, float current_limit_a);

    // Change a motor's CAN ID (requires power cycle)
    bool changeMotorCanId(uint8_t current_id, uint8_t host_id, uint8_t new_id);

    // Set current position as mechanical zero
    bool setZeroPosition(uint8_t motor_id, uint8_t host_id);

    // Receive and parse feedback (non-blocking, returns false if no message)
    bool receiveFeedback(RobstrideFeedback& fb, uint32_t timeout_ms = 1);

private:
    bool sendFrame(uint32_t ext_id, const uint8_t* data, uint8_t len);
    bool _initialized = false;
};
