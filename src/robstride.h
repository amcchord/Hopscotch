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

// Motor run modes (written to RUN_MODE parameter 0x7005)
enum class RobstrideRunMode : uint8_t {
    MIT       = 0,   // Operation control mode
    PP        = 1,   // Profile Position (trapezoidal trajectory, no dynamic speed change)
    Speed     = 2,   // Velocity closed-loop
    Current   = 3,   // Current (torque) mode
    CSP       = 5,   // Cyclic Synchronous Position (real-time position updates)
};

// Parameter addresses per RS00 User Manual parameter index table
namespace RobstrideParam {
    // Writable control parameters
    static constexpr uint16_t RUN_MODE        = 0x7005;  // uint8: 0=MIT, 1=PP, 2=Speed, 3=Current, 5=CSP
    static constexpr uint16_t TARGET_CURRENT  = 0x7006;  // float: iq_ref, -16 to 16A
    static constexpr uint16_t TARGET_SPEED    = 0x700A;  // float: spd_ref, -33 to 33 rad/s
    static constexpr uint16_t TORQUE_LIMIT    = 0x700B;  // float: limit_torque, 0 to 14 Nm
    static constexpr uint16_t TARGET_POSITION = 0x7016;  // float: loc_ref, position in rad
    static constexpr uint16_t SPEED_LIMIT     = 0x7017;  // float: limit_spd (CSP mode speed limit), 0 to 33 rad/s
    static constexpr uint16_t CURRENT_LIMIT   = 0x7018;  // float: limit_cur, 0 to 16A
    static constexpr uint16_t POSITION_KP     = 0x701E;  // float: loc_kp, default 40
    static constexpr uint16_t SPEED_KP        = 0x701F;  // float: spd_kp, default 6
    static constexpr uint16_t SPEED_KI        = 0x7020;  // float: spd_ki, default 0.02
    static constexpr uint16_t SPEED_FILT_GAIN = 0x7021;  // float: spd_filt_gain, default 0.1
    static constexpr uint16_t ACC_RAD         = 0x7022;  // float: velocity mode acceleration, default 20 rad/s^2
    static constexpr uint16_t VEL_MAX         = 0x7024;  // float: PP mode speed, default 10 rad/s
    static constexpr uint16_t ACC_SET         = 0x7025;  // float: PP mode acceleration, default 10 rad/s^2

    // Read-only feedback parameters
    static constexpr uint16_t MECH_POS        = 0x7019;  // float: load mechanical angle (rad) R/O
    static constexpr uint16_t IQ_FILT         = 0x701A;  // float: iq filter value (A) R/O
    static constexpr uint16_t MECH_VEL        = 0x701B;  // float: load speed (rad/s) R/O
    static constexpr uint16_t VBUS            = 0x701C;  // float: bus voltage (V) R/O

    // Legacy aliases used by other parts of the code
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

    // Populated when the response is a param-read reply (comm_type 17)
    bool     is_param_response = false;
    uint16_t param_addr  = 0;
    float    param_value = 0.0f;
};

class Robstride {
public:
    bool begin(int tx_pin, int rx_pin);
    void end();

    // Build the 29-bit extended CAN ID
    static uint32_t makeCanId(RobstrideCommType type, uint16_t data16, uint8_t target_id, uint8_t host_id);

    // Broadcast device discovery (ObtainID type 0, target=0)
    bool broadcastScan(uint8_t host_id);

    // Send motion control ping with all zeros to provoke feedback
    bool sendMotionPing(uint8_t motor_id);

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

    // Read a single parameter from a motor (provokes a response)
    bool readParam(uint8_t motor_id, uint8_t host_id, uint16_t param_addr);

    // Read a parameter synchronously: sends request, blocks up to timeout_ms
    // for the response, and returns the float value. Returns true on success.
    bool readParamSync(uint8_t motor_id, uint8_t host_id, uint16_t param_addr,
                       float& out_value, uint32_t timeout_ms = 50);

    // Change a motor's CAN ID (requires power cycle)
    bool changeMotorCanId(uint8_t current_id, uint8_t host_id, uint8_t new_id);

    // Set current position as mechanical zero
    bool setZeroPosition(uint8_t motor_id, uint8_t host_id);

    // Receive and parse feedback (non-blocking, returns false if no message)
    bool receiveFeedback(RobstrideFeedback& fb, uint32_t timeout_ms = 1);

    // Print TWAI bus status and error counters for diagnostics
    void printBusStatus();

    uint32_t tx_ok_count = 0;
    uint32_t tx_fail_count = 0;
    uint32_t rx_count = 0;

    // Debug: store last N raw received frames for later inspection
    static constexpr int RX_LOG_SIZE = 16;
    struct RxLogEntry {
        uint32_t id;
        uint8_t  dlc;
        uint8_t  data[8];
        bool     extd;
        bool     used;
    };
    RxLogEntry rx_log[RX_LOG_SIZE] = {};
    int rx_log_idx = 0;
    void printRxLog();

private:
    bool sendFrame(uint32_t ext_id, const uint8_t* data, uint8_t len);
    bool _initialized = false;
};
