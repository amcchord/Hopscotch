#pragma once

#include <cstdint>
#include "robstride.h"
#include "config.h"

static constexpr int NUM_DRIVE_MOTORS = 4;
static constexpr int NUM_ARM_MOTORS   = 2;
static constexpr int NUM_MOTORS       = NUM_DRIVE_MOTORS + NUM_ARM_MOTORS;

// Robstride feedback position is encoded in [-4pi, +4pi] and wraps.
// Full range for unwrapping.
static constexpr float POSITION_FEEDBACK_RANGE = 4.0f * 3.14159265f * 2.0f;  // ~25.133 rad
static constexpr float POSITION_FEEDBACK_HALF  = POSITION_FEEDBACK_RANGE / 2.0f;

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
    float    position;       // rad (unwrapped, continuous)
    float    raw_position;   // rad (raw from feedback, [-4pi, +4pi])
    float    prev_raw_position; // previous raw position for unwrap detection
    float    unwrap_offset;  // cumulative unwrap offset
    float    _abs_pos_offset; // motor's absolute position at arm time
    bool     has_first_feedback; // true after first feedback received
    float    velocity;       // rad/s
    float    torque;         // Nm
    float    temperature;    // C
    uint16_t errors;
    uint32_t last_feedback_ms;
    RobstrideRunMode run_mode;
};

// Non-blocking arming state machine
enum class ArmingStep : uint8_t {
    Idle,
    StopMotor,
    WaitStop,
    SetMode,
    WaitMode,
    Enable,
    WaitEnable,
    ReadPos,
    WaitReadPos,
    WaitConfigure,
    NextMotor,
    Complete,
    Failed,
};

struct ArmingContext {
    ArmingStep step = ArmingStep::Idle;
    int first_idx = 0;
    int last_idx = 0;
    int current_idx = 0;
    uint32_t step_start_ms = 0;
    bool all_ok = true;
    bool is_drive = false;
    float read_pos = 0.0f;
    bool pos_received = false;
};

// Timing for non-blocking arming steps (ms)
static constexpr uint32_t ARMING_STOP_DELAY_MS       = 50;
static constexpr uint32_t ARMING_MODE_DELAY_MS        = 5;
static constexpr uint32_t ARMING_ENABLE_DELAY_MS      = 20;
static constexpr uint32_t ARMING_READ_POS_TIMEOUT_MS  = 100;
static constexpr uint32_t ARMING_CONFIGURE_DELAY_MS   = 5;
static constexpr uint32_t ARMING_INTER_MOTOR_DELAY_MS = 10;

class MotorManager {
public:
    void begin(Robstride* can_bus);

    // Configure CAN IDs from settings
    void setMotorId(MotorRole role, uint8_t can_id);
    uint8_t getMotorId(MotorRole role) const;

    // Non-blocking arming: request, update each tick, cancel
    void requestArmDrive();
    void requestArmArms();
    void updateArming();
    void cancelArming();

    bool isArming() const { return _arming.step != ArmingStep::Idle && _arming.step != ArmingStep::Complete && _arming.step != ArmingStep::Failed; }
    bool isArmingDrive() const { return isArming() && _arming.is_drive; }
    bool isArmingArms() const { return isArming() && !_arming.is_drive; }
    bool armingJustCompleted() const { return _arming_just_completed; }
    bool armingJustCompletedDrive() const { return _arming_just_completed && _arming_completed_drive; }
    bool armingJustCompletedArms() const { return _arming_just_completed && !_arming_completed_drive; }
    void clearArmingCompleted() { _arming_just_completed = false; }

    // Disarm (always sends stop commands regardless of armed state)
    void disarmDriveMotors();
    void disarmArmMotors();
    void disarmAll();

    bool isDriveArmed() const { return _drive_armed; }
    bool isArmArmed() const { return _arm_armed; }

    // Send position command to a drive motor (handles reversal)
    bool sendDrivePosition(MotorRole role, float position_rad, float speed_limit_rad_s);

    // Send only the speed limit to a drive motor (for braking, no target position write)
    bool sendDriveSpeedLimit(MotorRole role, float speed_limit_rad_s);

    // Send position command to an arm motor
    bool sendArmPosition(MotorRole role, float position_rad, float speed_limit_rad_s);

    // Process incoming CAN feedback (call frequently)
    void processFeedback();

    // Mark motors offline if no feedback within timeout
    void checkTimeouts(uint32_t timeout_ms = 500);

    // Ping one motor by reading its position parameter (round-robins through motors)
    void scanNextMotor();

    // Change a motor's CAN ID on the bus
    bool changeMotorCanIdOnBus(uint8_t old_id, uint8_t new_id);

    // Access motor state
    const MotorState& getMotor(MotorRole role) const { return _motors[static_cast<int>(role)]; }
    const MotorState& getMotor(int index) const { return _motors[index]; }
    int motorCount() const { return NUM_MOTORS; }

    // Bus voltage read from motors (all share the same power bus)
    float getBusVoltage() const { return _bus_voltage; }

    // Sum of absolute motor currents (amps) from IQ_FILT param reads
    float getTotalCurrent() const;

    Robstride* getCanBus() { return _can; }

private:
    Robstride* _can = nullptr;
    MotorState _motors[NUM_MOTORS];
    bool _drive_armed = false;
    bool _arm_armed = false;

    ArmingContext _arming;
    bool _arming_just_completed = false;
    bool _arming_completed_drive = false;

    int findMotorByCanId(uint8_t can_id);
    void configureMotorAfterEnable(int idx, float motor_pos);
    int _scan_index = 0;

    float _bus_voltage = 0.0f;
    float _motor_current[NUM_MOTORS] = {};
    int   _iq_scan_index = 0;
};
