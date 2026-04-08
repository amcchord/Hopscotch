#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------

// CAN bus via Atomic CAN Base (CA-IS3050G transceiver)
static constexpr int PIN_CAN_TX = 5;
static constexpr int PIN_CAN_RX = 6;

// ELRS CRSF receiver (Grove port)
static constexpr int PIN_CRSF_RX = 1;   // ESP RX <- Receiver TX
static constexpr int PIN_CRSF_TX = 2;   // ESP TX -> Receiver RX

// ---------------------------------------------------------------------------
// CAN bus
// ---------------------------------------------------------------------------
static constexpr uint32_t CAN_BITRATE = 1000000;  // 1 Mbps
static constexpr uint8_t  CAN_HOST_ID = 0xFD;     // Our master/host ID on bus

// ---------------------------------------------------------------------------
// Motor CAN IDs  (defaults, overridable via settings)
// ---------------------------------------------------------------------------
static constexpr uint8_t DEFAULT_MOTOR_ID_FRONT_RIGHT = 10;
static constexpr uint8_t DEFAULT_MOTOR_ID_BACK_RIGHT  = 20;
static constexpr uint8_t DEFAULT_MOTOR_ID_BACK_LEFT   = 30;
static constexpr uint8_t DEFAULT_MOTOR_ID_FRONT_LEFT  = 40;
static constexpr uint8_t DEFAULT_MOTOR_ID_ARM_LEFT    = 1;
static constexpr uint8_t DEFAULT_MOTOR_ID_ARM_RIGHT   = 2;

// ---------------------------------------------------------------------------
// CRSF / ELRS
// ---------------------------------------------------------------------------
static constexpr uint32_t CRSF_BAUDRATE       = 420000;
static constexpr uint16_t CRSF_CHANNEL_MIN    = 172;
static constexpr uint16_t CRSF_CHANNEL_MID    = 992;
static constexpr uint16_t CRSF_CHANNEL_MAX    = 1811;
static constexpr uint32_t CRSF_TIMEOUT_MS     = 500;   // signal-loss threshold

// ---------------------------------------------------------------------------
// Default channel mapping (0-indexed CRSF channel numbers)
// ---------------------------------------------------------------------------
static constexpr uint8_t DEFAULT_CH_STEERING         = 0;
static constexpr uint8_t DEFAULT_CH_THROTTLE         = 1;
static constexpr uint8_t DEFAULT_CH_ARM_DRIVE        = 9;   // Channel 10
static constexpr uint8_t DEFAULT_CH_ARM_ARMS         = 8;   // Channel 9 (lighted button)
static constexpr uint8_t DEFAULT_CH_ARM_LEFT         = 12;  // Channel 13 (knob)
static constexpr uint8_t DEFAULT_CH_ARM_RIGHT        = 13;  // Channel 14 (knob)
static constexpr uint8_t DEFAULT_CH_ARM_MODE         = 5;   // Channel 6  (3-way switch)
static constexpr uint8_t DEFAULT_CH_ARM_SELECT_GROUP = 5;   // Channel 6  (3-way switch)
static constexpr uint8_t DEFAULT_CH_ARM_SELECT_VAR   = 6;   // Channel 7  (3-way switch)
static constexpr uint8_t DEFAULT_CH_ARM_TRIGGER_EXEC = 10;  // Channel 11 (trigger button)
static constexpr uint8_t DEFAULT_CH_ARM_TRIGGER_HOME = 11;  // Channel 12 (trigger button)

// ---------------------------------------------------------------------------
// Control loop timing
// ---------------------------------------------------------------------------
static constexpr uint32_t CONTROL_LOOP_HZ       = 50;
static constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 1000 / CONTROL_LOOP_HZ;  // 20 ms
static constexpr uint32_t DISPLAY_PERIOD_MS      = 40;   // ~25 fps
static constexpr uint32_t WEBSOCKET_PERIOD_MS    = 100;  // ~10 Hz

// ---------------------------------------------------------------------------
// Drive defaults
// ---------------------------------------------------------------------------
static constexpr float DEFAULT_MAX_DRIVE_SPEED_RAD_S = 33.0f;  // RS05 max: 33 rad/s (~315 RPM)
static constexpr float DEFAULT_POSITION_HORIZON_SEC  = 3.0f;
static constexpr uint16_t DEFAULT_DEADBAND           = 50;   // raw CRSF units

// ---------------------------------------------------------------------------
// WiFi defaults (overridable via settings.json)
// ---------------------------------------------------------------------------
static const char* DEFAULT_WIFI_SSID     = "Hopscotch";
static const char* DEFAULT_WIFI_PASSWORD = "hopscotch";

// ---------------------------------------------------------------------------
// Robstride motor specs
// ---------------------------------------------------------------------------
struct MotorSpec {
    float max_torque;
    float max_speed;
    float max_kp;
    float max_kd;
};

static constexpr MotorSpec SPEC_RS00 = { 17.0f, 50.0f, 500.0f, 5.0f };
static constexpr MotorSpec SPEC_RS05 = { 17.0f, 33.0f, 500.0f, 5.0f };

// ---------------------------------------------------------------------------
// Arm speed / nudge / jump
// ---------------------------------------------------------------------------
static constexpr uint8_t  CH_ARM_SPEED           = 4;      // Channel 5 (0-indexed)
static constexpr uint8_t  CH_ARM_NUDGE           = 3;      // Channel 4 (0-indexed)
static constexpr float    ARM_SLOW_SPEED_RAD_S   = 3.14159265f;  // 30 RPM
static constexpr float    ARM_MAX_SPEED_RAD_S    = 50.0f;        // RS00 max
static constexpr float    ARM_NUDGE_MAX_RAD      = 2.0f * 3.14159265f;  // 1 full rotation
static constexpr float    ARM_JUMP_DELTA_LEFT    = 4.39f;   // delta from forward ref
static constexpr float    ARM_JUMP_DELTA_RIGHT   = 0.60f;   // delta from forward ref

// ---------------------------------------------------------------------------
// Self-balance mode
// ---------------------------------------------------------------------------
static constexpr float    BALANCE_SETPOINT_DEG          = 90.0f;   // target roll angle on back wheels
static constexpr float    BALANCE_ENGAGE_THRESHOLD_DEG  = 15.0f;   // how close roll must be to engage PID
static constexpr float    BALANCE_BAILOUT_THRESHOLD_DEG = 45.0f;   // disengage if error exceeds this

static constexpr float    BALANCE_ARM_TIP_LEFT          = 2.61f;   // arm delta to tip robot up (left)
static constexpr float    BALANCE_ARM_TIP_RIGHT         = 1.89f;   // arm delta to tip robot up (right)
static constexpr float    BALANCE_ARM_TIP_SPEED         = 1.0f;    // rad/s ramp rate for tip-up
static constexpr float    BALANCE_ARM_RETURN_SPEED      = 0.5f;    // rad/s ramp rate for return to zero

static constexpr float    BALANCE_MAX_DRIVE_SPEED       = 15.0f;   // rad/s speed limit for balance corrections

static constexpr float    BALANCE_OUTER_KP              = 3.0f;
static constexpr float    BALANCE_OUTER_KI              = 0.3f;
static constexpr float    BALANCE_OUTER_KD              = 0.0f;
static constexpr float    BALANCE_OUTER_I_MAX           = 30.0f;   // integral windup clamp (deg/s)

static constexpr float    BALANCE_INNER_KP              = 0.3f;
static constexpr float    BALANCE_INNER_KI              = 0.0f;
static constexpr float    BALANCE_INNER_KD              = 0.0f;

static constexpr uint32_t BALANCE_LOG_DURATION_MS       = 30000;   // telemetry recording window
static const char*        BALANCE_LOG_PATH              = "/bal_log.csv";

// ---------------------------------------------------------------------------
// CRSF telemetry
// ---------------------------------------------------------------------------
static constexpr uint32_t CRSF_TELEMETRY_PERIOD_MS = 200;  // ~5 Hz
