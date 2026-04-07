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
static constexpr uint8_t DEFAULT_CH_STEERING    = 0;
static constexpr uint8_t DEFAULT_CH_THROTTLE    = 1;
static constexpr uint8_t DEFAULT_CH_ARM_DRIVE   = 9;   // Channel 10
static constexpr uint8_t DEFAULT_CH_ARM_ARMS    = 4;   // Channel 5
static constexpr uint8_t DEFAULT_CH_ARM_LEFT    = 12;  // Channel 13
static constexpr uint8_t DEFAULT_CH_ARM_RIGHT   = 13;  // Channel 14

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
static constexpr float DEFAULT_MAX_DRIVE_SPEED_RAD_S = 10.0f;
static constexpr float DEFAULT_POSITION_HORIZON_SEC  = 2.0f;
static constexpr uint16_t DEFAULT_DEADBAND           = 50;   // raw CRSF units

// ---------------------------------------------------------------------------
// WiFi defaults (overridable via settings.json)
// ---------------------------------------------------------------------------
static const char* DEFAULT_WIFI_SSID     = "SvensHaus";
static const char* DEFAULT_WIFI_PASSWORD = "montreal19";

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
