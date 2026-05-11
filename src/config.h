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
static constexpr uint32_t CONTROL_LOOP_HZ        = 50;
static constexpr uint32_t CONTROL_LOOP_PERIOD_MS  = 1000 / CONTROL_LOOP_HZ;  // 20 ms
static constexpr uint32_t BALANCE_LOOP_HZ         = 200;
static constexpr uint32_t BALANCE_LOOP_PERIOD_US  = 1000000 / BALANCE_LOOP_HZ;  // 5000 us
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
//
// Two cascaded control loops:
//
//   OUTER LOOP (50Hz, Core 1) -- Position PI
//     Input:  wheel odometry (rad) from engage origin
//     Output: balance angle offset (deg) added to scheduled setpoint
//     Goal:   keep robot near its starting position on the ground
//
//   INNER LOOP (200Hz, Core 0) -- Balance PD
//     Input:  tilt angle (deg) from complementary filter, gyro rate (deg/s)
//     Output: motor velocity command (rad/s) integrated into wheel position targets
//     Goal:   keep robot upright at the effective setpoint angle
//
//   effective_setpoint = arm_scheduled_base + capture_shift + pos_shift
//
// ---------------------------------------------------------------------------
static constexpr float    BALANCE_SETPOINT_ARMS_FWD     = 91.0f;   // balance point with arms at forward ref
static constexpr float    BALANCE_SETPOINT_ARMS_TIP     = 86.5f;   // balance point with arms at tip position
static constexpr float    BALANCE_SETPOINT_MIN          = 70.0f;   // hard safety clamp (fallen forward)
static constexpr float    BALANCE_SETPOINT_MAX          = 110.0f;  // hard safety clamp (fallen backward)
static constexpr float    BALANCE_VEL_GAIN              = 0.5f;    // deg per (rad/s) per second of command integration (trim only)
static constexpr float    BALANCE_SETPOINT_RATE_MAX     = 2.0f;    // max deg/s trim can change
static constexpr float    BALANCE_TRIM_MAX_DEG          = 3.0f;    // trim clamp -- reduced to limit positive feedback with position PI
static constexpr float    BALANCE_TRIM_DECAY             = 0.99f;  // per-tick decay when position PI active (~1.5s half-life at 50Hz)
static constexpr float    BALANCE_CAPTURE_SHIFT_MAX_DEG = 15.0f;   // cover large engage angle differences
static constexpr bool     BALANCE_USE_SCHEDULED_SP      = true;    // if false, base setpoint stays at ARMS_FWD (92) always
static constexpr bool     BALANCE_USE_CAPTURE_SHIFT     = true;    // if false, no capture shift -- engage directly at scheduled sp
static constexpr float    BALANCE_BASE_SP_RATE_MAX      = 3.0f;    // slower ramp: less overshoot momentum at ramp end (5 caused 6-deg overshoot)
static constexpr float    BALANCE_ENGAGE_THRESHOLD_DEG  = 15.0f;   // wide enough for tip position
static constexpr float    BALANCE_ENGAGE_RATE_MAX_DPS   = 50.0f;   // max roll rate to engage
static constexpr float    BALANCE_BAILOUT_THRESHOLD_DEG = 45.0f;   // disengage if error exceeds this
static constexpr float    BALANCE_CAPTURE_ERR_MAX_DEG   = 1.0f;    // must be this close to effective setpoint before arm return
static constexpr float    BALANCE_CAPTURE_RATE_MAX_DPS  = 4.0f;    // max roll rate for a "captured" balance state
static constexpr float    BALANCE_CAPTURE_CMD_MAX       = 1.0f;    // max PD command magnitude for a "captured" state
static constexpr uint32_t BALANCE_CAPTURE_SETTLE_MS     = 400;     // capture must stay quiet this long before arm return
static constexpr uint32_t BALANCE_ARM_HOLD_MAX_MS       = 1000;    // start returning arms quickly (was 2500 -- too long at unstable tip)

static constexpr float    BALANCE_ARM_TIP_LEFT          = 2.71f;   // arm delta to tip robot up (left)
static constexpr float    BALANCE_ARM_TIP_RIGHT         = 1.96f;   // arm delta to tip robot up (right)
static constexpr float    BALANCE_ARM_TIP_SPEED         = 0.7f;    // rad/s ramp rate for tip-up (slower = less overshoot)
static constexpr float    BALANCE_ARM_RETURN_SPEED      = 1.5f;    // rad/s -- gentle return over ~1.7s (10 caused violent overshoot oscillation)

static constexpr float    BALANCE_MAX_DRIVE_SPEED       = 25.0f;   // rad/s speed limit for balance corrections

// Inner loop PD gains (200Hz, Core 0)
//   Input:  angle_err = effective_setpoint - tilt_angle (deg)
//   Input:  gyro_rate (deg/s)
//   Output: motor_vel = Kp * angle_err - Kd * gyro_rate (rad/s)
//   motor_vel is integrated into wheel position targets each tick
static constexpr float    BALANCE_KP                    = 2.0f;    // rad/s per degree of angle error
static constexpr float    BALANCE_KD                    = 0.08f;   // rad/s per deg/s of roll rate

static constexpr float    COMPLEMENTARY_ALPHA           = 0.996f;  // gyro weight in complementary filter

static constexpr uint32_t BALANCE_LOG_DURATION_MS       = 120000;  // telemetry recording window (flush deferred, safe to be long)
static const char*        BALANCE_LOG_PATH              = "/bal_log.csv";

// Safety abort thresholds
static constexpr float    BALANCE_SAFE_TILT_MIN         = 30.0f;    // hard abort below this (fallen forward)
static constexpr float    BALANCE_SAFE_TILT_MAX         = 150.0f;   // hard abort above this (fallen backward)
static constexpr float    BALANCE_SAFE_ERR_MAX_DEG      = 35.0f;    // sustained effective-error threshold
static constexpr uint32_t BALANCE_SAFE_ERR_DURATION_MS  = 2000;     // must persist this long before abort
static constexpr float    BALANCE_SAFE_RATE_MAX_DPS     = 200.0f;   // extreme roll-rate threshold
static constexpr uint32_t BALANCE_SAFE_RATE_DURATION_MS = 500;      // must persist this long
static constexpr uint32_t BALANCE_SAFE_SAT_DURATION_MS  = 3000;     // motor saturated this long -> disengage

// Outer position PI loop (50Hz, Core 1) -- always active, gated by angle error
//   Input:  meas_drift = avg(back_wheel_pos) - wheel_origin (rad)
//   Input:  meas_vel = avg(back_wheel_velocity) (rad/s)
//   Output: pos_shift (deg) added to effective_setpoint
//   If BALANCE_POS_RESET_ORIGIN_ON_ARM_RETURN is true, wheel origin resets when arms finish
//   returning so PI only corrects post-balance drift. If false, origin stays at engage position.
static constexpr float    BALANCE_POS_KP                = 0.20f;    // proportional: halved from 0.40 to reduce oscillation
static constexpr float    BALANCE_POS_KI                = 0.15f;    // integral: sustained correction for steady-state
static constexpr float    BALANCE_POS_KD                = 0.20f;    // velocity damping: increased from 0.12 to damp oscillation
static constexpr float    BALANCE_POS_SHIFT_MAX_DEG     = 6.0f;     // authority (origin resets at arm return, so less needed)
static constexpr float    BALANCE_POS_SHIFT_RATE_MAX    = 4.0f;     // faster rate for quicker response
static constexpr float    BALANCE_POS_DEADBAND_RAD      = 0.15f;    // react early
static constexpr float    BALANCE_POS_INTEGRAL_MAX      = 200.0f;   // integral clamp
static constexpr float    BALANCE_POS_GATE_ERR_DEG      = 8.0f;     // error at which position authority -> 0
static constexpr bool     BALANCE_POS_RESET_ORIGIN_ON_ARM_RETURN = true;  // reset wheel origin when arms reach forward

// Arm balance assist (active after arms reach forward)
//   Dual-purpose: fast velocity reflex + drift correction.
//   No integral -- vel_trim handles steady-state, arms provide fast physical forces.
//   Arms spring back naturally when velocity and drift are near zero.
static constexpr float    BALANCE_SETPOINT_ARMS_CENTER  = 83.0f;    // balance point at center (measured)
static constexpr float    BALANCE_ARM_BAL_MAX_FRAC      = 0.25f;    // max arm fraction in either direction
static constexpr float    BALANCE_ARM_BAL_FRAC_RATE     = 0.60f;    // max frac change per second (was 0.30, rate-limited 43% of time)
static constexpr float    BALANCE_ARM_BAL_MOTOR_SPEED   = 4.0f;     // rad/s speed limit sent to arm motors (match faster frac rate)
static constexpr float    BALANCE_ARM_VEL_GAIN          = 0.05f;    // frac per rad/s of wheel velocity (fast reflex, was 0.02)
static constexpr float    BALANCE_ARM_DRIFT_GAIN        = 0.08f;    // frac per rad of drift (position correction, was 0.03)
static constexpr float    BALANCE_ARM_BAL_DEADBAND_RAD  = 0.15f;    // ignore drift below this

// Velocity trim: integrator that finds the balance angle producing a target velocity.
//   The odometry PID computes a target velocity to return to origin.
//   The vel_trim integrates (filtered_vel - target_vel) to find the setpoint
//   that makes the wheels move at that target velocity.
static constexpr float    BALANCE_VEL_TRIM_GAIN         = 0.15f;    // deg per (rad/s_error * s) -- calm, won't whip during oscillation
static constexpr float    BALANCE_VEL_TRIM_MAX_DEG      = 4.0f;     // max trim clamp
static constexpr float    BALANCE_VEL_TRIM_FILTER       = 0.01f;    // velocity filter alpha (~2s time constant, averages out oscillation)
static constexpr float    BALANCE_VEL_TRIM_GATE_DEG     = 1.5f;     // very tight gate: only integrate when truly balanced

// Odometry PID: computes target wheel velocity to return to origin.
//   target_vel = -ODO_KP * drift - ODO_KD * wheel_vel
//   The vel_trim then adjusts setpoint to achieve this target velocity.
static constexpr float    BALANCE_ODO_KP                = 0.10f;    // rad/s per rad of drift (was 0.30 -- too aggressive during oscillation)
static constexpr float    BALANCE_ODO_KD                = 0.05f;    // damping on wheel velocity
static constexpr float    BALANCE_ODO_MAX_VEL           = 0.5f;     // max target velocity (was 2.0 -- gentle return, don't destabilize)

// Stuck / wall detection (uses measured odometry)
static constexpr float    BALANCE_STUCK_CMD_THRESHOLD   = 2.0f;     // |motor_vel| must exceed this
static constexpr float    BALANCE_STUCK_VEL_THRESHOLD   = 0.5f;     // |measured_vel| must be below this
static constexpr uint32_t BALANCE_STUCK_DURATION_MS     = 300;      // condition must persist
static constexpr float    BALANCE_STUCK_INTEGRAL_DECAY  = 0.95f;    // per-tick integral decay when stuck
static constexpr float    BALANCE_STUCK_ESCAPE_RATE     = 3.0f;     // deg/s to shift setpoint away when stuck

// ---------------------------------------------------------------------------
// CRSF telemetry
// ---------------------------------------------------------------------------
static constexpr uint32_t CRSF_TELEMETRY_PERIOD_MS = 200;  // ~5 Hz
