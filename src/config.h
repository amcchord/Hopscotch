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

// Control-core / comms-core split. Core 0 hosts the networking stack (WiFi
// task is pinned there by the framework, async_tcp/lwip pinned there via
// build flags); Core 1 is the dedicated control core. Priorities: sentinel
// (stall forensics) > 200Hz balance PD > 50Hz control > loopTask (display/
// WebSocket/debug at priority 1) -- nothing on the comms core can preempt
// control, and on the control core only control preempts control.
static constexpr int      CONTROL_CORE           = 1;
static constexpr int      CONTROL_TASK_PRIORITY  = 12;
static constexpr int      BALANCE_TASK_PRIORITY  = 18;
static constexpr int      SENTINEL_TASK_PRIORITY = 24;
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
// Self-balance mode (Speed-mode architecture, Phase 14)
//
// Back wheels run in Robstride Speed mode while balancing so the PD output
// directly commands wheel velocity. This restores the tilt-to-translation
// coupling that CSP position mode structurally broke (TUNING_HISTORY lessons
// 19-23): a tilt setpoint offset now genuinely translates the robot.
//
// Cascade:
//   POSITION LOOP (50Hz, Core 1) -- P only
//     target_vel = clamp(-DRIFT_VEL_KP * drift)
//   VELOCITY LOOP (50Hz, Core 1) -- PI, the single integrator (lesson 18)
//     sp_offset = VEL_SP_KP * vel_err + integral(VEL_SP_KI * vel_err)
//     where vel_err = filtered_wheel_vel - target_vel
//   INNER LOOP (200Hz, Core 0) -- Balance PD (unchanged, proven)
//     wheel_speed_cmd = Kp * (effective_sp - tilt) - Kd * gyro_rate
//
//   effective_setpoint = arm_curve(arm_frac) + capture_shift + sp_offset
//
// ---------------------------------------------------------------------------
// The old decoder reported RS05 wheel velocity at 33/50 of its physical value.
// Convert feedback-based tuning to correct units without increasing loop gain.
static constexpr float BALANCE_WHEEL_VELOCITY_SCALE = 50.0f / 33.0f;

static constexpr float    BALANCE_SETPOINT_ARMS_FWD     = 84.0f;   // balance point arms-forward. Was 89.3; the battery ejection/
                                                                   // reinstall (run 164837 crash) moved the CG ~5 deg -- run
                                                                   // 224649 equilibrated at 82.9 with the integral railed at -8
                                                                   // compensating. Anchors re-zeroed to the current physical
                                                                   // reality; the learned trim tracks future (smaller) shifts.
static constexpr float    BALANCE_SETPOINT_ARMS_TIP     = 82.1f;   // balance point with arms at tip position. Refit from the
                                                                   // Speed-mode capture calibrations (Jul 2026): the true
                                                                   // tip->fwd equilibrium drop is ~1.9 deg (measured +1.0..+3.3
                                                                   // across runs), not the 2.9 the old anchors assumed.
static constexpr float    BALANCE_SETPOINT_MIN          = 70.0f;   // hard safety clamp (fallen forward)
static constexpr float    BALANCE_SETPOINT_MAX          = 110.0f;  // hard safety clamp (fallen backward)
static constexpr float    BALANCE_CAPTURE_SHIFT_MAX_DEG = 15.0f;   // cover large engage angle differences
static constexpr bool     BALANCE_USE_SCHEDULED_SP      = true;    // if false, base setpoint stays at ARMS_FWD always
static constexpr bool     BALANCE_USE_CAPTURE_SHIFT     = true;    // if false, no capture shift -- engage directly at scheduled sp
static constexpr float    BALANCE_BASE_SP_RATE_MAX      = 4.0f;    // must not bottleneck the 2.5 rad/s arm return (the curve
                                                                   // needs ~2.9 deg over ~1s of return)
static constexpr float    BALANCE_RAMP_VEL_SLOW         = 2.0f * BALANCE_WHEEL_VELOCITY_SCALE;    // rad/s wheel speed at which the base ramp fully pauses --
                                                                   // the ramp waits for the robot instead of towing it
                                                                   // (standup surge, runs 164837/171838)
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
static constexpr float    BALANCE_ARM_RETURN_SPEED      = 1.5f;    // rad/s. 2.5 was dynamically infeasible: the equilibrium
                                                                   // moved 3 deg in 1s while the robot's tilt never budged --
                                                                   // velocity-mode PD chases a moving equilibrium with
                                                                   // velocity, not the acceleration needed to tilt (runaway,
                                                                   // run 223630). Halving return time quadruples the required
                                                                   // acceleration.

static constexpr float    BALANCE_MAX_DRIVE_SPEED       = 30.0f;   // rad/s speed limit (railed at 25 during the 17 rad/s
                                                                   // tap recovery in bal_20260702_161437; RS05 feedback range is 50; keep tested command cap)

// Arm-position -> balance-point curve (piecewise linear on arm tip fraction,
// 0 = arms at forward ref, 1 = arms at tip pose). Fitted from stable-balance
// windows across all 80 telemetry logs (scripts/fit_balance_model.py). The
// relationship is strongly nonlinear: most of the CG shift happens in the
// first ~15% of arm travel, then the balance point is flat to the tip pose.
struct BalanceSpAnchor {
    float arm_frac;
    float setpoint_deg;
};
static constexpr BalanceSpAnchor BALANCE_SP_CURVE[] = {
    // Monotonic tip->forward rise. The CSP-era shape (flat middle, dip at
    // 0.15, jump at the end) made the setpoint lag the true equilibrium
    // through the arm return -- the robot towed forward +15 rad chasing it
    // (run 094448). Absolute level is re-zeroed at every settled capture;
    // these anchors only need the right SHAPE.
    // Shape refit from Speed-mode data (Jul 2026, fit_balance_model.py
    // --speed-only + per-run tip-vs-forward equilibrium deltas): total drop
    // ~1.9 deg, close to linear across the return. The old 2.9 deg drop
    // overshot the real shift, holding the setpoint ~1 deg above the true
    // equilibrium through the return = 2 rad/s of tow velocity.
    { 0.00f, 84.0f },    // arms forward
    { 0.50f, 83.05f },   // mid return
    { 1.00f, 82.1f },    // arms at tip
};
static constexpr int BALANCE_SP_CURVE_LEN =
    sizeof(BALANCE_SP_CURVE) / sizeof(BALANCE_SP_CURVE[0]);

// Inner loop PD gains (200Hz, Core 0)
//   Input:  angle_err = effective_setpoint - tilt_angle (deg)
//   Input:  gyro_rate (deg/s)
//   Output: motor_vel = Kp * angle_err - Kd * gyro_rate (rad/s)
//   motor_vel is sent directly as the wheel speed command (Speed mode)
static constexpr float    BALANCE_KP                    = 2.0f;    // rad/s per degree of angle error
static constexpr float    BALANCE_KD                    = 0.08f;   // rad/s per deg/s of roll rate

static constexpr float    COMPLEMENTARY_ALPHA           = 0.996f;  // gyro weight in complementary filter

// Speed-mode motor configuration (applied at the START of tip-up, while the
// robot is static on all fours -- the blocking switch/verified writes there
// cost nothing, and a failed switch aborts a standup that never started.
// Doing it at engage stalled Core 1 for 0.4-7.3s in EVERY logged run, right
// at the moment the capture begins.)
static constexpr float    BALANCE_SPEED_ACC_RAD         = 100.0f;  // rad/s^2 velocity-mode accel limit (default 20 is too slow for balance)
static constexpr float    BALANCE_SPEED_CURRENT_LIMIT_A = 14.0f;   // near motor max (16): recoveries were torque-starved at 10
static constexpr uint32_t BALANCE_TIPUP_SPEED_REFRESH_MS = 250;    // re-send 0-speed during tip-up so the motor-side CAN
                                                                   // watchdog (0x200C, ~1s) never fires mid-tip

static constexpr uint32_t BALANCE_LOG_DURATION_MS       = 120000;  // full 120s at 50Hz (6000 samples); binary persistence
                                                                   // keeps the expanded schema inside the LittleFS partition
static const char*        BALANCE_LOG_PATH              = "/bal_log.bin";
static const char*        BALANCE_LEGACY_LOG_PATH       = "/bal_log.csv";

// Safety abort thresholds
static constexpr float    BALANCE_SAFE_TILT_MIN         = 30.0f;    // hard abort below this (fallen forward)
static constexpr float    BALANCE_SAFE_TILT_MAX         = 150.0f;   // hard abort above this (fallen backward)
static constexpr float    BALANCE_SAFE_ERR_MAX_DEG      = 35.0f;    // sustained effective-error threshold
static constexpr uint32_t BALANCE_SAFE_ERR_DURATION_MS  = 2000;     // must persist this long before abort
static constexpr float    BALANCE_SAFE_RATE_MAX_DPS     = 200.0f;   // extreme roll-rate threshold
static constexpr uint32_t BALANCE_SAFE_RATE_DURATION_MS = 500;      // must persist this long
static constexpr uint32_t BALANCE_SAFE_SAT_DURATION_MS  = 3000;     // speed command saturated this long -> disengage
static constexpr uint32_t BALANCE_FEEDBACK_STALE_MS     = 400;      // abort if wheel feedback older than this (CAN failure)
static constexpr uint32_t BALANCE_IMU_STALE_US           = 50000;    // ten missing 200Hz samples
static constexpr uint32_t BALANCE_IMU_READY_MS           = 200;      // healthy stream before a new attempt
static constexpr float    BALANCE_ARM_REACHED_RAD       = 0.15f;    // measured arrival as well as target completion
static constexpr uint32_t BALANCE_TIP_TIMEOUT_MS         = 15000;
static constexpr uint32_t BALANCE_RETURN_TIMEOUT_MS      = 8000;

// Two-stage dead-man for Core 1 stalls (run 173619: a 740ms stall with an
// instant wheel-stop dead-man dropped a perfectly balanced robot). The
// 200Hz inner PD on Core 0 can hold balance on a stale setpoint for a
// while; it just must not be allowed to run away.
static constexpr uint32_t BALANCE_DEADMAN_SOFT_MS       = 300;      // clamp wheel authority (keep balancing)
static constexpr float    BALANCE_DEADMAN_SOFT_CMD_MAX  = 20.0f;    // rad/s clamp in degraded mode. Stale-setpoint creep is
                                                                    // <8 rad/s so a tight clamp never stopped creep -- it only
                                                                    // throttled real catches (fall during a 430ms stall with
                                                                    // cmd pinned at 8.0, run 093643)
static constexpr uint32_t BALANCE_DEADMAN_HARD_MS       = 1500;     // stop wheels entirely

// Outer cascade (50Hz, Core 1) -- active after the base setpoint ramp
// completes, gated by angle error. Gains from scripts/fit_balance_model.py
// (linearized model, robust across A x0.5-2, B x0.7-1.4, motor lag 30-80ms).
// Margins are structurally thin: start conservative, retune from Speed-mode
// telemetry.
static constexpr float    BALANCE_DRIFT_VEL_KP          = 0.05f * BALANCE_WHEEL_VELOCITY_SCALE;    // retain July's tested return gain. The prepared 0.08
                                                                    // regressed recovery in the refreshed simulation; validate
                                                                    // reduced standup tow before speeding up return-to-origin.
static constexpr float    BALANCE_DRIFT_MAX_VEL         = 1.0f * BALANCE_WHEEL_VELOCITY_SCALE;     // max return velocity (rad/s)
// Early position P: the position loop also runs DURING the standup ramp
// (origin at engage) at reduced gain, so standup drift is opposed as it
// develops instead of repaid after ramp completion. Sim (balance_sim.py
// standup-matrix): fewer standup falls and ~20% less peak tow, with no
// interference with the ramp (the gain is low and the angle-error gate
// still applies). The integrator stays ramp-gated (single-integrator rule).
static constexpr float    BALANCE_RAMP_DRIFT_KP         = 0.03f * BALANCE_WHEEL_VELOCITY_SCALE;    // rad/s per rad of drift during the ramp
static constexpr float    BALANCE_RAMP_DRIFT_MAX_VEL    = 0.6f * BALANCE_WHEEL_VELOCITY_SCALE;     // clamp during the ramp (rad/s)
static constexpr float    BALANCE_VEL_SP_KP             = 2.2f / BALANCE_WHEEL_VELOCITY_SCALE;     // deg per rad/s of velocity error ABOVE the soft knee
                                                                    // (tap/disturbance regime -- keeps the athletic recovery)
static constexpr float    BALANCE_VEL_SP_KP_LOW         = 0.7f / BALANCE_WHEEL_VELOCITY_SCALE;     // deg per rad/s BELOW the knee (station-keeping regime).
                                                                    // A single 2.2 gain limit-cycled at ~0.3 Hz: the setpoint
                                                                    // chased idle velocity ripple and the robot swayed
                                                                    // (bal_20260702_164034: sp_offset std > roll std).
static constexpr float    BALANCE_VEL_SP_KNEE           = 0.8f * BALANCE_WHEEL_VELOCITY_SCALE;     // rad/s boundary between the two slopes
static constexpr float    BALANCE_VEL_SP_KI             = 0.35f / BALANCE_WHEEL_VELOCITY_SCALE;    // deg/s per rad/s of velocity error (single integrator)
static constexpr float    BALANCE_SP_OFFSET_MAX_DEG     = 8.0f;     // setpoint offset clamp (6 railed for 6.6s during the
                                                                    // escalating-tap run -- it was the binding constraint)
static constexpr float    BALANCE_RAMP_SP_OFFSET_MAX_DEG = 1.5f;    // pre-ramp authority limit; Sept 14 assisted trial.
                                                                    // July's largest surge came AFTER ramp_complete released
                                                                    // this limit. This clamp alone does not address that surge;
                                                                    // see docs/BALANCE_REVIEW_2026-09.md.
static constexpr float    BALANCE_SP_OFFSET_RATE        = 12.0f;    // deg/s rate limit -- still step-free; 16 allowed a
                                                                    // +6.6 -> -2.0 whipsaw in 0.5s (run 232626 overcorrection)
static constexpr float    BALANCE_VEL_FILTER_ALPHA      = 0.35f;    // wheel velocity LPF (~55ms) -- earlier lean-in on taps
static constexpr float    BALANCE_POS_GATE_ERR_DEG      = 8.0f;     // angle error at which outer-loop authority -> 0

// High-velocity shed: braking-by-lean stops working near the wheel speed
// ceiling (leaning back needs MORE forward acceleration -- the runaway that
// crashed runs 155640 and 164837). Above SHED_START the velocity-P authority
// fades out; at SHED_FULL it is zero and the robot accepts displacement
// instead of pumping itself into saturation.
static constexpr float    BALANCE_SHED_VEL_START        = 8.0f * BALANCE_WHEEL_VELOCITY_SCALE;     // rad/s
static constexpr float    BALANCE_SHED_VEL_FULL         = 14.0f * BALANCE_WHEEL_VELOCITY_SCALE;    // rad/s

// Arm assist v2: arms swing toward the CENTER pose (the physically-symmetric
// "both arms up" axis -- the tip pose is asymmetric per-arm and scaling it
// moves the arms in opposite directions) to brake forward runaways. The
// balance point at center is ~6 deg below arms-forward (CSP-era measurement
// 83.0), so a partial excursion shifts equilibrium down with NO wheel
// acceleration, plus reaction torque in the braking direction. The setpoint
// follows via the center-axis term in computeScheduledSetpoint(). Arms
// spring back to forward when calm.
static constexpr float    BALANCE_SETPOINT_ARMS_CENTER  = 77.7f;    // balance point at full center. The SLOPE vs ARMS_FWD is what
                                                                    // matters (-6.3 deg per center-frac, CSP-era measurement);
                                                                    // shifted with the other anchors to preserve it.
// Push recovery lives or dies on arm speed (run 224649: a -7 rad/s shove
// peaked the arms at 0.17 of 0.30 range while the setpoint railed -- the
// old filter+threshold+gain chain wound up after the event was over).
// BUT the threshold must stay OUTSIDE the settle band: at 1.0 rad/s the
// arms engaged on ordinary settling motion and became a 0.5 Hz oscillator
// (run 225359: arm-vel correlation 0.88 at 240ms lag, roll +/-2.7 deg,
// never settled). Below the threshold arm gain is zero and the wheel-only
// cascade is proven stable -- the limit cycle cannot sustain itself.
static constexpr float    BALANCE_ARM_ASSIST_THRESH     = 1.4f * BALANCE_WHEEL_VELOCITY_SCALE;     // rad/s vel error to engage arms. 2.0 + double filtering
                                                                    // meant a -3.1 rad/s push never triggered at all (225944);
                                                                    // 1.0 sat in the settle band (oscillator, 225359).
static constexpr float    BALANCE_ARM_ASSIST_GAIN       = 0.40f / BALANCE_WHEEL_VELOCITY_SCALE;    // center-frac per rad/s beyond threshold: steep -- a real
                                                                    // push gets a committed throw, not a proportional dribble
static constexpr float    BALANCE_ARM_ASSIST_BIAS_FRAC  = 0.00f;    // neutral stance = the FORWARD reference pose. With the
                                                                    // robot standing (body rotated ~90 deg from driving), the
                                                                    // forward pose points straight up -- it IS top-dead-center.
static constexpr float    BALANCE_ARM_ASSIST_RANGE_POS  = 0.45f;    // toward calibrated center (arms back, brakes forward
                                                                    // motion) -- proven territory
static constexpr float    BALANCE_ARM_ASSIST_RANGE_NEG  = 0.30f;    // forward of vertical (brakes backward motion) --
                                                                    // mechanically unverified beyond this, extend after check
static constexpr float    BALANCE_ARM_ASSIST_VEL_TAU    = 0.02f;    // s, essentially raw: motor velocity feedback is clean and
                                                                    // every ms of filter lag delays the arm throw (0.35s
                                                                    // push-to-deploy measured in run 094448). The one-shot
                                                                    // latch makes a rare noise blip cost a bump, not a cycle.
static constexpr float    BALANCE_ARM_ASSIST_TAU_IN     = 0.08f;    // s, deploy time constant: fast IS the feature
static constexpr float    BALANCE_ARM_ASSIST_TAU_OUT    = 0.65f;    // s, release: monotonic return to neutral (operator asked
                                                                    // for ~50% quicker than the 1.0s it shipped with)

// Calm definition for re-arming the assist. Must sit OUTSIDE the robot's
// normal breathing band (+/-1.5 rad/s -- calm<1.0 locked the arms in
// COOLDOWN for an entire run and a tap got zero arm help, run 222915) but
// INSIDE the oscillation band (the 0.56 Hz limit cycle ran +/-4.5 rad/s).
static constexpr float    BALANCE_ARM_CALM_VEL          = 1.8f * BALANCE_WHEEL_VELOCITY_SCALE;     // rad/s
static constexpr float    BALANCE_ARM_CALM_RATE         = 20.0f;    // dps
static constexpr float    BALANCE_ARM_CALM_MS           = 300.0f;   // sustained before re-arm

// Emergency arm throw: wheels far into their authority while still carrying
// velocity error means a roll-away in progress -- any arm authority is pure
// gain. Bypasses the engagement lifecycle and throws the arms to their full
// stop in the braking direction. 45% of max (=13.5 rad/s) still sits well
// above the observed oscillation band (+/-8) -- the run-231458 forward
// runaway held cmd at 13-15 rad/s for 1.5s and PEAKED at 17.3, just under
// the old 18 rad/s trigger; the robot needed a hand stop.
static constexpr float    BALANCE_ARM_EMERGENCY_CMD_FRAC = 0.45f;
static constexpr float    BALANCE_ARM_ASSIST_SPEED      = 12.0f;    // rad/s arm motor speed limit

// React to incipient roll-away during arm return, before normal learning starts.
// Physical RS05 units. A brief gain boost learns a correction from wheel motion;
// the same integrator then hands over continuously to ordinary balance control.
static constexpr float    BALANCE_START_RECOVERY_SPEED = 1.0f;  // rad/s; acceleration required below force threshold
static constexpr float    BALANCE_START_RECOVERY_FORCE_SPEED = 4.0f;
static constexpr float    BALANCE_START_RECOVERY_ACCEL = 2.0f;  // outward rad/s^2
static constexpr float    BALANCE_START_RECOVERY_ACCEL_TAU = 0.06f;
static constexpr uint32_t BALANCE_START_RECOVERY_CONFIRM_MS = 60;
static constexpr float    BALANCE_START_RECOVERY_TIP_MAX = 0.90f;
static constexpr uint32_t BALANCE_START_RECOVERY_FEEDBACK_MS = 30;
static constexpr float    BALANCE_START_RECOVERY_KI = 1.0f;
static constexpr uint32_t BALANCE_START_RECOVERY_BOOST_MS = 800;
static constexpr float    BALANCE_START_RECOVERY_LIMIT_DEG = 6.0f;
static constexpr float    BALANCE_START_RECOVERY_RATE_DPS = 6.0f;
static constexpr float    BALANCE_START_RECOVERY_CALM_VEL = 0.7f;
static constexpr float    BALANCE_START_RECOVERY_CALM_RATE = 4.0f;
static constexpr float    BALANCE_START_RECOVERY_CALM_ERR = 1.0f;
static constexpr uint32_t BALANCE_START_RECOVERY_CALM_MS = 400;

// Dynamic equilibrium learning. The velocity-PI integrator IS the equilibrium
// estimator (it converges to the true balance offset from the arm-curve
// nominal). Three additions make it dynamic instead of per-run:
//   1. It is seeded from the persisted settings.balance_trim at engage.
//   2. It learns faster while the robot is "gliding" (tracking the setpoint
//      well but persistently moving = the equilibrium estimate is wrong).
//   3. Its converged value is blended back into settings after a good run,
//      absorbing battery placement, payload, surface, and IMU mounting bias.
static constexpr float    BALANCE_GLIDE_VEL_ERR         = 0.8f * BALANCE_WHEEL_VELOCITY_SCALE;     // rad/s of filtered vel error = gliding (matches the knee;
                                                                    // 0.4 let station-keeping wobble pump the integral)
static constexpr float    BALANCE_GLIDE_KI_BOOST        = 4.0f;     // Ki multiplier while gliding
// A genuine glide is CALM (steady lean, low rate, small commands). A push /
// tap recovery also has large vel error but is violent -- boosting there
// corrupts the equilibrium estimate mid-recovery. Only boost when calm:
static constexpr float    BALANCE_GLIDE_RATE_MAX_DPS    = 10.0f;    // no boost above this roll rate
static constexpr float    BALANCE_GLIDE_CMD_MAX         = 3.0f;     // no boost above this |wheel cmd|

// Wheel yaw sync: in Speed mode the two wheel velocity loops run
// independently and small errors integrate into heading drift (CSP kept them
// position-locked). A differential speed correction holds the left/right
// position difference at its engage value.
static constexpr float    BALANCE_YAW_SYNC_KP           = 2.0f;     // rad/s per rad of L/R position divergence
static constexpr float    BALANCE_YAW_SYNC_MAX          = 1.5f;     // rad/s clamp on the correction
static constexpr uint32_t BALANCE_TRIM_SAVE_MIN_MS      = 8000;     // post-ramp balance time before trim is trusted
static constexpr float    BALANCE_TRIM_BLEND            = 0.5f;     // new_trim = old + blend*(learned - old)
static constexpr float    BALANCE_TRIM_SAVE_DELTA_DEG   = 0.05f;    // skip flash write for smaller changes

// ---------------------------------------------------------------------------
// CRSF telemetry
// ---------------------------------------------------------------------------
static constexpr uint32_t CRSF_TELEMETRY_PERIOD_MS = 200;  // ~5 Hz
