#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

struct BalanceSample {
    uint32_t t_ms;
    uint16_t sample_dt_ms;
    uint16_t inner_dt_max_us;
    uint16_t feedback_age_l_ms;
    uint16_t feedback_age_r_ms;
    uint16_t update_age_ms;
    uint16_t marker;
    uint16_t diag_flags;
    uint8_t  state;
    uint8_t  flags;
    uint8_t  arm_stage;
    uint8_t  inner_ticks;
    uint8_t  inner_sat_ticks;
    uint8_t  imu_age_ms;  // window maximum, saturated at 255; header feature bit 0
    float    roll;
    float    roll_rate;
    float    accel_angle;
    float    gyro_raw;
    float    accel_norm;
    float    setpoint;
    float    angle_err;
    float    base_sp;
    float    raw_base_sp;
    float    capture_shift;
    float    run_curve_shift;
    float    motor_vel_raw;
    float    motor_vel;
    float    cmd_left;
    float    cmd_right;
    float    sp_offset;
    float    sp_offset_target;
    float    target_vel;
    float    filtered_vel;
    float    vel_err;
    float    vel_integral;
    float    vel_p_term;
    float    pos_gate;
    float    shed;
    float    bl_pos, br_pos;
    float    bl_vel, br_vel;
    float    bl_torque, br_torque;
    float    arm_l, arm_r;
    float    arm_l_tgt, arm_r_tgt;
    float    arm_l_vel, arm_r_vel;
    float    arm_l_torque, arm_r_torque;
    float    meas_drift;
    float    meas_vel;
    float    arm_tip_frac;
    float    arm_assist_frac;
    float    arm_assist_vel;
    float    arm_demand;
    float    arm_calm_ms;
    float    yaw_diff;
    float    yaw_corr;
    float    bus_voltage;
    float    total_current;
    // Schema 3 appends pilot intent after the exact 220-byte schema-2 prefix.
    float    pilot_forward;
    float    pilot_steering;
    float    pilot_turn;
    uint32_t pilot_flags; // ready=1, moving/braking=2, turning=4, fresh RC=8, accel/handoff=16
    // Schema 4 preserves the complete schema-3 prefix.
    float    pilot_arm; // planned center fraction after recovery priority, before arm output filter
};

// One 50 Hz sample for the complete 120-second capture. The in-memory and
// on-flash representations are binary so this expanded forensic schema fits
// comfortably in PSRAM and the 1.5 MB LittleFS partition.
static constexpr int BALANCE_LOG_MAX_SAMPLES = 6000;

static constexpr int BALANCE_LOG_MAX_CURVE_POINTS = 6;

struct BalanceLogConfigSnapshot {
    float inner_kp;
    float inner_kd;
    float drift_vel_kp;
    float drift_max_vel;
    float ramp_drift_kp;
    float ramp_drift_max_vel;
    float vel_sp_kp;
    float vel_sp_kp_low;
    float vel_sp_knee;
    float vel_sp_ki;
    float sp_offset_max;
    float ramp_sp_offset_max;
    float sp_offset_rate;
    float vel_filter_alpha;
    float pos_gate_err;
    float shed_vel_start;
    float shed_vel_full;
    float stored_trim;
    float glide_vel_err;
    float glide_ki_boost;
    float speed_acc_rad;
    float speed_current_limit;
    float base_sp_fwd;
    float base_sp_tip;
    float base_sp_center;
    float base_sp_rate_max;
    float ramp_vel_slow;
    float comp_alpha;
    float max_drive_speed;
    float arm_return_speed;
    float arm_assist_thresh;
    float arm_assist_gain;
    float arm_range_pos;
    float arm_range_neg;
    float arm_tau_in;
    float arm_tau_out;
    float arm_emergency_cmd_frac;
    float yaw_sync_kp;
    float yaw_sync_max;
    uint32_t capture_settle_ms;
    uint32_t arm_hold_max_ms;
    uint32_t log_duration_ms;
    uint16_t balance_loop_hz;
    uint16_t control_loop_hz;
    uint8_t curve_len;
    uint8_t reserved[3];
    float curve_frac[BALANCE_LOG_MAX_CURVE_POINTS];
    float curve_sp[BALANCE_LOG_MAX_CURVE_POINTS];
};

namespace balance_log {
static constexpr uint16_t V2_SAMPLE_BYTES = 220;
static constexpr uint16_t V3_SAMPLE_BYTES = 236;
static constexpr uint16_t V4_SAMPLE_BYTES = 240;
inline bool supported(uint16_t schema, uint16_t bytes) {
    return (schema == 2 && bytes == V2_SAMPLE_BYTES)
        || (schema == 3 && bytes == V3_SAMPLE_BYTES)
        || ((schema == 4 || schema == 5) && bytes == V4_SAMPLE_BYTES); // v5 changes policy metadata only
}
}
static_assert(offsetof(BalanceSample, pilot_forward) == balance_log::V2_SAMPLE_BYTES,
              "Schema 2 prefix changed");
static_assert(offsetof(BalanceSample, pilot_arm) == balance_log::V3_SAMPLE_BYTES,
              "Schema 3 prefix changed");
static_assert(sizeof(BalanceSample) == balance_log::V4_SAMPLE_BYTES, "Schema 4 layout changed");
