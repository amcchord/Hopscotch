#pragma once
#include <cstdint>
#include "network_safety.h"

struct NetworkMotor {
    float pos = 0, vel = 0, torque = 0, temp = 0;
    uint32_t age_ms = 0;
    uint8_t id = 0;
    uint16_t error = 0;
    bool online = false, enabled = false;
};
struct NetworkSnapshot {
    uint32_t sequence = 0, uptime_ms = 0, imu_age_us = 0, rc_age_ms = 0;
    bool drive_armed = false, arm_armed = false, arming = false;
    bool link_up = false, balance_active = false, saving_log = false;
    bool rearm_required = true, maintenance = false, safe = false;
    bool calibration = false, test = false, simulation = false;
    int8_t rssi = 0;
    uint8_t lq = 0;
    uint16_t inner_fault = 0, channels[16] = {};
    float tilt = 0, rate = 0, setpoint = 0, voltage = 0, current = 0;
    bool voltage_fresh = false, current_fresh = false;
    char balance_state[24] = {}, end_reason[48] = {};
    char start_status[64] = {};
    NetworkMotor motors[6];
    LoopTiming balance_timing, control_timing;
};
