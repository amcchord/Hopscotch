#include "robstride.h"
#include <Arduino.h>
#include <cstring>

// The Robstride 29-bit extended CAN ID is packed as:
//   bits [28:24] = communication type  (5 bits)
//   bits [23:16] = extra data field    (8 bits, usage depends on command)
//   bits [15:8]  = target motor CAN ID (8 bits)
//   bits [7:0]   = host/master ID      (8 bits)

uint32_t Robstride::makeCanId(RobstrideCommType type, uint16_t data16,
                               uint8_t target_id, uint8_t host_id) {
    uint32_t id = 0;
    id |= (static_cast<uint32_t>(type) & 0x1F) << 24;
    id |= (static_cast<uint32_t>(data16) & 0xFF) << 16;
    id |= (static_cast<uint32_t>(target_id) & 0xFF) << 8;
    id |= (static_cast<uint32_t>(host_id) & 0xFF);
    return id;
}

bool Robstride::begin(int tx_pin, int rx_pin) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(tx_pin),
        static_cast<gpio_num_t>(rx_pin),
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] Driver install failed: 0x%x\n", err);
        return false;
    }

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[CAN] Start failed: 0x%x\n", err);
        twai_driver_uninstall();
        return false;
    }

    _initialized = true;
    Serial.println("[CAN] TWAI initialized at 1Mbps");
    return true;
}

void Robstride::end() {
    if (_initialized) {
        twai_stop();
        twai_driver_uninstall();
        _initialized = false;
    }
}

bool Robstride::sendFrame(uint32_t ext_id, const uint8_t* data, uint8_t len) {
    if (!_initialized) return false;

    twai_message_t msg = {};
    msg.extd = 1;
    msg.identifier = ext_id;
    msg.data_length_code = len;
    if (data && len > 0) {
        memcpy(msg.data, data, len);
    }

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (err != ESP_OK) {
        Serial.printf("[CAN] TX fail id=0x%08lX err=0x%x\n", (unsigned long)ext_id, err);
        return false;
    }
    return true;
}

bool Robstride::enableMotor(uint8_t motor_id, uint8_t host_id) {
    uint32_t can_id = makeCanId(RobstrideCommType::Enable, 0, motor_id, host_id);
    uint8_t data[8] = {0};
    return sendFrame(can_id, data, 8);
}

bool Robstride::stopMotor(uint8_t motor_id, uint8_t host_id, bool clear_fault) {
    uint8_t data16 = clear_fault ? 1 : 0;
    uint32_t can_id = makeCanId(RobstrideCommType::Stop, data16, motor_id, host_id);
    uint8_t data[8] = {0};
    return sendFrame(can_id, data, 8);
}

bool Robstride::sendMITControl(uint8_t motor_id, uint8_t host_id,
                                float position, float velocity,
                                float kp, float kd, float torque) {
    // MIT mode packs target position, velocity, kp, kd, torque into 8 bytes
    // using scaled integer encoding within known ranges:
    //   position: [-12.5, 12.5] rad  -> uint16  (16 bits)
    //   velocity: [-44, 44] rad/s    -> uint12  (12 bits)
    //   kp:       [0, 500]           -> uint12  (12 bits)
    //   kd:       [0, 5]             -> uint12  (12 bits)
    //   torque:   [-17, 17] Nm       -> uint12  (12 bits)

    auto float_to_uint = [](float x, float x_min, float x_max, int bits) -> uint16_t {
        float span = x_max - x_min;
        if (x < x_min) x = x_min;
        if (x > x_max) x = x_max;
        return static_cast<uint16_t>((x - x_min) / span * ((1 << bits) - 1));
    };

    uint16_t p = float_to_uint(position, -12.5f, 12.5f, 16);
    uint16_t v = float_to_uint(velocity, -44.0f, 44.0f, 12);
    uint16_t kp_i = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_i = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint16_t t = float_to_uint(torque, -17.0f, 17.0f, 12);

    uint8_t data[8];
    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;
    data[2] = (v >> 4) & 0xFF;
    data[3] = ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F);
    data[4] = kp_i & 0xFF;
    data[5] = (kd_i >> 4) & 0xFF;
    data[6] = ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F);
    data[7] = t & 0xFF;

    uint32_t can_id = makeCanId(RobstrideCommType::Control, 0, motor_id, host_id);
    return sendFrame(can_id, data, 8);
}

bool Robstride::writeFloatParam(uint8_t motor_id, uint8_t host_id,
                                 uint16_t param_addr, float value) {
    uint8_t data[8] = {0};
    // Bytes 0-1: parameter index (little-endian)
    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;
    // Bytes 4-7: float value (little-endian)
    memcpy(&data[4], &value, sizeof(float));

    uint32_t can_id = makeCanId(RobstrideCommType::ParamWrite, 0, motor_id, host_id);
    return sendFrame(can_id, data, 8);
}

bool Robstride::writeU8Param(uint8_t motor_id, uint8_t host_id,
                              uint16_t param_addr, uint8_t value) {
    uint8_t data[8] = {0};
    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = value;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    uint32_t can_id = makeCanId(RobstrideCommType::ParamWrite, 0, motor_id, host_id);
    return sendFrame(can_id, data, 8);
}

bool Robstride::setRunMode(uint8_t motor_id, uint8_t host_id, RobstrideRunMode mode) {
    return writeU8Param(motor_id, host_id, RobstrideParam::RUN_MODE, static_cast<uint8_t>(mode));
}

bool Robstride::sendPositionCommand(uint8_t motor_id, uint8_t host_id,
                                     float target_pos_rad, float speed_limit_rad_s) {
    writeFloatParam(motor_id, host_id, RobstrideParam::SPEED_LIMIT, speed_limit_rad_s);
    delayMicroseconds(200);
    return writeFloatParam(motor_id, host_id, RobstrideParam::TARGET_POSITION, target_pos_rad);
}

bool Robstride::sendSpeedCommand(uint8_t motor_id, uint8_t host_id,
                                  float target_speed_rad_s, float current_limit_a) {
    writeFloatParam(motor_id, host_id, RobstrideParam::CURRENT_LIMIT, current_limit_a);
    delayMicroseconds(200);
    return writeFloatParam(motor_id, host_id, RobstrideParam::TARGET_SPEED, target_speed_rad_s);
}

bool Robstride::changeMotorCanId(uint8_t current_id, uint8_t host_id, uint8_t new_id) {
    uint32_t can_id = makeCanId(RobstrideCommType::SetID, new_id, current_id, host_id);
    uint8_t data[8] = {0};
    return sendFrame(can_id, data, 8);
}

bool Robstride::setZeroPosition(uint8_t motor_id, uint8_t host_id) {
    uint32_t can_id = makeCanId(RobstrideCommType::SetZero, 1, motor_id, host_id);
    uint8_t data[8] = {0};
    return sendFrame(can_id, data, 8);
}

bool Robstride::receiveFeedback(RobstrideFeedback& fb, uint32_t timeout_ms) {
    if (!_initialized) return false;

    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) return false;

    if (!msg.extd) return false;

    uint32_t id = msg.identifier;
    uint8_t comm_type = (id >> 24) & 0x1F;
    uint8_t motor_id = (id >> 8) & 0xFF;

    fb.motor_id = motor_id;

    if (comm_type == static_cast<uint8_t>(RobstrideCommType::Feedback)) {
        // Feedback frame: 8 bytes packed similarly to MIT command
        // Decode position (16-bit), velocity (12-bit), torque (12-bit), temp (8-bit), errors, mode
        auto uint_to_float = [](uint16_t val, float x_min, float x_max, int bits) -> float {
            float span = x_max - x_min;
            return x_min + static_cast<float>(val) / ((1 << bits) - 1) * span;
        };

        uint16_t p_raw = (static_cast<uint16_t>(msg.data[0]) << 8) | msg.data[1];
        uint16_t v_raw = (static_cast<uint16_t>(msg.data[2]) << 4) | (msg.data[3] >> 4);
        uint16_t t_raw = (static_cast<uint16_t>(msg.data[3] & 0x0F) << 8) | msg.data[4];
        uint16_t temp_raw = (static_cast<uint16_t>(msg.data[5]) << 4) | (msg.data[6] >> 4);

        fb.position = uint_to_float(p_raw, -12.5f, 12.5f, 16);
        fb.velocity = uint_to_float(v_raw, -44.0f, 44.0f, 12);
        fb.torque = uint_to_float(t_raw, -17.0f, 17.0f, 12);
        fb.temperature = uint_to_float(temp_raw, 0.0f, 120.0f, 12);

        // Mode and errors from ID data field
        uint8_t data_field = (id >> 16) & 0xFF;
        fb.mode = (data_field >> 5) & 0x03;
        fb.errors = data_field & 0x1F;
        fb.has_fault = (fb.errors != 0);

        return true;
    }

    if (comm_type == static_cast<uint8_t>(RobstrideCommType::Fault)) {
        fb.has_fault = true;
        fb.errors = msg.data[0] | (msg.data[1] << 8);
        return true;
    }

    return false;
}
