#include "robstride.h"
#include <Arduino.h>
#include <cstring>

// Robstride 29-bit extended CAN ID layout (confirmed from working JumpRopeM5/JumpropeESP32):
//   bits [28:24] = communication type  (5 bits)
//   bits [8:23]  = data field          (16 bits, typically master_id << 8 for non-motion)
//   bits [0:7]   = motor CAN ID        (8 bits)
//
// For non-motion commands: (commType << 24) | (masterId << 8) | motorId
// For motion control (0x01): (0x01 << 24) | (torque_u16 << 8) | motorId

uint32_t Robstride::makeCanId(RobstrideCommType type, uint16_t data16,
                               uint8_t target_id, uint8_t host_id) {
    if (type == RobstrideCommType::Control) {
        // Motion control: data16 = torque_u16 goes in bits 8-23, motor in bits 0-7
        return (static_cast<uint32_t>(type) << 24) |
               (static_cast<uint32_t>(data16) << 8) |
               static_cast<uint32_t>(target_id);
    }

    // All other commands: master_id in bits 8-15, motor_id in bits 0-7
    return (static_cast<uint32_t>(type) << 24) |
           (static_cast<uint32_t>(host_id) << 8) |
           static_cast<uint32_t>(target_id);
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

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        tx_fail_count++;
        return false;
    }
    tx_ok_count++;
    return true;
}

bool Robstride::broadcastScan(uint8_t host_id) {
    // ObtainID command: broadcast (motor_id=0) to discover all motors
    uint32_t can_id = makeCanId(RobstrideCommType::ObtainID, 0, 0, host_id);
    uint8_t data[8] = {0};
    return sendFrame(can_id, data, 8);
}

bool Robstride::sendMotionPing(uint8_t motor_id) {
    // Send motion control frame with all zeros -- provokes a feedback response
    // without actually commanding any movement
    uint32_t can_id = (static_cast<uint32_t>(RobstrideCommType::Control) << 24) |
                      (static_cast<uint32_t>(0x0000) << 8) |
                      static_cast<uint32_t>(motor_id);
    uint8_t data[8] = {0};
    return sendFrame(can_id, data, 8);
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
    // Private protocol Type 1 (operation control mode) per RS00 User Manual:
    //   CAN ID bits 23:8 = torque uint16 [0~65535] → [-14, +14] Nm
    //   Data Byte0~1: position uint16 [0~65535] → [-4pi, +4pi] rad
    //   Data Byte2~3: velocity uint16 [0~65535] → [-33, +33] rad/s
    //   Data Byte4~5: Kp       uint16 [0~65535] → [0, 500]
    //   Data Byte6~7: Kd       uint16 [0~65535] → [0, 5]
    // All fields are 16-bit, high byte first.

    auto float_to_uint16 = [](float x, float x_min, float x_max) -> uint16_t {
        float span = x_max - x_min;
        if (x < x_min) x = x_min;
        if (x > x_max) x = x_max;
        return static_cast<uint16_t>((x - x_min) / span * 65535.0f);
    };

    uint16_t t = float_to_uint16(torque, -14.0f, 14.0f);
    uint16_t p = float_to_uint16(position, -12.566f, 12.566f);
    uint16_t v = float_to_uint16(velocity, -33.0f, 33.0f);
    uint16_t kp_i = float_to_uint16(kp, 0.0f, 500.0f);
    uint16_t kd_i = float_to_uint16(kd, 0.0f, 5.0f);

    uint8_t data[8];
    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;
    data[2] = (v >> 8) & 0xFF;
    data[3] = v & 0xFF;
    data[4] = (kp_i >> 8) & 0xFF;
    data[5] = kp_i & 0xFF;
    data[6] = (kd_i >> 8) & 0xFF;
    data[7] = kd_i & 0xFF;

    uint32_t can_id = makeCanId(RobstrideCommType::Control, t, motor_id, host_id);
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

bool Robstride::readParam(uint8_t motor_id, uint8_t host_id, uint16_t param_addr) {
    uint8_t data[8] = {0};
    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;

    uint32_t can_id = makeCanId(RobstrideCommType::ParamRead, 0, motor_id, host_id);
    return sendFrame(can_id, data, 8);
}

bool Robstride::readParamSync(uint8_t motor_id, uint8_t host_id,
                               uint16_t param_addr, float& out_value,
                               uint32_t timeout_ms) {
    if (!readParam(motor_id, host_id, param_addr)) return false;

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(5));
        if (err != ESP_OK) continue;

        rx_count++;
        if (!msg.extd) continue;

        uint8_t comm_type = (msg.identifier >> 24) & 0x1F;
        uint8_t resp_motor = (msg.identifier >> 8) & 0xFF;

        if (comm_type == static_cast<uint8_t>(RobstrideCommType::ParamRead) &&
            resp_motor == motor_id) {
            uint16_t resp_param = msg.data[0] | (msg.data[1] << 8);
            if (resp_param == param_addr) {
                memcpy(&out_value, &msg.data[4], sizeof(float));
                return true;
            }
        }

        // If it's a feedback frame or other response, let it pass
        // (we lose it, but this is a debug tool)
    }
    return false;
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

    rx_count++;

    // Log raw frame for later inspection
    if (rx_log_idx < RX_LOG_SIZE) {
        RxLogEntry& e = rx_log[rx_log_idx];
        e.id = msg.identifier;
        e.dlc = msg.data_length_code;
        e.extd = msg.extd;
        e.used = true;
        memcpy(e.data, msg.data, 8);
        rx_log_idx++;
    }

    if (!msg.extd) {
        return false;
    }

    uint32_t id = msg.identifier;
    uint8_t comm_type = (id >> 24) & 0x1F;

    // In the Robstride protocol, response frames have motor_id at bits 8-15
    // (confirmed from working JumpRopeM5: feedback motor ID parsed from bits 8-15)
    uint8_t motor_id = (id >> 8) & 0xFF;
    fb.motor_id = motor_id;

    if (comm_type == static_cast<uint8_t>(RobstrideCommType::Feedback)) {
        // Feedback frame: 4x uint16 big-endian (position, velocity, torque, temperature)
        // Each scaled over full 16-bit range with motor-specific limits
        // Error/mode from CAN ID bits 16-23
        auto uint_to_float = [](uint16_t val, float x_min, float x_max, int bits) -> float {
            float span = x_max - x_min;
            return x_min + static_cast<float>(val) / ((1 << bits) - 1) * span;
        };

        uint16_t pos_u16  = (static_cast<uint16_t>(msg.data[0]) << 8) | msg.data[1];
        uint16_t vel_u16  = (static_cast<uint16_t>(msg.data[2]) << 8) | msg.data[3];
        uint16_t torq_u16 = (static_cast<uint16_t>(msg.data[4]) << 8) | msg.data[5];
        uint16_t temp_u16 = (static_cast<uint16_t>(msg.data[6]) << 8) | msg.data[7];

        // Per RS00 User Manual section 4.1.2 (Type 2 feedback):
        //   Position: [0~65535] → [-4pi, +4pi] rad
        //   Velocity: [0~65535] → [-33, +33] rad/s
        //   Torque:   [0~65535] → [-14, +14] Nm
        fb.position = uint_to_float(pos_u16, -12.566f, 12.566f, 16);
        fb.velocity = uint_to_float(vel_u16, -33.0f, 33.0f, 16);
        fb.torque   = uint_to_float(torq_u16, -14.0f, 14.0f, 16);
        fb.temperature = static_cast<float>(temp_u16) * 0.1f;

        // Mode (bits 22-23) and error code (bits 16-21) from CAN ID
        fb.mode = (id >> 22) & 0x03;
        fb.errors = (id >> 16) & 0x3F;
        fb.has_fault = (fb.errors != 0);

        return true;
    }

    if (comm_type == static_cast<uint8_t>(RobstrideCommType::Fault)) {
        fb.has_fault = true;
        fb.errors = msg.data[0] | (msg.data[1] << 8);
        return true;
    }

    // Log any received frame for debugging
    Serial.printf("[CAN RX] type=%d motor=%d dlc=%d data=", comm_type, motor_id, msg.data_length_code);
    for (int i = 0; i < msg.data_length_code; i++) Serial.printf("%02X ", msg.data[i]);
    Serial.println();

    // Any other response from a motor means it's alive (param read response, enable ack, etc.)
    if (comm_type == static_cast<uint8_t>(RobstrideCommType::ParamRead) ||
        comm_type == static_cast<uint8_t>(RobstrideCommType::ParamWrite) ||
        comm_type == static_cast<uint8_t>(RobstrideCommType::Enable) ||
        comm_type == static_cast<uint8_t>(RobstrideCommType::Stop) ||
        comm_type == static_cast<uint8_t>(RobstrideCommType::ObtainID)) {

        // For param read responses, try to extract float value from data[4..7]
        if (comm_type == static_cast<uint8_t>(RobstrideCommType::ParamRead)) {
            uint16_t param_addr = msg.data[0] | (msg.data[1] << 8);
            if (param_addr == RobstrideParam::MOTOR_POSITION) {
                float val;
                memcpy(&val, &msg.data[4], sizeof(float));
                fb.position = val;
            }
        }

        fb.has_fault = false;
        fb.errors = 0;
        return true;
    }

    return false;
}

void Robstride::printRxLog() {
    Serial.printf("[CAN] RX log (%d frames captured):\n", rx_log_idx);
    for (int i = 0; i < rx_log_idx; i++) {
        RxLogEntry& e = rx_log[i];
        if (!e.used) break;
        Serial.printf("  #%d: ext=%d id=0x%08lX dlc=%d data=",
                      i, e.extd, (unsigned long)e.id, e.dlc);
        for (int j = 0; j < e.dlc; j++) {
            Serial.printf("%02X ", e.data[j]);
        }
        // Decode the ID fields
        if (e.extd) {
            uint8_t type_field = (e.id >> 24) & 0x1F;
            uint8_t data_field = (e.id >> 16) & 0xFF;
            uint8_t id_high    = (e.id >> 8) & 0xFF;
            uint8_t id_low     = e.id & 0xFF;
            Serial.printf(" [type=%d d16=0x%02X hi=0x%02X(%d) lo=0x%02X(%d)]",
                          type_field, data_field, id_high, id_high, id_low, id_low);
        }
        Serial.println();
    }
}

void Robstride::printBusStatus() {
    if (!_initialized) {
        Serial.println("[CAN] Not initialized");
        return;
    }

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        Serial.printf("[CAN] state=%d tx_err=%lu rx_err=%lu hw_tx_fail=%lu rx_miss=%lu arb_lost=%lu bus_err=%lu\n",
                      status.state,
                      status.tx_error_counter,
                      status.rx_error_counter,
                      status.tx_failed_count,
                      status.rx_missed_count,
                      status.arb_lost_count,
                      status.bus_error_count);
        Serial.printf("[CAN] our_tx_ok=%lu our_tx_fail=%lu our_rx=%lu\n",
                      tx_ok_count, tx_fail_count, rx_count);
    }
}
