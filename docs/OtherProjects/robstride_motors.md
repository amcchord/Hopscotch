# RobStride Motor Control Reference

## Motor: RobStride RS-02

| Specification    | Value          |
|-----------------|----------------|
| Max Torque      | 17 Nm          |
| Max Speed       | 44 rad/s       |
| KP Range        | 0 - 500.0      |
| KD Range        | 0 - 5.0        |
| Control Modes   | MIT, Position, Speed |

## CAN Bus Protocol

### Physical Layer

- **Standard**: CAN 2.0B
- **Frame format**: Extended (29-bit identifier)
- **Baud rate**: 1 Mbps
- **Transceiver**: TJA1051T/3 (via M5Stack Mini CAN Unit)

### Hardware Connection

```
M5StickC Plus 2          Mini CAN Unit (TJA1051T)       RobStride Motor
  HY2.0-4P Port
  G32 (Yellow) -------> CAN_TX -----> CAN H ---------> CAN H
  G33 (White)  -------> CAN_RX -----> CAN L ---------> CAN L
  5V  (Red)    -------> VCC
  GND (Black)  -------> GND -------> GND -------------> GND
```

### ESP32 TWAI Configuration

The ESP32 has a built-in TWAI (Two-Wire Automotive Interface) controller,
which is CAN 2.0B compatible. The Mini CAN Unit (TJA1051T) acts as the
physical transceiver.

```cpp
#include "driver/twai.h"

// Pin configuration for M5StickC Plus 2 + Mini CAN Unit
#define CAN_TX_PIN GPIO_NUM_32
#define CAN_RX_PIN GPIO_NUM_33

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Install and start TWAI driver
twai_driver_install(&g_config, &t_config, &f_config);
twai_start();
```

## CAN Frame ID Allocation

| ID Range       | Purpose                    |
|----------------|----------------------------|
| 0x200 + ID     | MIT mode control commands  |
| 0x300 + ID     | Position mode commands     |
| 0x400 + ID     | Speed mode commands        |
| 0x500 + ID     | System status queries      |
| 0x600 + ID     | System configuration       |

Where ID is the motor's CAN ID (typically 1-127).

## Control Modes

### MIT Mode (Mode 0) - Direct Torque Control

Best for: Fast response applications, force feedback, balancing

```
Command frame (8 bytes):
  Bytes 0-3: p_des (int32) - Target position in radians
  Bytes 4-5: v_des (int16) - Target velocity in rad/s
  Bytes 6:   kp (uint8)    - Position gain (scaled)
  Bytes 7:   kd (uint8)    - Velocity gain (scaled)
  (Feed-forward torque encoded in ID or separate byte depending on firmware)
```

### Position Mode (Mode 1) - Position Closed-Loop

Best for: Precise positioning, robot joint control

Parameters: Position, Velocity limit, Max Torque limit

### Speed Mode (Mode 2) - Speed Closed-Loop

Best for: Constant speed applications

Parameters: Target velocity, Max Torque limit

## Motor Status Feedback (8 bytes)

```
  Bytes 0-3: position (int32) - Current position in radians
  Bytes 4-5: velocity (int16) - Current velocity in rad/s
  Bytes 6:   torque (int16)   - Current torque in Nm
  Byte 7:    mode (uint8)     - Current mode
             error (uint8)    - Error code
```

## Arduino Implementation (ESP32 TWAI)

```cpp
#include "driver/twai.h"

class RobStrideMotor {
private:
    uint8_t motor_id;

public:
    RobStrideMotor(uint8_t id) : motor_id(id) {}

    bool begin(gpio_num_t tx_pin, gpio_num_t rx_pin) {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
        if (err != ESP_OK) return false;

        err = twai_start();
        return (err == ESP_OK);
    }

    void enable() {
        twai_message_t msg;
        msg.identifier = 0x200 + motor_id;
        msg.extd = 1;  // Extended frame
        msg.data_length_code = 8;
        memset(msg.data, 0xFF, 8);  // Enable command
        twai_transmit(&msg, pdMS_TO_TICKS(100));
    }

    void sendMITCommand(float position, float velocity, float kp, float kd, float torque) {
        twai_message_t msg;
        msg.identifier = 0x200 + motor_id;
        msg.extd = 1;
        msg.data_length_code = 8;

        int32_t pos_int = (int32_t)(position * 1000.0f);
        msg.data[0] = pos_int & 0xFF;
        msg.data[1] = (pos_int >> 8) & 0xFF;
        msg.data[2] = (pos_int >> 16) & 0xFF;
        msg.data[3] = (pos_int >> 24) & 0xFF;
        msg.data[4] = (uint8_t)(velocity * 1000.0f);
        msg.data[5] = (uint8_t)(kp * 5.0f);
        msg.data[6] = (uint8_t)(kd * 500.0f);
        msg.data[7] = (uint8_t)(torque * 10.0f);

        twai_transmit(&msg, pdMS_TO_TICKS(100));
    }

    void disable() {
        twai_message_t msg;
        msg.identifier = 0x200 + motor_id;
        msg.extd = 1;
        msg.data_length_code = 8;
        memset(msg.data, 0x00, 8);  // Disable command
        twai_transmit(&msg, pdMS_TO_TICKS(100));
    }
};
```

## Error Codes

| Code | Description           | Solution                          |
|------|-----------------------|-----------------------------------|
| 0x01 | Communication timeout | Check CAN wiring and termination  |
| 0x02 | Parameter out of range| Check control parameter ranges    |
| 0x03 | Motor overcurrent     | Check load and torque limits      |
| 0x04 | Position overflow     | Check limits and target position  |
| 0x05 | Temperature too high  | Check cooling and reduce load     |

## Mini CAN Unit (TJA1051T/3)

- **SKU**: U179
- **Transceiver**: TJA1051T/3
- **Max Speed**: 1 Mbit/s
- **Standard**: ISO 11898-2:2016, SAE J2284
- **Protection**: EMC + ESD
- **Power**: Grove 5V or VH3.96 terminal (9-24V)
- **Pinout**: Yellow = CAN_TX, White = CAN_RX (via HY2.0-4P)
- **Note**: This is a transceiver only (no CAN controller). The ESP32's
  built-in TWAI controller provides the CAN protocol handling.

## References

- RobStride Control Library: https://github.com/Seeed-Projects/RobStride_Control
- RobStride GitHub: https://github.com/RobStride
- ESP32 TWAI docs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
- Mini CAN Unit: https://docs.m5stack.com/en/unit/Unit-Mini%20CAN
- TJA1051T datasheet: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/unit/Unit-Mini%20CAN/TJA1051T.pdf
