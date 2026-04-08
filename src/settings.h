#pragma once

#include <cstdint>
#include <Arduino.h>

static constexpr const char* SETTINGS_PATH = "/settings.json";

struct ChannelMap {
    uint8_t steering;
    uint8_t throttle;
    uint8_t arm_disarm_drive;
    uint8_t arm_disarm_arms;
    uint8_t arm_left;
    uint8_t arm_right;
    uint8_t arm_mode;
    uint8_t arm_select_group;
    uint8_t arm_select_var;
    uint8_t arm_trigger_exec;
    uint8_t arm_trigger_home;
};

struct ArmCalibration {
    float center_left;      // delta from forward (rad)
    float center_right;     // delta from forward (rad)
    float backward_left;    // delta from forward (rad)
    float backward_right;   // delta from forward (rad)
    bool  calibrated;
};

struct MotorAssignments {
    uint8_t front_right;
    uint8_t back_right;
    uint8_t back_left;
    uint8_t front_left;
    uint8_t arm_left;
    uint8_t arm_right;
};

struct Settings {
    char     wifi_ssid[64];
    char     wifi_password[64];
    ChannelMap channel_map;
    uint16_t deadband;
    MotorAssignments motor_ids;
    float    max_drive_speed;
    float    position_horizon_sec;
    float    max_arm_speed;
    float    arm_range;
    ArmCalibration arm_cal;
};

class SettingsManager {
public:
    // Initialize LittleFS and load settings (or create defaults)
    bool begin();

    // Load settings from LittleFS
    bool load();

    // Save current settings to LittleFS
    bool save();

    // Reset to factory defaults
    void resetDefaults();

    // Serialize current settings to JSON string (for web API)
    String toJson() const;

    // Deserialize JSON string into settings (for web API)
    bool fromJson(const String& json);

    Settings settings;

private:
    void applyDefaults();
};
