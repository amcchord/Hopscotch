#include "settings.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <cmath>
#include "config.h"

void SettingsManager::applyDefaults() {
    strlcpy(settings.wifi_ssid,     DEFAULT_WIFI_SSID,     sizeof(settings.wifi_ssid));
    strlcpy(settings.wifi_password, DEFAULT_WIFI_PASSWORD, sizeof(settings.wifi_password));

    settings.channel_map.steering         = DEFAULT_CH_STEERING;
    settings.channel_map.throttle         = DEFAULT_CH_THROTTLE;
    settings.channel_map.arm_disarm_drive = DEFAULT_CH_ARM_DRIVE;
    settings.channel_map.arm_disarm_arms  = DEFAULT_CH_ARM_ARMS;
    settings.channel_map.arm_left         = DEFAULT_CH_ARM_LEFT;
    settings.channel_map.arm_right        = DEFAULT_CH_ARM_RIGHT;
    settings.channel_map.arm_mode         = DEFAULT_CH_ARM_MODE;
    settings.channel_map.arm_select_group = DEFAULT_CH_ARM_SELECT_GROUP;
    settings.channel_map.arm_select_var   = DEFAULT_CH_ARM_SELECT_VAR;
    settings.channel_map.arm_trigger_exec = DEFAULT_CH_ARM_TRIGGER_EXEC;
    settings.channel_map.arm_trigger_home = DEFAULT_CH_ARM_TRIGGER_HOME;

    settings.deadband = DEFAULT_DEADBAND;

    settings.motor_ids.front_right = DEFAULT_MOTOR_ID_FRONT_RIGHT;
    settings.motor_ids.back_right  = DEFAULT_MOTOR_ID_BACK_RIGHT;
    settings.motor_ids.back_left   = DEFAULT_MOTOR_ID_BACK_LEFT;
    settings.motor_ids.front_left  = DEFAULT_MOTOR_ID_FRONT_LEFT;
    settings.motor_ids.arm_left    = DEFAULT_MOTOR_ID_ARM_LEFT;
    settings.motor_ids.arm_right   = DEFAULT_MOTOR_ID_ARM_RIGHT;

    settings.max_drive_speed      = DEFAULT_MAX_DRIVE_SPEED_RAD_S;
    settings.position_horizon_sec = DEFAULT_POSITION_HORIZON_SEC;
    settings.max_arm_speed        = 5.0f;
    settings.arm_range            = 3.14f;

    settings.arm_cal.center_left    = 1.77f;
    settings.arm_cal.center_right   = -1.77f;
    settings.arm_cal.backward_left  = 3.54f;
    settings.arm_cal.backward_right = -3.54f;
    settings.arm_cal.calibrated     = false;
}

bool SettingsManager::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("[Settings] LittleFS mount failed");
        return false;
    }

    if (!load()) {
        Serial.println("[Settings] No saved settings, applying defaults");
        applyDefaults();
        save();
    }

    Serial.printf("[Settings] WiFi SSID: %s\n", settings.wifi_ssid);
    return true;
}

bool SettingsManager::load() {
    File f = LittleFS.open(SETTINGS_PATH, "r");
    if (!f) return false;

    String json = f.readString();
    f.close();

    return fromJson(json);
}

bool SettingsManager::save() {
    String json = toJson();

    File f = LittleFS.open(SETTINGS_PATH, "w");
    if (!f) {
        Serial.println("[Settings] Failed to open file for writing");
        return false;
    }

    f.print(json);
    f.close();
    Serial.println("[Settings] Saved to flash");
    return true;
}

void SettingsManager::resetDefaults() {
    applyDefaults();
    save();
}

String SettingsManager::toJson() const {
    JsonDocument doc;

    doc["wifi_ssid"]     = settings.wifi_ssid;
    doc["wifi_password"] = settings.wifi_password;

    JsonObject ch = doc["channel_map"].to<JsonObject>();
    ch["steering"]         = settings.channel_map.steering;
    ch["throttle"]         = settings.channel_map.throttle;
    ch["arm_disarm_drive"] = settings.channel_map.arm_disarm_drive;
    ch["arm_disarm_arms"]  = settings.channel_map.arm_disarm_arms;
    ch["arm_left"]         = settings.channel_map.arm_left;
    ch["arm_right"]        = settings.channel_map.arm_right;
    ch["arm_mode"]         = settings.channel_map.arm_mode;
    ch["arm_select_group"] = settings.channel_map.arm_select_group;
    ch["arm_select_var"]   = settings.channel_map.arm_select_var;
    ch["arm_trigger_exec"] = settings.channel_map.arm_trigger_exec;
    ch["arm_trigger_home"] = settings.channel_map.arm_trigger_home;

    doc["deadband"] = settings.deadband;

    JsonObject mo = doc["motor_ids"].to<JsonObject>();
    mo["front_right"] = settings.motor_ids.front_right;
    mo["back_right"]  = settings.motor_ids.back_right;
    mo["back_left"]   = settings.motor_ids.back_left;
    mo["front_left"]  = settings.motor_ids.front_left;
    mo["arm_left"]    = settings.motor_ids.arm_left;
    mo["arm_right"]   = settings.motor_ids.arm_right;

    doc["max_drive_speed"]      = settings.max_drive_speed;
    doc["position_horizon_sec"] = settings.position_horizon_sec;
    doc["max_arm_speed"]        = settings.max_arm_speed;
    doc["arm_range"]            = settings.arm_range;

    JsonObject ac = doc["arm_cal"].to<JsonObject>();
    ac["center_left"]    = settings.arm_cal.center_left;
    ac["center_right"]   = settings.arm_cal.center_right;
    ac["backward_left"]  = settings.arm_cal.backward_left;
    ac["backward_right"] = settings.arm_cal.backward_right;
    ac["calibrated"]     = settings.arm_cal.calibrated;

    String output;
    serializeJsonPretty(doc, output);
    return output;
}

bool SettingsManager::fromJson(const String& json) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.printf("[Settings] JSON parse error: %s\n", err.c_str());
        return false;
    }

    // Start from defaults so missing keys are safe
    applyDefaults();

    if (doc["wifi_ssid"].is<const char*>()) {
        strlcpy(settings.wifi_ssid, doc["wifi_ssid"].as<const char*>(), sizeof(settings.wifi_ssid));
    }
    if (doc["wifi_password"].is<const char*>()) {
        strlcpy(settings.wifi_password, doc["wifi_password"].as<const char*>(), sizeof(settings.wifi_password));
    }

    JsonObject ch = doc["channel_map"];
    if (!ch.isNull()) {
        if (ch["steering"].is<uint8_t>())         settings.channel_map.steering = ch["steering"];
        if (ch["throttle"].is<uint8_t>())         settings.channel_map.throttle = ch["throttle"];
        if (ch["arm_disarm_drive"].is<uint8_t>()) settings.channel_map.arm_disarm_drive = ch["arm_disarm_drive"];
        if (ch["arm_disarm_arms"].is<uint8_t>())  settings.channel_map.arm_disarm_arms = ch["arm_disarm_arms"];
        if (ch["arm_left"].is<uint8_t>())         settings.channel_map.arm_left = ch["arm_left"];
        if (ch["arm_right"].is<uint8_t>())        settings.channel_map.arm_right = ch["arm_right"];
        if (ch["arm_mode"].is<uint8_t>())         settings.channel_map.arm_mode = ch["arm_mode"];
        if (ch["arm_select_group"].is<uint8_t>()) settings.channel_map.arm_select_group = ch["arm_select_group"];
        if (ch["arm_select_var"].is<uint8_t>())   settings.channel_map.arm_select_var = ch["arm_select_var"];
        if (ch["arm_trigger_exec"].is<uint8_t>()) settings.channel_map.arm_trigger_exec = ch["arm_trigger_exec"];
        if (ch["arm_trigger_home"].is<uint8_t>()) settings.channel_map.arm_trigger_home = ch["arm_trigger_home"];
    }

    if (doc["deadband"].is<uint16_t>()) settings.deadband = doc["deadband"];

    JsonObject mo = doc["motor_ids"];
    if (!mo.isNull()) {
        if (mo["front_right"].is<uint8_t>()) settings.motor_ids.front_right = mo["front_right"];
        if (mo["back_right"].is<uint8_t>())  settings.motor_ids.back_right = mo["back_right"];
        if (mo["back_left"].is<uint8_t>())   settings.motor_ids.back_left = mo["back_left"];
        if (mo["front_left"].is<uint8_t>())  settings.motor_ids.front_left = mo["front_left"];
        if (mo["arm_left"].is<uint8_t>())    settings.motor_ids.arm_left = mo["arm_left"];
        if (mo["arm_right"].is<uint8_t>())   settings.motor_ids.arm_right = mo["arm_right"];
    }

    if (doc["max_drive_speed"].is<float>())      settings.max_drive_speed = doc["max_drive_speed"];
    if (doc["position_horizon_sec"].is<float>())  settings.position_horizon_sec = doc["position_horizon_sec"];
    if (doc["max_arm_speed"].is<float>())         settings.max_arm_speed = doc["max_arm_speed"];
    if (doc["arm_range"].is<float>())             settings.arm_range = doc["arm_range"];

    JsonObject ac = doc["arm_cal"];
    if (!ac.isNull()) {
        if (ac["center_left"].is<float>())    settings.arm_cal.center_left = ac["center_left"];
        if (ac["center_right"].is<float>())   settings.arm_cal.center_right = ac["center_right"];
        if (ac["backward_left"].is<float>())  settings.arm_cal.backward_left = ac["backward_left"];
        if (ac["backward_right"].is<float>()) settings.arm_cal.backward_right = ac["backward_right"];
        if (ac["calibrated"].is<bool>())      settings.arm_cal.calibrated = ac["calibrated"];

        // Migrate old absolute-position format to delta-from-forward format
        if (ac["forward_left"].is<float>() && ac["forward_right"].is<float>()) {
            float fwd_l = ac["forward_left"].as<float>();
            float fwd_r = ac["forward_right"].as<float>();
            if (fabsf(fwd_l) > 0.001f || fabsf(fwd_r) > 0.001f) {
                Serial.printf("[Settings] Migrating arm_cal from absolute to delta format (fwd_l=%.3f fwd_r=%.3f)\n",
                              fwd_l, fwd_r);
                settings.arm_cal.center_left    -= fwd_l;
                settings.arm_cal.center_right   -= fwd_r;
                settings.arm_cal.backward_left  -= fwd_l;
                settings.arm_cal.backward_right -= fwd_r;
            }
        }
    }

    return true;
}
