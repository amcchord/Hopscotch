#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "config.h"
#include "settings.h"
#include "robstride.h"
#include "motor_manager.h"
#include "crsf.h"
#include "drive_controller.h"
#include "arm_controller.h"
#include "display.h"
#include "web_server.h"

// ---------------------------------------------------------------------------
// Global instances
// ---------------------------------------------------------------------------
static SettingsManager  settingsMgr;
static Robstride        canBus;
static MotorManager     motorMgr;
static CrsfReceiver     crsfRx;
static DriveController  driveCtrl;
static ArmController    armCtrl;
static Display          display;
static WebUI            webUI;

// Timing
static uint32_t lastControlTick = 0;
static uint32_t lastDisplayTick = 0;
static uint32_t lastWsTick      = 0;
static uint32_t controlTickCount = 0;

// WiFi state
static bool     wifiConnected = false;
static String   wifiIP = "0.0.0.0";

// Previous arm switch states for edge detection
static bool prevDriveArmSwitch = false;
static bool prevArmArmSwitch   = false;

// ---------------------------------------------------------------------------
// Apply settings to subsystems
// ---------------------------------------------------------------------------
static void applySettings() {
    const Settings& s = settingsMgr.settings;

    // Motor CAN IDs
    motorMgr.setMotorId(MotorRole::FrontRight, s.motor_ids.front_right);
    motorMgr.setMotorId(MotorRole::BackRight,  s.motor_ids.back_right);
    motorMgr.setMotorId(MotorRole::BackLeft,   s.motor_ids.back_left);
    motorMgr.setMotorId(MotorRole::FrontLeft,  s.motor_ids.front_left);
    motorMgr.setMotorId(MotorRole::ArmLeft,    s.motor_ids.arm_left);
    motorMgr.setMotorId(MotorRole::ArmRight,   s.motor_ids.arm_right);

    // Drive parameters
    driveCtrl.setMaxSpeed(s.max_drive_speed);
    driveCtrl.setPositionHorizon(s.position_horizon_sec);

    // Arm parameters
    armCtrl.setMaxArmSpeed(s.max_arm_speed);
    armCtrl.setArmRange(s.arm_range);

    Serial.println("[Main] Settings applied to subsystems");
}

// ---------------------------------------------------------------------------
// Callbacks for WebUI
// ---------------------------------------------------------------------------
static void onSettingsChanged() {
    applySettings();
}

static void onDisarmRequested() {
    Serial.println("[Main] Emergency disarm requested via web");
    driveCtrl.emergencyStop();
    armCtrl.holdPosition();
    motorMgr.disarmAll();
}

static bool onCanIdChange(uint8_t old_id, uint8_t new_id) {
    Serial.printf("[Main] CAN ID change: %d -> %d\n", old_id, new_id);
    return motorMgr.changeMotorCanIdOnBus(old_id, new_id);
}

// ---------------------------------------------------------------------------
// WiFi management
// ---------------------------------------------------------------------------
static void initWifi() {
    const Settings& s = settingsMgr.settings;
    WiFi.mode(WIFI_STA);
    WiFi.begin(s.wifi_ssid, s.wifi_password);
    Serial.printf("[WiFi] Connecting to %s...\n", s.wifi_ssid);
}

static void updateWifi() {
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected != wifiConnected) {
        wifiConnected = connected;
        if (connected) {
            wifiIP = WiFi.localIP().toString();
            Serial.printf("[WiFi] Connected: %s\n", wifiIP.c_str());
        } else {
            wifiIP = "0.0.0.0";
            Serial.println("[WiFi] Disconnected");
        }
        webUI.setWifiState(wifiConnected, wifiIP.c_str());
    }
}

// ---------------------------------------------------------------------------
// Apply deadband to a normalized channel value
// ---------------------------------------------------------------------------
static float applyDeadband(float value, float deadband_norm) {
    if (fabsf(value) < deadband_norm) return 0.0f;
    float sign = (value > 0) ? 1.0f : -1.0f;
    return sign * (fabsf(value) - deadband_norm) / (1.0f - deadband_norm);
}

// ---------------------------------------------------------------------------
// Detect rising edge on a switch channel (> 0.5 = active)
// ---------------------------------------------------------------------------
static bool isSwitchActive(float norm_value) {
    return norm_value > 0.5f;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  Hopscotch Robot Controller");
    Serial.println("========================================");

    // 1. Settings (needs LittleFS)
    settingsMgr.begin();

    // 2. Display
    display.begin();

    // 3. CAN bus
    if (!canBus.begin(PIN_CAN_TX, PIN_CAN_RX)) {
        Serial.println("[FATAL] CAN bus init failed!");
    }

    // 4. Motor manager
    motorMgr.begin(&canBus);
    applySettings();

    // 5. CRSF receiver
    crsfRx.begin(Serial1, PIN_CRSF_RX, PIN_CRSF_TX, CRSF_BAUDRATE);

    // 6. Controllers
    driveCtrl.begin(&motorMgr);
    armCtrl.begin(&motorMgr);

    // 7. WiFi
    initWifi();

    // 8. Web server (starts after WiFi begins connecting)
    webUI.begin(&settingsMgr, &motorMgr, &crsfRx);
    webUI.onSettingsChanged(onSettingsChanged);
    webUI.onDisarmRequested(onDisarmRequested);
    webUI.onCanIdChange(onCanIdChange);

    Serial.println("[Main] Initialization complete, entering control loop");
    Serial.printf("[Main] Control loop: %d Hz, Display: 25 fps, WS: 10 Hz\n", CONTROL_LOOP_HZ);

    lastControlTick = millis();
    lastDisplayTick = millis();
    lastWsTick = millis();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
    uint32_t now = millis();

    // -----------------------------------------------------------------------
    // 50 Hz control loop
    // -----------------------------------------------------------------------
    if (now - lastControlTick >= CONTROL_LOOP_PERIOD_MS) {
        lastControlTick = now;
        controlTickCount++;
        float dt = static_cast<float>(CONTROL_LOOP_PERIOD_MS) / 1000.0f;

        // 1. Read CRSF data
        crsfRx.update();

        // 2. Process CAN feedback
        motorMgr.processFeedback();
        motorMgr.checkTimeouts(500);

        // 3. Read channel inputs
        const Settings& s = settingsMgr.settings;
        float deadband_norm = static_cast<float>(s.deadband) / static_cast<float>(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID);

        float throttle_raw = crsfRx.getChannelNormalized(s.channel_map.throttle);
        float steering_raw = crsfRx.getChannelNormalized(s.channel_map.steering);
        float drive_arm_sw = crsfRx.getChannelNormalized(s.channel_map.arm_disarm_drive);
        float arm_arm_sw   = crsfRx.getChannelNormalized(s.channel_map.arm_disarm_arms);
        float arm_left_in  = crsfRx.getChannelNormalized(s.channel_map.arm_left);
        float arm_right_in = crsfRx.getChannelNormalized(s.channel_map.arm_right);

        float throttle = applyDeadband(throttle_raw, deadband_norm);
        float steering = applyDeadband(steering_raw, deadband_norm);
        float arm_l = applyDeadband(arm_left_in, deadband_norm);
        float arm_r = applyDeadband(arm_right_in, deadband_norm);

        // 4. Arm/disarm on switch edges
        bool driveSwNow = isSwitchActive(drive_arm_sw);
        bool armSwNow   = isSwitchActive(arm_arm_sw);

        if (crsfRx.isLinkUp()) {
            // Drive arm toggle
            if (driveSwNow && !prevDriveArmSwitch) {
                if (!motorMgr.isDriveArmed()) {
                    motorMgr.armDriveMotors();
                }
            } else if (!driveSwNow && prevDriveArmSwitch) {
                if (motorMgr.isDriveArmed()) {
                    driveCtrl.emergencyStop();
                    motorMgr.disarmDriveMotors();
                }
            }

            // Arm motors toggle
            if (armSwNow && !prevArmArmSwitch) {
                if (!motorMgr.isArmArmed()) {
                    motorMgr.armArmMotors();
                }
            } else if (!armSwNow && prevArmArmSwitch) {
                if (motorMgr.isArmArmed()) {
                    armCtrl.holdPosition();
                    motorMgr.disarmArmMotors();
                }
            }
        }

        prevDriveArmSwitch = driveSwNow;
        prevArmArmSwitch = armSwNow;

        // 5. Signal loss failsafe
        if (!crsfRx.isLinkUp()) {
            if (motorMgr.isDriveArmed()) {
                driveCtrl.emergencyStop();
            }
            if (motorMgr.isArmArmed()) {
                armCtrl.holdPosition();
            }
        } else {
            // 6. Drive control
            if (motorMgr.isDriveArmed()) {
                driveCtrl.update(throttle, steering, dt);
            }

            // 7. Arm control
            if (motorMgr.isArmArmed()) {
                armCtrl.update(arm_l, arm_r, dt);
            }
        }

        // 8. Periodic debug output (every 2 seconds)
        if (controlTickCount % 100 == 0) {
            Serial.printf("[Loop] t=%lu link=%d drv=%d arm=%d thr=%.2f str=%.2f\n",
                          now, crsfRx.isLinkUp(),
                          motorMgr.isDriveArmed(), motorMgr.isArmArmed(),
                          throttle, steering);
        }
    }

    // -----------------------------------------------------------------------
    // ~25 fps display update
    // -----------------------------------------------------------------------
    if (now - lastDisplayTick >= DISPLAY_PERIOD_MS) {
        lastDisplayTick = now;

        updateWifi();

        display.render(motorMgr, crsfRx,
                       wifiConnected, wifiIP.c_str(),
                       motorMgr.isDriveArmed(), motorMgr.isArmArmed());
    }

    // -----------------------------------------------------------------------
    // ~10 Hz WebSocket telemetry
    // -----------------------------------------------------------------------
    if (now - lastWsTick >= WEBSOCKET_PERIOD_MS) {
        lastWsTick = now;
        webUI.sendTelemetry();
    }
}
