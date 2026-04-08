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

// Loop timing diagnostics
static uint32_t loopMaxUs = 0;
static uint32_t loopTotalUs = 0;
static uint32_t loopOverruns = 0;

// WiFi state
static bool     wifiConnected = false;
static String   wifiIP = "0.0.0.0";

// Previous arm switch states for edge detection
static bool prevDriveArmSwitch = false;
static bool prevArmArmSwitch   = false;

// ---------------------------------------------------------------------------
// Serial debug console -- simulated input override
// ---------------------------------------------------------------------------
static bool  simEnabled = false;
static float simThrottle = 0.0f;
static float simSteering = 0.0f;
static bool  testModeActive = false;
static char  serialBuf[128];
static int   serialBufLen = 0;

static void printSerialHelp() {
    Serial.println();
    Serial.println("--- Serial Debug Commands ---");
    Serial.println("  sim on / sim off  Enable/disable simulated RC input");
    Serial.println("  t <val>           Set sim throttle (-1.0 to 1.0)");
    Serial.println("  s <val>           Set sim steering (-1.0 to 1.0)");
    Serial.println("  arm / disarm      Arm/disarm drive motors");
    Serial.println("  arm arms          Arm robot arm motors");
    Serial.println("  disarm arms       Disarm robot arm motors");
    Serial.println("  status            Print full status");
    Serial.println("--- Low-Level Motor Debug ---");
    Serial.println("  ping <id>                   Ping motor, print feedback");
    Serial.println("  stop <id>                   Stop motor (clear fault)");
    Serial.println("  enable <id>                 Enable motor");
    Serial.println("  zero <id>                   Set zero position");
    Serial.println("  mode <id> <0-3>             Set run mode (0=MIT 1=Pos 2=Spd 3=Cur)");
    Serial.println("  rp <id> <paramHex>          Read param (e.g. rp 10 7017)");
    Serial.println("  wp <id> <paramHex> <float>  Write float param");
    Serial.println("  wpu <id> <paramHex> <u8>    Write u8 param");
    Serial.println("  pos <id> <pos> <spdLim>     Send position + speed_limit");
    Serial.println("  spd <id> <speed> <curLim>   Send speed + current_limit");
    Serial.println("  test <id>                   Run automated motor test");
    Serial.println("  help                        Show this help");
    Serial.println("-----------------------------");
}

// ---------------------------------------------------------------------------
// Automated motor test sequence
// ---------------------------------------------------------------------------
static void runMotorTest(uint8_t id) {
    Serial.println("========================================");
    Serial.printf("[TEST] Automated motor test for CAN ID %d\n", id);
    Serial.println("========================================");
    int pass = 0, fail = 0;

    auto check = [&](const char* name, bool condition, const char* detail) {
        if (condition) {
            Serial.printf("  PASS: %s -- %s\n", name, detail);
            pass++;
        } else {
            Serial.printf("  FAIL: %s -- %s\n", name, detail);
            fail++;
        }
    };

    // Helper: read velocity feedback from motor via param read (more reliable than ping)
    auto readVel = [&]() -> float {
        float v = 0;
        canBus.readParamSync(id, CAN_HOST_ID, RobstrideParam::MECH_VEL, v, 100);
        return v;
    };
    auto readPos = [&]() -> float {
        float p = 0;
        canBus.readParamSync(id, CAN_HOST_ID, RobstrideParam::MECH_POS, p, 100);
        return p;
    };

    // Drain any stale CAN messages from the RX queue
    {
        RobstrideFeedback drain_fb;
        for (int d = 0; d < 50; d++) {
            if (!canBus.receiveFeedback(drain_fb, 1)) break;
        }
    }

    // --- Test 1+2: Stop, Zero, CSP Enable, Communication check ---
    // Motor must be enabled to respond to param reads reliably.
    // So we combine the enable and communication check.
    Serial.println("\n[TEST 1] Stop, CSP Enable, Zero");
    canBus.stopMotor(id, CAN_HOST_ID, true);
    delay(200);
    canBus.setRunMode(id, CAN_HOST_ID, RobstrideRunMode::CSP);
    delay(10);
    canBus.enableMotor(id, CAN_HOST_ID);
    delay(50);
    canBus.setZeroPosition(id, CAN_HOST_ID);
    delay(100);

    // First verify the CAN bus works with a raw ping
    Serial.println("\n[TEST 2] Communication check");
    Serial.println("[TEST 2] Sending motion ping to wake motor...");
    canBus.sendMotionPing(id);
    delay(50);

    // Check for ANY response on the bus
    bool any_response = false;
    for (int i = 0; i < 10; i++) {
        RobstrideFeedback pfb;
        if (canBus.receiveFeedback(pfb, 20)) {
            Serial.printf("[TEST 2] Got CAN response: motor=%d mode=%d pos=%.2f vel=%.2f\n",
                          pfb.motor_id, pfb.mode, pfb.position, pfb.velocity);
            any_response = true;
            break;
        }
    }
    if (!any_response) {
        canBus.printBusStatus();
        check("CAN bus alive", false, "no response to motion ping -- check CAN wiring/power");
        Serial.println("[TEST] ABORTED -- no CAN communication");
        Serial.printf("[TEST] Results: %d passed, %d failed\n", pass, fail);
        return;
    }
    check("CAN bus alive", true, "motor responded to ping");

    // Now try param read
    float run_mode_val = 0;
    bool rm_ok = false;
    uint8_t rm_u8 = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (canBus.readParamSync(id, CAN_HOST_ID, RobstrideParam::RUN_MODE, run_mode_val, 200)) {
            memcpy(&rm_u8, &run_mode_val, 1);
            rm_ok = true;
            break;
        }
        Serial.printf("[TEST 2] Param read attempt %d failed, retrying...\n", retry + 1);
        delay(100);
    }
    if (!rm_ok) {
        check("Param read works", false, "no param read response (motor may not support param reads in this state)");
        rm_u8 = 255;
    } else {
        check("Param read works", true, "got response");
    }
    char rm_detail[64];
    snprintf(rm_detail, sizeof(rm_detail), "run_mode=%d (expected 5)", rm_u8);
    check("Run mode = CSP (5)", rm_u8 == 5, rm_detail);

    float pos_after_zero = readPos();
    char pz_detail[64];
    snprintf(pz_detail, sizeof(pz_detail), "pos=%.3f rad (note: zero may not reset across power cycles)", pos_after_zero);
    check("Position readable", true, pz_detail);

    // Use current position as the base for relative target tests
    float base_pos = pos_after_zero;

    // --- Test 3: Position hold ---
    Serial.println("\n[TEST 3] Position hold at current");
    canBus.sendPositionCommand(id, CAN_HOST_ID, base_pos, 5.0f);
    delay(1000);
    float hold_vel = readVel();
    char hv_detail[64];
    snprintf(hv_detail, sizeof(hv_detail), "vel=%.3f rad/s (expected near 0)", hold_vel);
    check("Velocity near 0 during hold", fabsf(hold_vel) < 1.0f, hv_detail);

    // --- Test 4: Forward motion ---
    float fwd_target = base_pos + 3.0f;
    Serial.printf("\n[TEST 4] Forward to %.1f rad (base + 3.0)\n", fwd_target);
    canBus.sendPositionCommand(id, CAN_HOST_ID, fwd_target, 10.0f);
    delay(200);
    float fwd_vel = readVel();
    char fv_detail[64];
    snprintf(fv_detail, sizeof(fv_detail), "vel=%.3f rad/s (expected > 0.1)", fwd_vel);
    check("Positive velocity", fwd_vel > 0.1f, fv_detail);

    delay(1500);
    float fwd_pos = readPos();
    char fp_detail[64];
    snprintf(fp_detail, sizeof(fp_detail), "pos=%.3f rad (expected near %.1f)", fwd_pos, fwd_target);
    check("Reached target", fabsf(fwd_pos - fwd_target) < 1.0f, fp_detail);

    // --- Test 5: Reverse motion ---
    float rev_target = base_pos - 3.0f;
    Serial.printf("\n[TEST 5] Reverse to %.1f rad (base - 3.0)\n", rev_target);
    canBus.sendPositionCommand(id, CAN_HOST_ID, rev_target, 10.0f);
    delay(500);
    float rev_vel = readVel();
    char rv_detail[64];
    snprintf(rv_detail, sizeof(rv_detail), "vel=%.3f rad/s (expected < -0.5)", rev_vel);
    check("Negative velocity", rev_vel < -0.5f, rv_detail);

    delay(2000);
    float rev_pos = readPos();
    char rp_detail[64];
    snprintf(rp_detail, sizeof(rp_detail), "pos=%.3f rad (expected near %.1f)", rev_pos, rev_target);
    check("Reached target", fabsf(rev_pos - rev_target) < 1.0f, rp_detail);

    // --- Test 6: Speed limit ---
    float spd_target = base_pos + 50.0f;
    Serial.println("\n[TEST 6] Speed limit test (target +50 rad from base, limit 2 rad/s)");
    canBus.sendPositionCommand(id, CAN_HOST_ID, spd_target, 2.0f);
    delay(1000);
    float sl_vel = readVel();
    char sv_detail[64];
    snprintf(sv_detail, sizeof(sv_detail), "vel=%.3f rad/s (expected near 2.0, < 5.0)", sl_vel);
    check("Speed limited", sl_vel > 0.5f && sl_vel < 5.0f, sv_detail);

    // --- Test 7: Stop (hold current position) ---
    Serial.println("\n[TEST 7] Stop at current position");
    float stop_pos = readPos();
    canBus.sendPositionCommand(id, CAN_HOST_ID, stop_pos, 0.5f);
    delay(1000);
    float stop_vel = readVel();
    char stv_detail[64];
    snprintf(stv_detail, sizeof(stv_detail), "vel=%.3f rad/s (expected near 0)", stop_vel);
    check("Velocity near 0 after stop", fabsf(stop_vel) < 1.0f, stv_detail);

    // --- Test 8: Disable ---
    Serial.println("\n[TEST 8] Disable motor");
    canBus.stopMotor(id, CAN_HOST_ID, false);
    delay(100);
    check("Stop command sent", true, "motor disabled");

    // --- Summary ---
    Serial.println();
    Serial.println("========================================");
    Serial.printf("[TEST] Results: %d passed, %d failed out of %d tests\n", pass, fail, pass + fail);
    if (fail == 0) {
        Serial.println("[TEST] ALL TESTS PASSED");
    } else {
        Serial.println("[TEST] SOME TESTS FAILED");
    }
    Serial.println("========================================");
}

static void processSerialCommand(const char* cmd) {
    // Skip leading whitespace
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;

    if (strcmp(cmd, "help") == 0) {
        printSerialHelp();

    } else if (strcmp(cmd, "sim on") == 0) {
        simEnabled = true;
        simThrottle = 0.0f;
        simSteering = 0.0f;
        Serial.println("[Sim] Enabled -- RC input overridden. Use 't <val>' and 's <val>'.");

    } else if (strcmp(cmd, "sim off") == 0) {
        simEnabled = false;
        simThrottle = 0.0f;
        simSteering = 0.0f;
        Serial.println("[Sim] Disabled -- returning to RC input.");

    } else if (cmd[0] == 't' && (cmd[1] == ' ' || cmd[1] == '\t')) {
        float val = atof(cmd + 2);
        if (val < -1.0f) val = -1.0f;
        if (val > 1.0f) val = 1.0f;
        simThrottle = val;
        Serial.printf("[Sim] Throttle = %.2f (%.1f rad/s = %.0f RPM)\n",
                      simThrottle,
                      simThrottle * driveCtrl.getMaxSpeed(),
                      simThrottle * driveCtrl.getMaxSpeed() * RAD_S_TO_RPM);

    } else if (cmd[0] == 's' && (cmd[1] == ' ' || cmd[1] == '\t')) {
        float val = atof(cmd + 2);
        if (val < -1.0f) val = -1.0f;
        if (val > 1.0f) val = 1.0f;
        simSteering = val;
        Serial.printf("[Sim] Steering = %.2f\n", simSteering);

    } else if (strcmp(cmd, "arm") == 0) {
        Serial.println("[Sim] Arming drive motors...");
        motorMgr.armDriveMotors();

    } else if (strcmp(cmd, "disarm") == 0) {
        Serial.println("[Sim] Disarming drive motors...");
        driveCtrl.emergencyStop();
        motorMgr.disarmDriveMotors();

    } else if (strcmp(cmd, "arm arms") == 0) {
        Serial.println("[Sim] Arming arm motors...");
        motorMgr.armArmMotors();

    } else if (strcmp(cmd, "disarm arms") == 0) {
        Serial.println("[Sim] Disarming arm motors...");
        armCtrl.holdPosition();
        motorMgr.disarmArmMotors();

    } else if (strcmp(cmd, "status") == 0) {
        uint32_t now = millis();
        Serial.println("==================================================");
        Serial.printf("[Status] t=%lu sim=%s thr=%.2f str=%.2f\n",
                      now, simEnabled ? "ON" : "OFF", simThrottle, simSteering);
        Serial.printf("[Status] link=%d drv_armed=%d arm_armed=%d\n",
                      crsfRx.isLinkUp(), motorMgr.isDriveArmed(), motorMgr.isArmArmed());
        Serial.printf("[Status] max_speed=%.1f rad/s (%.0f RPM) horizon=%.1f s\n",
                      driveCtrl.getMaxSpeed(),
                      driveCtrl.getMaxSpeed() * RAD_S_TO_RPM,
                      driveCtrl.getPositionHorizon());

        for (int i = 0; i < motorMgr.motorCount(); i++) {
            const MotorState& m = motorMgr.getMotor(i);
            Serial.printf("[Motor] ID=%2d %s | pos=%7.2f (raw=%6.2f unwrap_off=%6.2f) | vel=%6.1f rad/s (%5.0f RPM) | trq=%5.2f Nm | tmp=%.1fC | err=0x%02X\n",
                          m.can_id,
                          m.online ? "ON " : "OFF",
                          m.position, m.raw_position, m.unwrap_offset,
                          m.velocity, m.velocity * RAD_S_TO_RPM,
                          m.torque,
                          m.temperature,
                          m.errors);
        }
        driveCtrl.printDebug();
        canBus.printBusStatus();
        Serial.println("==================================================");

    } else if (strncmp(cmd, "ping ", 5) == 0) {
        uint8_t id = (uint8_t)atoi(cmd + 5);
        Serial.printf("[Dbg] Pinging motor %d...\n", id);
        canBus.sendMotionPing(id);
        delay(20);
        RobstrideFeedback fb;
        if (canBus.receiveFeedback(fb, 50)) {
            Serial.printf("[Dbg] Response: motor=%d pos=%.3f vel=%.3f trq=%.3f tmp=%.1f mode=%d err=0x%02X\n",
                          fb.motor_id, fb.position, fb.velocity, fb.torque,
                          fb.temperature, fb.mode, fb.errors);
        } else {
            Serial.printf("[Dbg] No response from motor %d\n", id);
        }

    } else if (strncmp(cmd, "stop ", 5) == 0) {
        uint8_t id = (uint8_t)atoi(cmd + 5);
        bool ok = canBus.stopMotor(id, CAN_HOST_ID, true);
        Serial.printf("[Dbg] Stop motor %d: %s\n", id, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "enable ", 7) == 0) {
        uint8_t id = (uint8_t)atoi(cmd + 7);
        bool ok = canBus.enableMotor(id, CAN_HOST_ID);
        Serial.printf("[Dbg] Enable motor %d: %s\n", id, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "zero ", 5) == 0) {
        uint8_t id = (uint8_t)atoi(cmd + 5);
        bool ok = canBus.setZeroPosition(id, CAN_HOST_ID);
        Serial.printf("[Dbg] Zero motor %d: %s\n", id, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "mode ", 5) == 0) {
        uint8_t id = 0;
        int modeInt = 0;
        sscanf(cmd + 5, "%hhu %d", &id, &modeInt);
        bool ok = canBus.setRunMode(id, CAN_HOST_ID, static_cast<RobstrideRunMode>(modeInt));
        Serial.printf("[Dbg] Set motor %d run_mode=%d: %s\n", id, modeInt, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "rp ", 3) == 0) {
        uint8_t id = 0;
        unsigned int paramHex = 0;
        sscanf(cmd + 3, "%hhu %x", &id, &paramHex);
        uint16_t param = (uint16_t)paramHex;
        float val = 0;
        Serial.printf("[Dbg] Reading param 0x%04X from motor %d...\n", param, id);
        if (canBus.readParamSync(id, CAN_HOST_ID, param, val, 100)) {
            Serial.printf("[Dbg] Param 0x%04X = %.6f (hex: ", param, val);
            uint8_t* bytes = (uint8_t*)&val;
            for (int b = 0; b < 4; b++) Serial.printf("%02X", bytes[b]);
            Serial.println(")");
        } else {
            Serial.printf("[Dbg] No response for param 0x%04X from motor %d\n", param, id);
        }

    } else if (strncmp(cmd, "wpu ", 4) == 0) {
        uint8_t id = 0;
        unsigned int paramHex = 0;
        unsigned int u8val = 0;
        sscanf(cmd + 4, "%hhu %x %u", &id, &paramHex, &u8val);
        uint16_t param = (uint16_t)paramHex;
        bool ok = canBus.writeU8Param(id, CAN_HOST_ID, param, (uint8_t)u8val);
        Serial.printf("[Dbg] Write param 0x%04X = %u (u8) to motor %d: %s\n",
                      param, u8val, id, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "wp ", 3) == 0) {
        uint8_t id = 0;
        unsigned int paramHex = 0;
        float fval = 0;
        sscanf(cmd + 3, "%hhu %x %f", &id, &paramHex, &fval);
        uint16_t param = (uint16_t)paramHex;
        bool ok = canBus.writeFloatParam(id, CAN_HOST_ID, param, fval);
        Serial.printf("[Dbg] Write param 0x%04X = %.4f to motor %d: %s\n",
                      param, fval, id, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "pos ", 4) == 0) {
        uint8_t id = 0;
        float posVal = 0, spdLim = 0;
        sscanf(cmd + 4, "%hhu %f %f", &id, &posVal, &spdLim);
        bool ok = canBus.sendPositionCommand(id, CAN_HOST_ID, posVal, spdLim);
        Serial.printf("[Dbg] Position cmd motor %d: pos=%.3f spd_lim=%.3f: %s\n",
                      id, posVal, spdLim, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "spd ", 4) == 0) {
        uint8_t id = 0;
        float spdVal = 0, curLim = 0;
        sscanf(cmd + 4, "%hhu %f %f", &id, &spdVal, &curLim);
        bool ok = canBus.sendSpeedCommand(id, CAN_HOST_ID, spdVal, curLim);
        Serial.printf("[Dbg] Speed cmd motor %d: spd=%.3f cur_lim=%.3f: %s\n",
                      id, spdVal, curLim, ok ? "sent" : "FAIL");

    } else if (strncmp(cmd, "test ", 5) == 0) {
        uint8_t id = (uint8_t)atoi(cmd + 5);
        Serial.printf("[Cmd] Running motor test on ID %d -- pausing control loop\n", id);
        driveCtrl.emergencyStop();
        motorMgr.disarmAll();
        testModeActive = true;
        runMotorTest(id);
        testModeActive = false;

    } else {
        Serial.printf("[Cmd] Unknown: '%s' -- type 'help'\n", cmd);
    }
}

static void pollSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBufLen > 0) {
                serialBuf[serialBufLen] = '\0';
                processSerialCommand(serialBuf);
                serialBufLen = 0;
            }
        } else if (serialBufLen < (int)sizeof(serialBuf) - 1) {
            serialBuf[serialBufLen++] = c;
        }
    }
}

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
    delay(1000);
    Serial.println();
    Serial.println();
    Serial.println("========================================");
    Serial.println("  Hopscotch Robot Controller");
    Serial.println("========================================");
    Serial.flush();

    // 1. Settings (needs LittleFS)
    settingsMgr.begin();

    // Migrate stale defaults from previous firmware versions
    bool settings_migrated = false;
    if (settingsMgr.settings.max_drive_speed < DEFAULT_MAX_DRIVE_SPEED_RAD_S - 0.1f) {
        Serial.printf("[Main] Migrating max_drive_speed: %.1f -> %.1f\n",
                      settingsMgr.settings.max_drive_speed, DEFAULT_MAX_DRIVE_SPEED_RAD_S);
        settingsMgr.settings.max_drive_speed = DEFAULT_MAX_DRIVE_SPEED_RAD_S;
        settings_migrated = true;
    }
    if (settingsMgr.settings.position_horizon_sec < DEFAULT_POSITION_HORIZON_SEC - 0.1f) {
        Serial.printf("[Main] Migrating position_horizon_sec: %.1f -> %.1f\n",
                      settingsMgr.settings.position_horizon_sec, DEFAULT_POSITION_HORIZON_SEC);
        settingsMgr.settings.position_horizon_sec = DEFAULT_POSITION_HORIZON_SEC;
        settings_migrated = true;
    }
    if (settings_migrated) {
        settingsMgr.save();
    }

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
    Serial.printf("[Main] Max drive speed: %.1f rad/s (%.0f RPM), Horizon: %.1f s\n",
                  settingsMgr.settings.max_drive_speed,
                  settingsMgr.settings.max_drive_speed * RAD_S_TO_RPM,
                  settingsMgr.settings.position_horizon_sec);
    printSerialHelp();

    lastControlTick = millis();
    lastDisplayTick = millis();
    lastWsTick = millis();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
    uint32_t now = millis();

    // Process serial debug commands (non-blocking)
    pollSerialCommands();

    // -----------------------------------------------------------------------
    // 50 Hz control loop
    // -----------------------------------------------------------------------
    if (now - lastControlTick >= CONTROL_LOOP_PERIOD_MS) {
        uint32_t tickStartUs = micros();
        lastControlTick = now;
        controlTickCount++;
        float dt = static_cast<float>(CONTROL_LOOP_PERIOD_MS) / 1000.0f;

        // 1. Read CRSF data
        crsfRx.update();

        // 2. Scan for motors and process feedback (skip during test mode
        //    so the test's readParamSync can use the CAN bus exclusively)
        if (!testModeActive) {
            motorMgr.scanNextMotor();
            motorMgr.processFeedback();
            motorMgr.checkTimeouts(500);
        }

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

        // Override with simulated values if sim mode is active
        if (simEnabled) {
            throttle = simThrottle;
            steering = simSteering;
        }

        // 4. Arm/disarm on switch edges (only from RC, not sim -- sim uses serial commands)
        bool driveSwNow = isSwitchActive(drive_arm_sw);
        bool armSwNow   = isSwitchActive(arm_arm_sw);

        if (crsfRx.isLinkUp() && !simEnabled) {
            // Drive arm toggle
            if (driveSwNow && !prevDriveArmSwitch) {
                if (!motorMgr.isDriveArmed()) {
                    Serial.printf("[Main] t=%lu DRIVE ARM requested via RC switch\n", now);
                    motorMgr.armDriveMotors();
                }
            } else if (!driveSwNow && prevDriveArmSwitch) {
                if (motorMgr.isDriveArmed()) {
                    Serial.printf("[Main] t=%lu DRIVE DISARM requested via RC switch\n", now);
                    driveCtrl.emergencyStop();
                    motorMgr.disarmDriveMotors();
                }
            }

            // Arm motors toggle
            if (armSwNow && !prevArmArmSwitch) {
                if (!motorMgr.isArmArmed()) {
                    Serial.printf("[Main] t=%lu ARMS ARM requested via RC switch\n", now);
                    motorMgr.armArmMotors();
                }
            } else if (!armSwNow && prevArmArmSwitch) {
                if (motorMgr.isArmArmed()) {
                    Serial.printf("[Main] t=%lu ARMS DISARM requested via RC switch\n", now);
                    armCtrl.holdPosition();
                    motorMgr.disarmArmMotors();
                }
            }
        }

        if (!simEnabled) {
            prevDriveArmSwitch = driveSwNow;
            prevArmArmSwitch = armSwNow;
        }

        // 5. Signal loss failsafe (skip in sim mode -- sim provides its own input)
        if (!simEnabled && !crsfRx.isLinkUp()) {
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

        // 8. Loop timing measurement
        uint32_t tickElapsedUs = micros() - tickStartUs;
        loopTotalUs += tickElapsedUs;
        if (tickElapsedUs > loopMaxUs) {
            loopMaxUs = tickElapsedUs;
        }
        if (tickElapsedUs > CONTROL_LOOP_PERIOD_MS * 1000) {
            loopOverruns++;
        }

        // 9. Periodic debug output (every 2 seconds = 100 ticks at 50Hz)
        if (controlTickCount % 100 == 0) {
            uint32_t avgUs = loopTotalUs / 100;
            Serial.println("==================================================");
            Serial.printf("[Loop] t=%lu tick=%lu | avg=%luus max=%luus overruns=%lu\n",
                          now, controlTickCount, avgUs, loopMaxUs, loopOverruns);
            Serial.printf("[Loop] link=%d rssi=%d lq=%d | drv_armed=%d arm_armed=%d | thr=%.2f str=%.2f | wifi=%d | sim=%s\n",
                          crsfRx.isLinkUp(), crsfRx.getRssi(), crsfRx.getLinkQuality(),
                          motorMgr.isDriveArmed(), motorMgr.isArmArmed(),
                          throttle, steering, wifiConnected,
                          simEnabled ? "ON" : "off");

            // Reset timing accumulators
            loopMaxUs = 0;
            loopTotalUs = 0;
            loopOverruns = 0;

            // Raw channel values
            Serial.print("[CRSF] CH: ");
            for (int i = 0; i < 16; i++) {
                Serial.printf("%d", crsfRx.getChannel(i));
                if (i < 15) Serial.print(",");
            }
            Serial.println();

            // Motor online status with position/velocity/torque
            for (int i = 0; i < motorMgr.motorCount(); i++) {
                const MotorState& m = motorMgr.getMotor(i);
                Serial.printf("[Motor] ID=%2d %s | pos=%7.2f (raw=%6.2f off=%6.2f) | vel=%6.1f rad/s (%5.0f RPM) | trq=%5.2f Nm | tmp=%.1fC | err=0x%02X\n",
                              m.can_id,
                              m.online ? "ON " : "OFF",
                              m.position, m.raw_position, m.unwrap_offset,
                              m.velocity, m.velocity * RAD_S_TO_RPM,
                              m.torque,
                              m.temperature,
                              m.errors);
            }

            // Drive controller per-motor closed-loop status
            driveCtrl.printDebug();

            // CAN bus diagnostics
            canBus.printBusStatus();
            Serial.println("==================================================");
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
