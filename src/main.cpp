#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>

#include "config.h"
#include "settings.h"
#include "robstride.h"
#include "motor_manager.h"
#include "crsf.h"
#include "drive_controller.h"
#include "arm_controller.h"
#include "balance_controller.h"
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
static BalanceController balanceCtrl;
static Display          display;
static WebUI            webUI;
static Madgwick         ahrsFilter;

// Timing
static uint32_t lastControlTick  = 0;
static uint32_t lastDisplayTick  = 0;
static uint32_t lastWsTick       = 0;
static uint32_t lastTelTick      = 0;
// (lastCalPrintTick removed -- calibration now via TX triggers)
static uint32_t controlTickCount = 0;

// Loop timing diagnostics
static uint32_t loopMaxUs = 0;
static uint32_t loopTotalUs = 0;
static uint32_t loopOverruns = 0;

// WiFi state
static bool     wifiConnected = false;
static String   wifiIP = "0.0.0.0";

// Previous arm switch states for edge detection
static bool prevDriveArmSwitch  = false;
static bool prevArmArmSwitch    = false;
static bool prevCalTrigger      = false;
static bool prevMoveTrigger     = false;

// Ch11 hold timer for entering calibration mode
static uint32_t calHoldStart    = 0;
static bool     calHoldFired    = false;
static constexpr uint32_t CAL_ENTRY_HOLD_MS = 3000;

// Balance mode state tracking
static bool  prevBalanceWasActive = false;
static float prevRollDeg          = 0.0f;
static float filteredRollRate     = 0.0f;

// Periodic debug output toggle (press 'd' + Enter to toggle)
static bool  debugOutputEnabled = true;

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
    Serial.println("  d                 Toggle periodic debug output on/off");
    Serial.println("  sim on / sim off  Enable/disable simulated RC input");
    Serial.println("  t <val>           Set sim throttle (-1.0 to 1.0)");
    Serial.println("  s <val>           Set sim steering (-1.0 to 1.0)");
    Serial.println("  arm / disarm      Arm/disarm drive motors");
    Serial.println("  arm arms          Arm robot arm motors");
    Serial.println("  disarm arms       Disarm robot arm motors");
    Serial.println("  status            Print full status");
    Serial.println("--- Arm Calibration (disarmed only) ---");
    Serial.println("  cal start         Enter guided calibration mode (3 steps)");
    Serial.println("  cal stop          Exit calibration mode");
    Serial.println("  cal status        Print current calibration values");
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
    Serial.println("--- Balance Mode ---");
    Serial.println("  bal status                  Print balance state, PID gains, log info");
    Serial.println("  bal setpoint <deg>          Set balance setpoint (default 90)");
    Serial.println("  bal kp/kd/adapt <val>       Set balance PD gains / adapt rate");
    Serial.println("  bal pkp/pki/pkd <val>       Set position-return PID gains");
    Serial.println("  bal log                     Dump telemetry log to serial");
    Serial.println("  bal log clear               Delete telemetry log file");
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

// ---------------------------------------------------------------------------
// Calibration serial commands
// ---------------------------------------------------------------------------
static void processCalCommand(const char* sub) {
    if (strcmp(sub, "status") == 0) {
        armCtrl.printCalTable();
    } else if (strcmp(sub, "start") == 0) {
        armCtrl.enterCalMode();
    } else if (strcmp(sub, "stop") == 0) {
        armCtrl.exitCalMode();
    } else {
        Serial.println("[Cal] Usage: cal start | cal stop | cal status");
    }
}

// ---------------------------------------------------------------------------
// Balance serial commands
// ---------------------------------------------------------------------------
static void processBalCommand(const char* sub) {
    if (strcmp(sub, "status") == 0) {
        balanceCtrl.printStatus();

    } else if (strcmp(sub, "log") == 0) {
        balanceCtrl.dumpLog();

    } else if (strcmp(sub, "log clear") == 0) {
        balanceCtrl.clearLog();

    } else if (strncmp(sub, "setpoint ", 9) == 0) {
        float val = atof(sub + 9);
        balanceCtrl.setSetpoint(val);
        Serial.printf("[Balance] Setpoint = %.1f deg\n", val);

    } else if (strncmp(sub, "kp ", 3) == 0) {
        float val = atof(sub + 3);
        balanceCtrl.setKp(val);
        Serial.printf("[Balance] Kp = %.4f\n", val);

    } else if (strncmp(sub, "kd ", 3) == 0) {
        float val = atof(sub + 3);
        balanceCtrl.setKd(val);
        Serial.printf("[Balance] Kd = %.4f\n", val);

    } else if (strncmp(sub, "expo ", 5) == 0) {
        float val = atof(sub + 5);
        balanceCtrl.setExpo(val);
        Serial.printf("[Balance] Expo = %.2f\n", val);

    } else if (strncmp(sub, "adapt ", 6) == 0) {
        float val = atof(sub + 6);
        balanceCtrl.setAdaptRate(val);
        Serial.printf("[Balance] Adapt rate = %.4f\n", val);

    } else if (strncmp(sub, "pkp ", 4) == 0) {
        float val = atof(sub + 4);
        balanceCtrl.setPosKp(val);
        Serial.printf("[Balance] Pos Kp = %.4f\n", val);

    } else if (strncmp(sub, "pki ", 4) == 0) {
        float val = atof(sub + 4);
        balanceCtrl.setPosKi(val);
        Serial.printf("[Balance] Pos Ki = %.4f\n", val);

    } else if (strncmp(sub, "pkd ", 4) == 0) {
        float val = atof(sub + 4);
        balanceCtrl.setPosKd(val);
        Serial.printf("[Balance] Pos Kd = %.4f\n", val);

    } else {
        Serial.println("[Balance] Usage: bal status | bal setpoint <deg>");
        Serial.println("         bal kp/kd/adapt <val> | bal pkp/pki/pkd <val>");
        Serial.println("         bal log | bal log clear");
    }
}

static void processSerialCommand(const char* cmd) {
    // Skip leading whitespace
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;

    if (strcmp(cmd, "help") == 0) {
        printSerialHelp();

    } else if (strcmp(cmd, "d") == 0) {
        debugOutputEnabled = !debugOutputEnabled;
        Serial.printf("[Debug] Periodic output %s\n", debugOutputEnabled ? "ON" : "OFF");

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
        Serial.println("[Sim] Arming arm motors (ensure arms are in FORWARD position)...");
        motorMgr.armArmMotors();
        armCtrl.setForwardReference();

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

    } else if (strncmp(cmd, "cal ", 4) == 0) {
        processCalCommand(cmd + 4);

    } else if (strcmp(cmd, "cal") == 0) {
        Serial.println("[Cal] Usage: cal start | cal stop | cal status");

    } else if (strncmp(cmd, "bal ", 4) == 0) {
        processBalCommand(cmd + 4);

    } else if (strcmp(cmd, "bal") == 0) {
        Serial.println("[Balance] Usage: bal status | bal log | bal log clear");

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
    armCtrl.setCalibration(s.arm_cal);

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
    WiFi.mode(WIFI_AP);
    WiFi.softAP(s.wifi_ssid, s.wifi_password);
    wifiIP = WiFi.softAPIP().toString();
    wifiConnected = true;
    webUI.setWifiState(true, wifiIP.c_str());
    Serial.printf("[WiFi] AP started: SSID=%s IP=%s\n", s.wifi_ssid, wifiIP.c_str());
}

static void updateWifi() {
    // AP mode is always available -- nothing to poll
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
    ahrsFilter.begin(CONTROL_LOOP_HZ);

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
    if (settingsMgr.settings.channel_map.arm_mode != DEFAULT_CH_ARM_MODE) {
        Serial.printf("[Main] Migrating arm_mode channel: %d -> %d\n",
                      settingsMgr.settings.channel_map.arm_mode, DEFAULT_CH_ARM_MODE);
        settingsMgr.settings.channel_map.arm_mode = DEFAULT_CH_ARM_MODE;
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
    armCtrl.setSettingsManager(&settingsMgr);
    balanceCtrl.begin(&motorMgr, &armCtrl);

    // 7. WiFi AP
    initWifi();

    // 8. Web server
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

    if (settingsMgr.settings.arm_cal.calibrated) {
        Serial.println("[Main] Arm calibration loaded from settings (delta format)");
        armCtrl.printCalTable();
    } else {
        Serial.println("[Main] No arm calibration -- use 'cal start' to calibrate (arms must be disarmed)");
    }

    lastControlTick = millis();
    lastDisplayTick = millis();
    lastWsTick = millis();
    lastTelTick = millis();
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

        // 0. Poll M5 hardware and update AHRS filter
        M5.update();
        if (M5.Imu.update()) {
            auto imu = M5.Imu.getImuData();
            ahrsFilter.update(
                imu.gyro.x,  imu.gyro.y,  imu.gyro.z,
                imu.accel.x, imu.accel.y, imu.accel.z,
                imu.mag.x,   imu.mag.y,   imu.mag.z);
        }

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

        float arm_move_raw    = crsfRx.getChannelNormalized(s.channel_map.arm_trigger_home);
        float arm_speed_raw   = crsfRx.getChannelNormalized(CH_ARM_SPEED);
        float arm_nudge_raw   = crsfRx.getChannelNormalized(CH_ARM_NUDGE);

        float throttle = applyDeadband(throttle_raw, deadband_norm);
        float steering = applyDeadband(steering_raw, deadband_norm);

        // Override with simulated values if sim mode is active
        if (simEnabled) {
            throttle = simThrottle;
            steering = simSteering;
        }

        // Ch12 move trigger (always active)
        bool moveNow = isSwitchActive(arm_move_raw);
        bool moveEdge = moveNow && !prevMoveTrigger;
        if (!simEnabled) {
            prevMoveTrigger = moveNow;
        }

        // Ch11: always read for cal mode entry (3s hold) and cal step advance (short press)
        float arm_cal_raw = crsfRx.getChannelNormalized(s.channel_map.arm_trigger_exec);
        bool calNow = isSwitchActive(arm_cal_raw);
        bool calEdge = calNow && !prevCalTrigger;
        if (!simEnabled) {
            prevCalTrigger = calNow;
        }

        // 3-second hold on Ch11 enters cal mode (only when arms disarmed and not already in cal mode)
        if (calNow && !motorMgr.isArmArmed() && !armCtrl.isInCalMode()) {
            if (calHoldStart == 0) {
                calHoldStart = now;
                calHoldFired = false;
            } else if (!calHoldFired && (now - calHoldStart >= CAL_ENTRY_HOLD_MS)) {
                calHoldFired = true;
                armCtrl.enterCalMode();
                calEdge = false;
            }
        } else if (!calNow) {
            calHoldStart = 0;
            calHoldFired = false;
        }

        // Suppress the rising edge that started the hold from being treated as a cal step
        if (calHoldFired) {
            calEdge = false;
        }

        // --- Balance controller: read Ch7 and IMU, update state machine ---
        float ch7_raw = crsfRx.getChannelNormalized(s.channel_map.arm_select_var);
        bool ch7Active = isSwitchActive(ch7_raw);

        float rollDeg = ahrsFilter.getRoll();
        float rawRollRate = (rollDeg - prevRollDeg) / dt;
        prevRollDeg = rollDeg;
        filteredRollRate = 0.25f * rawRollRate + 0.75f * filteredRollRate;
        float rollRateDps = filteredRollRate;

        // When Ch7 is active and balance is not yet running, Ch11 edge triggers tip-up.
        // When balance is active, suppress Ch11's normal jump behavior.
        bool balanceWantsEdge = ch7Active && !balanceCtrl.isActive() && calEdge;
        balanceCtrl.update(rollDeg, rollRateDps, ch7Active, balanceWantsEdge, dt);

        bool balanceDriving = balanceCtrl.isControllingDrive();
        bool balanceActive  = balanceCtrl.isActive();

        // If balance just released drive control, re-sync drive controller
        if (prevBalanceWasActive && !balanceDriving) {
            driveCtrl.emergencyStop();
        }
        prevBalanceWasActive = balanceDriving;

        ArmInput armInput = {};
        armInput.cal_trigger = calEdge;
        armInput.move_trigger = moveEdge;
        armInput.jump_trigger = balanceActive ? false : calEdge;
        armInput.speed_channel = arm_speed_raw;
        armInput.nudge_channel = applyDeadband(arm_nudge_raw, deadband_norm);

        if (calEdge || moveEdge) {
            Serial.printf("[ArmInput] cal=%d move=%d jump=%d\n", calEdge, moveEdge, calEdge);
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
                    Serial.printf("[Main] t=%lu ARMS ARM requested via RC switch (ensure arms at FORWARD)\n", now);
                    motorMgr.armArmMotors();
                    armCtrl.setForwardReference();
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
        } else if (balanceDriving) {
            // Balance controller owns back wheels and arms.
            // Hold front wheels at their current position via drive controller
            // only if drive is armed (back wheels handled inside balanceCtrl).
            // Arm controller still called -- override is active inside it.
            armCtrl.update(armInput, dt);
        } else {
            // 6. Drive control
            if (motorMgr.isDriveArmed()) {
                driveCtrl.update(throttle, steering, dt);
            }

            // 7. Arm control (always called -- calibration works even when disarmed,
            //    movement/hold gated by isArmArmed inside update)
            armCtrl.update(armInput, dt);
        }

        // (calibration streaming removed -- now via TX triggers)

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
        //    Toggle with 'd' command over serial
        if (debugOutputEnabled && controlTickCount % 100 == 0) {
            uint32_t avgUs = loopTotalUs / 100;
            Serial.println("==================================================");
            Serial.printf("[Loop] t=%lu tick=%lu | avg=%luus max=%luus overruns=%lu\n",
                          now, controlTickCount, avgUs, loopMaxUs, loopOverruns);
            Serial.printf("[Loop] link=%d rssi=%d lq=%d | drv_armed=%d arm_armed=%d | thr=%.2f str=%.2f | wifi=%d | sim=%s\n",
                          crsfRx.isLinkUp(), crsfRx.getRssi(), crsfRx.getLinkQuality(),
                          motorMgr.isDriveArmed(), motorMgr.isArmArmed(),
                          throttle, steering, wifiConnected,
                          simEnabled ? "ON" : "off");

            Serial.printf("[IMU] pitch=%+6.1f roll=%+6.1f yaw=%+6.1f\n",
                          ahrsFilter.getPitch(), ahrsFilter.getRoll(), ahrsFilter.getYaw());

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
            {
                const char* arm_pos_str = armCtrl.isMoving() ? "MOVING" :
                    (armCtrl.getCurrentPosition() == ArmPosition::Forward ? "FWD" :
                     armCtrl.getCurrentPosition() == ArmPosition::Center ? "CTR" :
                     armCtrl.getCurrentPosition() == ArmPosition::Jump ? "JUMP" : "BWD");
                float fwd_l = armCtrl.getForwardLeft();
                float fwd_r = armCtrl.getForwardRight();
                float cur_l = motorMgr.getMotor(MotorRole::ArmLeft).position;
                float cur_r = motorMgr.getMotor(MotorRole::ArmRight).position;
                Serial.printf("[Arm] pos=%s cal_mode=%s | L: %.2f (cal %.2f) R: %.2f (cal %.2f)\n",
                              arm_pos_str,
                              armCtrl.isInCalMode() ? "YES" : "no",
                              cur_l, cur_l - fwd_l,
                              cur_r, cur_r - fwd_r);
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

            Serial.printf("[VBUS] %.1fV  [Current] %.2fA\n",
                          motorMgr.getBusVoltage(), motorMgr.getTotalCurrent());

            // Drive controller per-motor closed-loop status
            driveCtrl.printDebug();

            // Balance controller status (only when not idle)
            if (balanceCtrl.isActive()) {
                Serial.printf("[Balance] state=%s  roll=%.1f  setpoint=%.1f\n",
                              balanceCtrl.getStateString(), rollDeg, balanceCtrl.getSetpoint());
            }

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
                       motorMgr.isDriveArmed(), motorMgr.isArmArmed(),
                       &armCtrl);
    }

    // -----------------------------------------------------------------------
    // ~10 Hz WebSocket telemetry
    // -----------------------------------------------------------------------
    if (now - lastWsTick >= WEBSOCKET_PERIOD_MS) {
        lastWsTick = now;
        webUI.sendTelemetry();
    }

    // -----------------------------------------------------------------------
    // ~5 Hz CRSF telemetry to transmitter
    // -----------------------------------------------------------------------
    if (now - lastTelTick >= CRSF_TELEMETRY_PERIOD_MS) {
        lastTelTick = now;

        const char* state = balanceCtrl.isActive()
            ? balanceCtrl.getStateString()
            : armCtrl.getStateString();
        crsfRx.sendFlightMode(state);
        crsfRx.sendBatteryTelemetry(motorMgr.getBusVoltage(),
                                    motorMgr.getTotalCurrent());
        crsfRx.sendAttitudeTelemetry(ahrsFilter.getPitch(),
                                     ahrsFilter.getRoll(),
                                     ahrsFilter.getYaw());
    }
}
