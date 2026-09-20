#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>

#include "config.h"
#include "settings.h"
#include "robstride.h"
#include "motor_manager.h"
#include "crsf.h"
#include "radio_status.h"
#include "drive_controller.h"
#include "ground_drive_gate.h"
#include "arm_controller.h"
#include "balance_controller.h"
#include "balance_math.h"
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
static GroundDriveGate  groundDriveGate;
static ArmController    armCtrl;
static BalanceController balanceCtrl;
static Display          display;
static WebUI            webUI;
static Madgwick         ahrsFilter;

// Timing
static uint32_t lastControlTick  = 0;
static uint32_t lastDisplayTick  = 0;
static uint32_t lastTelTick      = 0;
// (lastCalPrintTick removed -- calibration now via TX triggers)
static uint32_t controlTickCount = 0;

// Loop timing diagnostics
static uint32_t loopMaxUs = 0;
static uint32_t loopTotalUs = 0;
static uint32_t loopOverruns = 0;

// Section profiler: worst-case duration per subsystem plus the largest gap
// between control-task iterations. A big ctl gap with small section maxima
// means an external task starved the control core; a big section max names
// the offender directly. Dumped as # prof_* lines with `bal log`.
enum ProfSection { PROF_IMU = 0, PROF_CTL, PROF_DISP, PROF_WS, PROF_TEL,
                   PROF_CRSF, PROF_CAN, PROF_BAL, PROF_ADRV, PROF_COUNT };
static const char* profNames[PROF_COUNT] = { "imu", "ctl", "disp", "ws", "tel",
                                             "crsf", "can", "bal", "adrv" };
static uint32_t profMaxUs[PROF_COUNT] = {};
static uint32_t profLoopGapMaxUs = 0;    // loopTask (display/WS) gap -- low prio, gaps expected
static uint32_t profLastLoopUs = 0;
static uint32_t profCtlGapMaxUs = 0;     // control-task gap -- THE stall metric
static uint32_t ctlLastTickUs = 0;

static inline void profRecord(ProfSection s, uint32_t start_us) {
    uint32_t dur = micros() - start_us;
    if (dur > profMaxUs[s]) profMaxUs[s] = dur;
}

// ---------------------------------------------------------------------------
// Stall forensics
//
// Every control-task gap >100ms is recorded as an event with:
//  - which profiler section grew since the last event (our own blocking code
//    names itself; 'none' = the whole task was preempted between sections)
//  - the sentinel gap: a tiny priority-24 task on the control core stamps a
//    timestamp every 10ms. If the sentinel ALSO gapped for the stall
//    duration, the whole core was dark (flash-cache stall during a LittleFS
//    write, or interrupts disabled); if the sentinel kept running, the
//    control task was blocked in its own code or preempted by a task with
//    priority between the control task and 24.
// Dumped as # stall_* lines with `bal log`.
// ---------------------------------------------------------------------------
static constexpr uint32_t STALL_EVENT_MIN_US = 100000;   // 100 ms

struct StallEvent {
    uint32_t t_ms;
    uint32_t gap_us;
    uint32_t sentinel_gap_us;
    int8_t   section;        // ProfSection that grew, or -1
    uint8_t  bal_state;
};
static constexpr int STALL_RING_LEN = 8;
static StallEvent stallRing[STALL_RING_LEN];
static uint32_t   stallCount = 0;
static uint32_t   profMaxAtLastEvent[PROF_COUNT] = {};
static volatile uint32_t sentinelGapMaxUs = 0;
static bool profileRunActive = false;

struct LoopProfileSnapshot {
    bool valid;
    uint32_t run_start_ms;
    uint32_t max_us[PROF_COUNT];
    uint32_t loop_gap_max_us;
    uint32_t ctl_gap_max_us;
    uint32_t stall_count;
    StallEvent stalls[STALL_RING_LEN];
};
static LoopProfileSnapshot frozenProfile = {};

// NOTE: naming the preempting task directly via uxTaskGetSystemState runtime
// deltas is not possible on this framework build (espressif32@6.7.0 ships
// with CONFIG_FREERTOS_USE_TRACE_FACILITY / GENERATE_RUN_TIME_STATS off).
// The sentinel-gap discriminator plus section attribution covers the same
// diagnostic question: own code vs preemption vs whole-core blackout.

static void recordStallEvent(uint32_t now_ms, uint32_t gap_us, uint8_t bal_state) {
    // Which section grew the most since the last event?
    int8_t section = -1;
    uint32_t best_delta = 0;
    for (int i = 0; i < PROF_COUNT; i++) {
        uint32_t delta = profMaxUs[i] - profMaxAtLastEvent[i];
        if (profMaxUs[i] >= profMaxAtLastEvent[i] && delta > best_delta) {
            best_delta = delta;
            section = (int8_t)i;
        }
        profMaxAtLastEvent[i] = profMaxUs[i];
    }
    if (best_delta < gap_us / 2) section = -1;  // stall not inside our code

    StallEvent& e = stallRing[stallCount % STALL_RING_LEN];
    e.t_ms = now_ms;
    e.gap_us = gap_us;
    e.sentinel_gap_us = sentinelGapMaxUs;
    e.section = section;
    e.bal_state = bal_state;
    stallCount++;
    sentinelGapMaxUs = 0;
}

static void printLoopProfile() {
    const uint32_t* max_us = frozenProfile.valid ? frozenProfile.max_us : profMaxUs;
    uint32_t loop_gap = frozenProfile.valid ? frozenProfile.loop_gap_max_us : profLoopGapMaxUs;
    uint32_t ctl_gap = frozenProfile.valid ? frozenProfile.ctl_gap_max_us : profCtlGapMaxUs;
    uint32_t count = frozenProfile.valid ? frozenProfile.stall_count : stallCount;
    const StallEvent* ring = frozenProfile.valid ? frozenProfile.stalls : stallRing;
    uint32_t run_start_ms = frozenProfile.valid ? frozenProfile.run_start_ms : 0;

    Serial.printf("# profile_scope=%s\n", frozenProfile.valid ? "balance_run" : "live");
    Serial.printf("# profile_run_start_uptime_ms=%lu\n", (unsigned long)run_start_ms);
    for (int i = 0; i < PROF_COUNT; i++) {
        Serial.printf("# prof_%s_max_us=%lu\n", profNames[i], (unsigned long)max_us[i]);
    }
    Serial.printf("# prof_loopgap_max_us=%lu\n", (unsigned long)loop_gap);
    Serial.printf("# prof_ctlgap_max_us=%lu\n", (unsigned long)ctl_gap);
    Serial.printf("# stall_events=%lu\n", (unsigned long)count);
    uint32_t shown = (count < STALL_RING_LEN) ? count : STALL_RING_LEN;
    uint32_t first = (count > STALL_RING_LEN) ? count - STALL_RING_LEN : 0;
    for (uint32_t i = 0; i < shown; i++) {
        const StallEvent& e = ring[(first + i) % STALL_RING_LEN];
        const char* sec = "none";
        if (e.section >= 0 && e.section < PROF_COUNT) sec = profNames[e.section];
        uint32_t run_t_ms = run_start_ms ? e.t_ms - run_start_ms : e.t_ms;
        Serial.printf("# stall_%lu=t:%lu,uptime_ms:%lu,gap_us:%lu,sentinel_us:%lu,sec:%s,state:%u\n",
                      (unsigned long)i, (unsigned long)run_t_ms,
                      (unsigned long)e.t_ms,
                      (unsigned long)e.gap_us, (unsigned long)e.sentinel_gap_us,
                      sec, e.bal_state);
    }
}

static void resetLoopProfile(uint32_t run_start_ms = 0) {
    for (int i = 0; i < PROF_COUNT; i++) {
        profMaxUs[i] = 0;
        profMaxAtLastEvent[i] = 0;
    }
    profLoopGapMaxUs = 0;
    profCtlGapMaxUs = 0;
    stallCount = 0;
    sentinelGapMaxUs = 0;
    ctlLastTickUs = micros();
    profLastLoopUs = ctlLastTickUs;
    profileRunActive = run_start_ms != 0;
    if (profileRunActive) frozenProfile.valid = false;
}

static void freezeLoopProfile() {
    if (!profileRunActive) return;
    frozenProfile.valid = true;
    frozenProfile.run_start_ms = balanceCtrl.getLogStartMs();
    for (int i = 0; i < PROF_COUNT; i++) frozenProfile.max_us[i] = profMaxUs[i];
    frozenProfile.loop_gap_max_us = profLoopGapMaxUs;
    frozenProfile.ctl_gap_max_us = profCtlGapMaxUs;
    frozenProfile.stall_count = stallCount;
    for (int i = 0; i < STALL_RING_LEN; i++) frozenProfile.stalls[i] = stallRing[i];
    profileRunActive = false;
}

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
static bool  prevBalanceTelemetryActive = false;

// Double-tap detection for force-engage
static uint32_t lastCalEdgeTime = 0;
static bool     waitingForDoubleTap = false;

// Fast task publishes the latest IMU sample for display attitude
static RawImuData sharedImu = {};
static volatile bool       sharedImuReady = false;
static portMUX_TYPE        imuMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool logDownloadActive = false;
static std::atomic<bool> webDisarmPending{false};
static bool rcRearmRequired = true; // Boot / OTA never arms from switches already high.
static LoopTiming networkControlTiming;
static LoopTiming networkBalanceTiming;
static QueueHandle_t networkBalanceStats = nullptr;
static void publishNetworkSnapshot(uint32_t now);

// Task handles (control core split)
static TaskHandle_t balanceTaskHandle  = nullptr;
static TaskHandle_t controlTaskHandle  = nullptr;
static TaskHandle_t sentinelTaskHandle = nullptr;
static void controlTaskFunc(void* param);
static void sentinelTaskFunc(void* param);

// Snapshots for the debug burst (written by control task, read by loopTask)
static volatile float dbgThrottle = 0.0f;
static volatile float dbgSteering = 0.0f;
static uint32_t lastDebugTick = 0;
static uint32_t lastDebugTickCount = 0;

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
    Serial.println("  bal status                  Print balance state and gains");
    Serial.println("  bal kp/kd <val>             Set inner PD gains");
    Serial.println("  bal dkp <val>               Set drift -> target velocity gain");
    Serial.println("  bal vkp/vki <val>           Set velocity PI gains (setpoint offset)");
    Serial.println("  bal log                     Dump telemetry log to serial");
    Serial.println("  bal log clear               Delete telemetry log file");
    Serial.println("  bal note <text>             Tag the next/current test run");
    Serial.println("  bal mark                    Add a numbered event marker");
    Serial.println("  net status                  WiFi address and maintenance state");
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

    } else if (strcmp(sub, "engage") == 0) {
        balanceCtrl.forceEngage();

    } else if (strcmp(sub, "log") == 0) {
        if (balanceCtrl.isActive()) {
            Serial.println("[Balance] Log dump REFUSED while balance mode is active");
            return;
        }
        if (motorMgr.isDriveArmed() || motorMgr.isArmArmed() || motorMgr.isArming()) {
            Serial.println("[Balance] Log dump REFUSED: disarm drive AND arms first");
            return;
        }
        logDownloadActive = true;
        printLoopProfile();
        balanceCtrl.dumpLog();
        logDownloadActive = false;

    } else if (strcmp(sub, "log clear") == 0) {
        balanceCtrl.clearLog();
        if (!balanceCtrl.isActive()) {
            frozenProfile = {};
            resetLoopProfile();
        }

    } else if (strncmp(sub, "note ", 5) == 0) {
        balanceCtrl.setLogNote(sub + 5);

    } else if (strcmp(sub, "note") == 0) {
        balanceCtrl.setLogNote("");

    } else if (strcmp(sub, "mark") == 0) {
        balanceCtrl.markEvent();
        Serial.println("[Balance] Event marker added");

    } else if (strncmp(sub, "kp ", 3) == 0) {
        float val = atof(sub + 3);
        balanceCtrl.setKp(val);
        Serial.printf("[Balance] Kp = %.4f\n", val);

    } else if (strncmp(sub, "kd ", 3) == 0) {
        float val = atof(sub + 3);
        balanceCtrl.setKd(val);
        Serial.printf("[Balance] Kd = %.4f\n", val);

    } else if (strncmp(sub, "dkp ", 4) == 0) {
        float val = atof(sub + 4);
        balanceCtrl.setDriftVelKp(val);
        Serial.printf("[Balance] Drift vel Kp = %.4f\n", val);

    } else if (strncmp(sub, "vkp ", 4) == 0) {
        float val = atof(sub + 4);
        balanceCtrl.setVelSpKp(val);
        Serial.printf("[Balance] Vel-sp Kp = %.4f\n", val);

    } else if (strncmp(sub, "vki ", 4) == 0) {
        float val = atof(sub + 4);
        balanceCtrl.setVelSpKi(val);
        Serial.printf("[Balance] Vel-sp Ki = %.4f\n", val);

    } else if (strncmp(sub, "trim ", 5) == 0) {
        if (balanceCtrl.isActive()) {
            Serial.println("[Balance] Stored trim write REFUSED while balance mode is active");
            return;
        }
        float val = atof(sub + 5);
        settingsMgr.settings.balance_trim = val;
        settingsMgr.save();
        Serial.printf("[Balance] Stored equilibrium trim = %.2f deg (saved)\n", val);

    } else if (strcmp(sub, "trim") == 0) {
        Serial.printf("[Balance] Stored equilibrium trim = %.2f deg\n",
                      settingsMgr.settings.balance_trim);

    } else {
        Serial.println("[Balance] Usage: bal status | bal engage");
        Serial.println("         bal kp/kd <val>   inner PD gains");
        Serial.println("         bal dkp <val>     drift -> target vel gain");
        Serial.println("         bal vkp/vki <val> velocity PI -> setpoint offset");
        Serial.println("         bal trim [<val>]  show/set stored equilibrium trim");
        Serial.println("         bal note <text> | bal mark");
        Serial.println("         bal log | bal log clear");
    }
}

static void processSerialCommand(const char* cmd) {
    // Skip leading whitespace
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;

    if (webUI.maintenance.busy() && strcmp(cmd, "disarm") && strcmp(cmd, "disarm arms")) {
        Serial.println("[Cmd] REFUSED during network maintenance");
        return;
    }
    if (strcmp(cmd, "net status") == 0) {
        Serial.printf("[Network] connected=%d rearm_required=%d maintenance=%d; address on display / mDNS\n",
                      webUI.connected(), rcRearmRequired, webUI.maintenance.busy());
        return;
    }
    // A capture snapshots its gains once. Keep the control task free of
    // bench diagnostics, flash writes and tuning changes during that run.
    if (balanceCtrl.isActive()
        && strcmp(cmd, "disarm") && strcmp(cmd, "disarm arms")
        && strcmp(cmd, "bal status") && strcmp(cmd, "bal mark")
        && strncmp(cmd, "bal note", 8)) {
        Serial.println("[Cmd] REFUSED during balance: disarm, bal status/note/mark only");
        return;
    }

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
        if (rcRearmRequired) { Serial.println("[Cmd] Lower both RC arm switches first"); return; }
        Serial.println("[Sim] Arming drive motors...");
        motorMgr.requestArmDrive();

    } else if (strcmp(cmd, "disarm") == 0) {
        Serial.println("[Sim] Disarming drive motors...");
        if (balanceCtrl.isActive()) balanceCtrl.hardAbort("serial disarm");
        rcRearmRequired = true;
        motorMgr.cancelArming();
        driveCtrl.emergencyStop();
        motorMgr.disarmDriveMotors();

    } else if (strcmp(cmd, "arm arms") == 0) {
        if (rcRearmRequired) { Serial.println("[Cmd] Lower both RC arm switches first"); return; }
        Serial.println("[Sim] Arming arm motors (ensure arms are in FORWARD position)...");
        motorMgr.requestArmArms();

    } else if (strcmp(cmd, "disarm arms") == 0) {
        Serial.println("[Sim] Disarming arm motors...");
        if (balanceCtrl.isActive()) balanceCtrl.hardAbort("serial arms disarm");
        rcRearmRequired = true;
        motorMgr.cancelArming();
        armCtrl.holdPosition();
        motorMgr.disarmArmMotors();

    } else if (strcmp(cmd, "status") == 0) {
        uint32_t now = millis();
        Serial.println("==================================================");
        Serial.printf("[Status] t=%lu sim=%s thr=%.2f str=%.2f\n",
                      now, simEnabled ? "ON" : "OFF", simThrottle, simSteering);
        Serial.printf("[Status] link=%d drv_armed=%d arm_armed=%d\n",
                      crsfRx.isLinkUp(), motorMgr.isDriveArmed(), motorMgr.isArmArmed());
        Serial.printf("[CRSF] RX max_us=%lu bytes=%lu budget_yields=%lu age_ms=%lu\n",
                      crsfRx.maxUpdateUs(), crsfRx.receivedBytes(),
                      crsfRx.budgetYields(), crsfRx.timeSinceLastFrame());
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
    // Bounded input, one command per tick; discard overlong commands whole.
    static bool overflow = false;
    unsigned budget = 128;
    while (budget-- && Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (overflow) {
                overflow = false;
                serialBufLen = 0;
                Serial.println("[Cmd] Overlong command discarded");
                return;
            }
            if (serialBufLen > 0) {
                serialBuf[serialBufLen] = '\0';
                processSerialCommand(serialBuf);
                serialBufLen = 0;
                return;
            }
        } else if (!overflow && serialBufLen < (int)sizeof(serialBuf) - 1) {
            serialBuf[serialBufLen++] = c;
        } else {
            overflow = true;
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
static void onDisarmRequested() {
    // Async TCP runs on the other core: enqueue intent, never mutate motor
    // modes or balance state concurrently with the control task.
    webDisarmPending = true;
}

static void serviceWebDisarm() {
    if (!webDisarmPending.exchange(false)) return;
    rcRearmRequired = true;
    Serial.println("[Main] Emergency disarm requested via web");
    if (balanceCtrl.isActive()) {
        balanceCtrl.hardAbort("web disarm");
    }
    motorMgr.cancelArming();
    driveCtrl.emergencyStop();
    armCtrl.holdPosition();
    motorMgr.disarmAll();
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
// Balance task (control core, 200Hz, direct IMU acquisition)
// ---------------------------------------------------------------------------
static void balanceTaskFunc(void* param) {
    (void)param;
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t prevUs = micros();

    Serial.printf("[Balance] 200Hz PD task started on core %d\n", xPortGetCoreID());

    for (;;) {
        uint32_t nowUs = micros();
        float dt = (float)(nowUs - prevUs) / 1000000.0f;
        prevUs = nowUs;
        // Flash maintenance intentionally pauses both cores. Exclude it and
        // initial boot from live-control measurements, resetting on exit.
        static bool wasMaintenance = false;
        const bool maintenance = webUI.maintenance.busy();
        if (wasMaintenance && !maintenance) networkBalanceTiming = {};
        else if (!maintenance && millis() > 5000) networkBalanceTiming.record(static_cast<uint32_t>(dt * 1000000.0f));
        wasMaintenance = maintenance;
        if (networkBalanceStats) xQueueOverwrite(networkBalanceStats, &networkBalanceTiming);
        if (dt <= 0.0f) dt = 0.005f;

        // This task owns sensor acquisition as well as PD. A control-task
        // stall can freeze the outer reference but cannot freeze the IMU.
        static RawImuData sample = {};
        const uint32_t imu_start_us = micros();
        const auto mask = M5.Imu.update();
        constexpr unsigned required = m5::IMU_Class::sensor_mask_accel
                                    | m5::IMU_Class::sensor_mask_gyro;
        if ((static_cast<unsigned>(mask) & required) == required) {
            const auto imu = M5.Imu.getImuData();
            sample.accel_x = imu.accel.x; sample.accel_y = imu.accel.y; sample.accel_z = imu.accel.z;
            sample.gyro_x = imu.gyro.x; sample.gyro_y = imu.gyro.y; sample.gyro_z = imu.gyro.z;
            sample.sample_us = micros();
            sample.valid = balance_math::finiteImu(sample.accel_x, sample.accel_y, sample.accel_z,
                                                   sample.gyro_x, sample.gyro_y, sample.gyro_z);
            portENTER_CRITICAL(&imuMux);
            sharedImu = sample;
            sharedImuReady = sample.valid;
            portEXIT_CRITICAL(&imuMux);
        }
        profRecord(PROF_IMU, imu_start_us);
        // Call even on read failure: the freshness watchdog must still run.
        balanceCtrl.balanceTick(sample, dt);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / BALANCE_LOOP_HZ));
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    ahrsFilter.begin(CONTROL_LOOP_HZ);

    Serial.begin(115200);
    // Arduino ESP32 2.0.16 HWCDC decrements its unsigned retry counter
    // before testing zero. A zero timeout can underflow when the host
    // stops consuming data. One millisecond avoids that path and bounds
    // a stalled write; never restore the default 100ms live timeout.
    Serial.setTxTimeoutMs(1);
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
    balanceCtrl.setSettingsManager(&settingsMgr);

    // Networking owns all HTTP/JSON/OTA work on core 0. Credentials are local
    // build configuration; no settings migration or filesystem image upload.
    networkBalanceStats = xQueueCreate(1, sizeof(LoopTiming));
    webUI.begin([](Print* out) { balanceCtrl.dumpLog(out); }, onDisarmRequested);

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
    lastTelTick = millis();
    lastDebugTick = millis();

    // Control-core / comms-core split: everything control-critical runs on
    // core CONTROL_CORE (1), above loopTask (priority 1) and out of reach of
    // the networking stack (WiFi/lwip/async_tcp all on core 0). The 200 Hz
    // PD outranks the 50 Hz control tick; the forensics sentinel outranks
    // both so it can witness whole-core stalls.
    xTaskCreatePinnedToCore(
        balanceTaskFunc,
        "BalanceTask",
        4096,
        nullptr,
        BALANCE_TASK_PRIORITY,
        &balanceTaskHandle,
        CONTROL_CORE
    );
    xTaskCreatePinnedToCore(
        controlTaskFunc,
        "ControlTask",
        8192,
        nullptr,
        CONTROL_TASK_PRIORITY,
        &controlTaskHandle,
        CONTROL_CORE
    );
    xTaskCreatePinnedToCore(
        sentinelTaskFunc,
        "StallSentinel",
        2048,
        nullptr,
        SENTINEL_TASK_PRIORITY,
        &sentinelTaskHandle,
        CONTROL_CORE
    );
    Serial.printf("[Main] Control core split: balance %dHz prio %d, control prio %d, sentinel prio %d on core %d\n",
                  BALANCE_LOOP_HZ, BALANCE_TASK_PRIORITY, CONTROL_TASK_PRIORITY,
                  SENTINEL_TASK_PRIORITY, CONTROL_CORE);
}

// ---------------------------------------------------------------------------
// Control tick -- runs on the dedicated control task (core 1, priority 12,
// 200 Hz cadence). Everything control-critical lives here: serial commands
// (CAN access stays single-threaded), IMU, CRSF RX/TX, CAN scan/feedback,
// arming, balance state machine, drive and arm control.
// ---------------------------------------------------------------------------
// Read-only radio snapshot: all motor/state-machine reads stay on their owner
// task. Fast-loop scalar reads are diagnostic, not a coherent control sample.
static radio_status::Snapshot radioSnapshot(uint32_t now, uint16_t sequence) {
    using namespace radio_status;
    Snapshot s;
    s.sequence = sequence;
    const bool drive = motorMgr.isDriveArmed(), arms = motorMgr.isArmArmed();
    if (drive) s.flags |= DriveArmed;
    if (arms) s.flags |= ArmsArmed;
    if (motorMgr.isArmingDrive()) s.flags |= DriveArming;
    if (motorMgr.isArmingArms()) s.flags |= ArmsArming;
    if (crsfRx.isLinkUp()) s.flags |= RcLinked;
    if (armCtrl.isMoving()) s.flags |= ArmsMoving;
    if (rcRearmRequired) s.flags |= RearmRequired;
    if (balanceCtrl.isLogPendingFlush()) s.flags |= SavingLog;
    if (simEnabled || testModeActive) s.flags |= Simulation;
    const uint32_t sample = balanceCtrl.getImuSampleUs();
    if (sample && uint32_t(micros() - sample) <= BALANCE_IMU_STALE_US) {
        s.flags |= ImuFresh;
        s.tilt = angle(balanceCtrl.getTiltAngle());
        if (balanceCtrl.isActive())
            s.error = angle(balanceCtrl.getTiltAngle() - balanceCtrl.getEffectiveSetpoint());
    }
    if (motorMgr.busVoltageFresh(now)) s.flags |= VoltageFresh;
    if (motorMgr.motorCurrentFresh(now)) s.flags |= CurrentFresh;
    s.voltage = unsignedScaled(motorMgr.getBusVoltage(), 10);
    s.motor_current = unsignedScaled(motorMgr.getTotalCurrent(), 10);
    s.inner_fault = balanceCtrl.getInnerFault();
    for (int i = 0; i < NUM_MOTORS; ++i) {
        const auto& m = motorMgr.getMotor(i);
        if (m.has_fault) s.faults |= 1u << i;
        if (!m.online || !m.has_first_feedback || uint32_t(now - m.last_feedback_ms) > 500) continue;
        s.online |= 1u << i;
        if (m.enabled) s.enabled |= 1u << i;
        if (std::isfinite(m.temperature) && m.temperature >= 0) {
            const auto t = static_cast<uint8_t>(fminf(m.temperature, 254));
            if (s.max_temp == 255 || t > s.max_temp) s.max_temp = t;
        }
    }
    s.mode = (drive || arms) ? 1 : 0;
    s.label = drive ? "WHEELS READY" : (arms ? "ARMS READY" : "DISARMED");
    if (drive) {
        for (int i = 0; i < NUM_DRIVE_MOTORS; ++i) {
            const auto state = driveCtrl.getMotorState(static_cast<MotorRole>(i));
            if (state == DriveMotorState::Braking) s.label = "BRAKING";
            if (state == DriveMotorState::Driving) { s.label = "DRIVING"; break; }
        }
    }
    if (arms) {
        s.motion = 1 + static_cast<uint8_t>(armCtrl.getCurrentPosition());
        if (armCtrl.isMoving()) { s.label = armCtrl.getStateString(); s.phase = 1; }
    }
    if (balanceCtrl.isActive()) {
        s.mode = 2; s.motion = 0x0100;
        s.phase = 2 + static_cast<uint8_t>(balanceCtrl.getState());
        s.label = balanceCtrl.getStateString();
    } else if (armCtrl.isInCalMode()) {
        s.mode = 3; s.phase = 6;
        s.calibration = static_cast<uint8_t>(armCtrl.getCalStep());
        s.label = armCtrl.getStateString();
    }
    if (motorMgr.isArming()) { s.label = "ARMING"; s.phase = 2; }
    return s;
}

static bool maintenanceAllowed() {
    bool anyEnabled = false;
    for (int i=0; i<NUM_MOTORS; ++i) anyEnabled |= motorMgr.getMotor(i).enabled;
    return MaintenanceConditions{balanceCtrl.isActive(), balanceCtrl.isLogPendingFlush(),
        motorMgr.isDriveArmed(), motorMgr.isArmArmed(), motorMgr.isArming(),
        armCtrl.isInCalMode(), testModeActive, simEnabled, logDownloadActive, anyEnabled}.allowed();
}

static void publishNetworkSnapshot(uint32_t now) {
    NetworkSnapshot s;
    static uint32_t sequence = 0;
    s.sequence = ++sequence; s.uptime_ms = now;
    s.drive_armed = motorMgr.isDriveArmed(); s.arm_armed = motorMgr.isArmArmed();
    s.arming = motorMgr.isArming(); s.link_up = crsfRx.isLinkUp();
    s.rssi = crsfRx.getRssi(); s.lq = crsfRx.getLinkQuality();
    s.rc_age_ms = crsfRx.timeSinceLastFrame();
    s.rearm_required = rcRearmRequired; s.balance_active = balanceCtrl.isActive();
    s.saving_log = balanceCtrl.isLogPendingFlush(); s.maintenance = webUI.maintenance.busy();
    s.safe = maintenanceAllowed();
    s.calibration = armCtrl.isInCalMode(); s.test = testModeActive; s.simulation = simEnabled;
    s.tilt = balanceCtrl.getTiltAngle(); s.rate = balanceCtrl.getGyroRate();
    s.setpoint = balanceCtrl.getEffectiveSetpoint(); s.inner_fault = balanceCtrl.getInnerFault();
    s.imu_age_us = micros() - balanceCtrl.getImuSampleUs();
    strlcpy(s.balance_state, balanceCtrl.getStateString(), sizeof(s.balance_state));
    strlcpy(s.end_reason, balanceCtrl.getLogEndReason(), sizeof(s.end_reason));
    strlcpy(s.start_status, balanceCtrl.getStartStatus(), sizeof(s.start_status));
    s.voltage = motorMgr.getBusVoltage(); s.current = motorMgr.getTotalCurrent();
    s.voltage_fresh = motorMgr.busVoltageFresh(now); s.current_fresh = motorMgr.motorCurrentFresh(now);
    for (int i=0; i<16; ++i) s.channels[i] = crsfRx.getChannel(i);
    for (int i=0; i<NUM_MOTORS; ++i) {
        const auto& m = motorMgr.getMotor(i); auto& out = s.motors[i];
        out.id = m.can_id; out.pos = m.position; out.vel = m.velocity;
        out.torque = m.torque; out.temp = m.temperature; out.error = m.errors;
        out.age_ms = m.has_first_feedback ? now - m.last_feedback_ms : UINT32_MAX;
        out.online = m.online && m.has_first_feedback && out.age_ms <= 500;
        out.enabled = m.enabled;
    }
    if (networkBalanceStats) xQueuePeek(networkBalanceStats, &s.balance_timing, 0);
    s.control_timing = networkControlTiming;
    webUI.publish(s);
}

static void controlTick() {
    uint32_t now = millis();
    uint32_t nowUs = micros();

    // Control gap: time since the previous control iteration. THE stall
    // metric -- >100 ms gaps are recorded as forensic events.
    if (ctlLastTickUs != 0) {
        uint32_t gap = nowUs - ctlLastTickUs;
        if (gap > profCtlGapMaxUs) profCtlGapMaxUs = gap;
        if (gap > STALL_EVENT_MIN_US && profileRunActive) {
            recordStallEvent(now, gap, (uint8_t)balanceCtrl.getState());
        }
    }
    ctlLastTickUs = nowUs;

    // All safety predicates are read on their owning control task. Once
    // granted, arming, serial mutation, calibration and balance triggers stop.
    if (webUI.maintenance.pending()) {
        const bool safe = maintenanceAllowed();
        webUI.maintenance.service(safe, [](bool ota) {
            // Complete the interlock before granting flash access on core 0.
            rcRearmRequired = true;
            waitingForDoubleTap = false;
            prevCalTrigger = prevMoveTrigger = true;
            calHoldStart = 0;
            calHoldFired = false;
            if (ota) {
                crsfRx.suspend();
                driveCtrl.emergencyStop();
                armCtrl.clearOverride();
                armCtrl.holdPosition();
                motorMgr.disarmAll();
                dbgThrottle = dbgSteering = 0.0f;
            }
        });
    }
    if (crsfRx.suspended() && !webUI.maintenance.ota()) {
        // Aborted/rejected OTA: old frames and partial packets are gone. Arm
        // switches must be observed LOW in new frames before any later arm.
        crsfRx.resume();
        rcRearmRequired = true;
        prevCalTrigger = prevMoveTrigger = true;
        calHoldStart = 0;
        calHoldFired = false;
    }
    if (webUI.maintenance.granted()) {
        rcRearmRequired = true;
        waitingForDoubleTap = false;
        prevCalTrigger = true;
        serviceWebDisarm();
        pollSerialCommands(); // Discard prohibited commands instead of deferring them.
        if (!webUI.maintenance.ota()) crsfRx.update();
        motorMgr.processFeedback();
        publishNetworkSnapshot(now);
        return;
    }
    // Process serial debug commands (non-blocking)
    serviceWebDisarm();
    pollSerialCommands();

    if (!testModeActive) {
        const uint32_t prof_can = micros();
        motorMgr.processFeedback();
        profRecord(PROF_CAN, prof_can);
    }

    // -----------------------------------------------------------------------
    // 50 Hz control loop
    // -----------------------------------------------------------------------
    if (now - lastControlTick >= CONTROL_LOOP_PERIOD_MS) {
        uint32_t tickStartUs = micros();
        lastControlTick = now;
        controlTickCount++;
        float dt = static_cast<float>(CONTROL_LOOP_PERIOD_MS) / 1000.0f;

        // Display attitude runs at its configured 50Hz; only the fast task
        // accesses the IMU hardware. The balance controller uses its own filter.
        RawImuData displayImu;
        portENTER_CRITICAL(&imuMux);
        displayImu = sharedImu;
        bool ready = sharedImuReady;
        portEXIT_CRITICAL(&imuMux);
        if (ready) ahrsFilter.updateIMU(displayImu.gyro_x, displayImu.gyro_y, displayImu.gyro_z,
                                       displayImu.accel_x, displayImu.accel_y, displayImu.accel_z);
        M5.update();

        // 1. Read CRSF data
        uint32_t prof_crsf = micros();
        crsfRx.update();
        profRecord(PROF_CRSF, prof_crsf);

        // 2. Scan for motors and process feedback (skip during test mode
        //    so the test's readParamSync can use the CAN bus exclusively)
        if (!testModeActive) {
            uint32_t prof_can = micros();
            canBus.maintainBus();
            motorMgr.scanNextMotor();
            motorMgr.checkTimeouts(500);
            motorMgr.updateArming();

            if (motorMgr.armingJustCompletedArms() && motorMgr.isArmArmed()) {
                armCtrl.setForwardReference();
            }
            motorMgr.clearArmingCompleted();
            profRecord(PROF_CAN, prof_can);
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
        dbgThrottle = throttle;
        dbgSteering = steering;

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

        // --- Balance controller: state machine at 50Hz (PD runs at 200Hz on Core 0) ---
        float ch7_raw = crsfRx.getChannelNormalized(s.channel_map.arm_select_var);
        bool ch7Active = isSwitchActive(ch7_raw);

        // Use complementary filter output from the fast balance task
        float rollDeg = balanceCtrl.getTiltAngle();
        float rollRateDps = balanceCtrl.getGyroRate();

        // Double-tap vs single-tap Ch11 detection while Ch7 high
        bool balanceWantsEdge = false;
        if (ch7Active && !balanceCtrl.isActive()) {
            if (calEdge) {
                if (waitingForDoubleTap && (now - lastCalEdgeTime < 500)) {
                    // Second tap within 500ms: force-engage
                    Serial.println("[Main] Double-tap Ch11 detected -- force engage!");
                    balanceCtrl.forceEngage();
                    waitingForDoubleTap = false;
                    calEdge = false;
                } else {
                    // First tap: wait to see if a second comes
                    waitingForDoubleTap = true;
                    lastCalEdgeTime = now;
                }
            }
            // 500ms passed after single tap without second: fire normal tip-up
            if (waitingForDoubleTap && !calEdge && (now - lastCalEdgeTime > 500)) {
                balanceWantsEdge = true;
                waitingForDoubleTap = false;
                Serial.println("[Main] Single tap Ch11 -- tip-up requested");
            }
        } else {
            waitingForDoubleTap = false;
        }
        // CH12 is a harmless, precisely timestamped test marker while
        // balance owns the arms. Press it immediately before a push/tap.
        if (ch7Active && balanceCtrl.isActive() && moveEdge) {
            balanceCtrl.markEvent();
        }
        uint32_t prof_bal = micros();
        // Explicit CH1/CH2 standing-drive mapping; never accept serial simulation
        // as pilot input. Link freshness is stricter than the global failsafe.
        const bool pilotValid = !simEnabled && crsfRx.isLinkUp()
            && crsfRx.timeSinceLastFrame() <= BALANCE_PILOT_RC_FRESH_MS
            && isSwitchActive(drive_arm_sw) && isSwitchActive(arm_arm_sw);
        // A fresh CH11 edge while balancing requests a supported stand-down.
        // Idle keeps the existing single/double-tap stand-up behavior.
        const bool lowerEdge = calEdge && pilotValid && ch7Active
            && balanceCtrl.getState() == BalanceState::Balancing;
        balanceCtrl.update(rollDeg, rollRateDps, ch7Active, balanceWantsEdge || lowerEdge, dt,
                           crsfRx.getChannelNormalized(DEFAULT_CH_THROTTLE),
                           crsfRx.getChannelNormalized(DEFAULT_CH_STEERING), pilotValid,
                           balance_math::fastTipSelected(crsfRx.getChannelNormalized(CH_FAST_TIP_UP)));
        profRecord(PROF_BAL, prof_bal);

        bool balanceDriving = balanceCtrl.isControllingDrive();
        bool balanceActive  = balanceCtrl.isActive();
        const bool groundDriveAllowed = groundDriveGate.update(
            balanceActive, waitingForDoubleTap || balanceWantsEdge,
            simEnabled || crsfRx.isLinkUp(), throttle, steering);

        // If balance just released drive control, re-sync drive controller
        if (prevBalanceWasActive && !balanceDriving) {
            driveCtrl.emergencyStop();
        }
        prevBalanceWasActive = balanceDriving;

        ArmInput armInput = {};
        armInput.cal_trigger = calEdge;
        armInput.move_trigger = balanceActive ? false : moveEdge;
        armInput.jump_trigger = balanceActive ? false : calEdge;
        armInput.speed_channel = arm_speed_raw;
        armInput.nudge_channel = applyDeadband(arm_nudge_raw, deadband_norm);

        if (!balanceActive && (calEdge || moveEdge)) {
            Serial.printf("[ArmInput] cal=%d move=%d jump=%d\n", calEdge, moveEdge, calEdge);
        }

        // 4. Arm/disarm (only from RC, not sim -- sim uses serial commands)
        //    Level-based arming: while switch is active and not armed, keep
        //    requesting arm (handles partial failures without re-toggle).
        //    Unconditional disarm: always send stop on falling edge regardless
        //    of armed state (fixes partial-arm leaving motors enabled).
        bool driveSwNow = isSwitchActive(drive_arm_sw);
        bool armSwNow   = isSwitchActive(arm_arm_sw);

        if (crsfRx.isLinkUp() && !simEnabled) {
            if (!driveSwNow && !armSwNow) rcRearmRequired = false;
            // Drive: level-based arm
            if (driveSwNow && !rcRearmRequired && !balanceCtrl.isLogPendingFlush()) {
                if (!motorMgr.isDriveArmed() && !motorMgr.isArmingDrive()) {
                    Serial.printf("[Main] t=%lu DRIVE ARM requested via RC switch\n", now);
                    motorMgr.requestArmDrive();
                }
            }
            // Drive: unconditional disarm on falling edge
            if (!driveSwNow && prevDriveArmSwitch) {
                Serial.printf("[Main] t=%lu DRIVE DISARM requested via RC switch\n", now);
                if (balanceCtrl.isActive()) {
                    balanceCtrl.hardAbort("drive disarmed");
                }
                motorMgr.cancelArming();
                driveCtrl.emergencyStop();
                motorMgr.disarmDriveMotors();
            }
            // Drive: LEVEL-based safety guarantee. While the switch is low,
            // any wheel still reporting motion gets stop commands re-sent
            // (covers stop frames lost during CAN bus-off).
            if (!driveSwNow) {
                motorMgr.enforceDriveStopped(now);
            }

            // Arms: level-based arm
            if (armSwNow && !rcRearmRequired && !balanceCtrl.isLogPendingFlush()) {
                if (!motorMgr.isArmArmed() && !motorMgr.isArmingArms()) {
                    Serial.printf("[Main] t=%lu ARMS ARM requested via RC switch (ensure arms at FORWARD)\n", now);
                    motorMgr.requestArmArms();
                }
            }
            // Arms: unconditional disarm on falling edge
            if (!armSwNow && prevArmArmSwitch) {
                Serial.printf("[Main] t=%lu ARMS DISARM requested via RC switch\n", now);
                if (balanceCtrl.isActive()) {
                    balanceCtrl.hardAbort("arms disarmed");
                }
                motorMgr.cancelArming();
                armCtrl.holdPosition();
                motorMgr.disarmArmMotors();
            }
        }

        if (!simEnabled) {
            prevDriveArmSwitch = driveSwNow;
            prevArmArmSwitch = armSwNow;
        }

        // 5. Signal loss failsafe (skip in sim mode -- sim provides its own input)
        if (!simEnabled && !crsfRx.isLinkUp()) {
            if (balanceCtrl.isActive()) {
                balanceCtrl.hardAbort("CRSF link loss");
            }
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
            uint32_t prof_adrv = micros();
            armCtrl.update(armInput, dt);
            profRecord(PROF_ADRV, prof_adrv);
        } else {
            // 6. Drive control
            uint32_t prof_adrv = micros();
            if (motorMgr.isDriveArmed()) {
                // Idle permits ground drive even with CH7 selected. A pending
                // start, active balance/arm return, or held stick at handoff
                // keeps the ground targets stopped.
                driveCtrl.update(groundDriveAllowed ? throttle : 0.0f,
                                 groundDriveAllowed ? steering : 0.0f, dt);
            }

            // 7. Arm control (always called -- calibration works even when disarmed,
            //    movement/hold gated by isArmArmed inside update)
            armCtrl.update(armInput, dt);
            profRecord(PROF_ADRV, prof_adrv);
        }

        // Scope timing forensics to the physical balance run. Freeze the
        // snapshot at the end so an idle `bal log` download cannot pollute it.
        bool telemetryActiveNow = balanceCtrl.isActive();
        if (telemetryActiveNow && !prevBalanceTelemetryActive) {
            resetLoopProfile(balanceCtrl.getLogStartMs());
        } else if (!telemetryActiveNow && prevBalanceTelemetryActive) {
            freezeLoopProfile();
        }
        prevBalanceTelemetryActive = telemetryActiveNow;

        // (calibration streaming removed -- now via TX triggers)

        // 8. Loop timing measurement
        profRecord(PROF_CTL, tickStartUs);
        uint32_t tickElapsedUs = micros() - tickStartUs;
        loopTotalUs += tickElapsedUs;
        if (tickElapsedUs > loopMaxUs) {
            loopMaxUs = tickElapsedUs;
        }
        if (tickElapsedUs > CONTROL_LOOP_PERIOD_MS * 1000) {
            loopOverruns++;
        }

        publishNetworkSnapshot(now);

        // (Periodic debug burst moved to loopTask -- Serial output must never
        // sit on the control path.)
    }

    // -----------------------------------------------------------------------
    // ~5 Hz CRSF telemetry to transmitter (stays with control: it shares the
    // CRSF UART with crsfRx.update())
    // -----------------------------------------------------------------------
    if (now - lastTelTick >= CRSF_TELEMETRY_PERIOD_MS) {
        lastTelTick = now;
        uint32_t prof_start = micros();

        static uint16_t radioSequence = 0;
        static uint8_t radioDetailTick = 0;
        const auto status = radioSnapshot(now, radioSequence++);
        // One custom frame per tick: four status snapshots then one run-detail
        // frame. Fixed stack buffers, no heap, no delays, no retries on full TX.
        if (++radioDetailTick == 5) {
            radioDetailTick = 0;
            uint8_t payload[radio_status::DETAIL_SIZE];
            radio_status::encodeDetail(status.sequence, balanceCtrl.getLogEndMs(),
                                      balanceCtrl.getLogEndReason(), payload);
            crsfRx.sendRobotTelemetry(payload, sizeof(payload));
        } else {
            uint8_t payload[radio_status::STATUS_SIZE];
            radio_status::encode(status, payload);
            crsfRx.sendRobotTelemetry(payload, sizeof(payload));
        }
        crsfRx.sendFlightMode(status.label);
        crsfRx.sendBatteryTelemetry(motorMgr.getBusVoltage(),
                                    motorMgr.getTotalCurrent());
        crsfRx.sendAttitudeTelemetry(ahrsFilter.getPitch(),
                                     ahrsFilter.getRoll(),
                                     ahrsFilter.getYaw());
        profRecord(PROF_TEL, prof_start);
    }
}

// ---------------------------------------------------------------------------
// Control task (core 1, priority 12): the 50 Hz control loop plus the 200 Hz
// IMU read, isolated from display/WebSocket/debug (loopTask, priority 1) and
// from the networking stack (pinned to core 0). Nothing that can run on this
// core below priority 12 can delay a control tick anymore.
// ---------------------------------------------------------------------------
static void controlTaskFunc(void* param) {
    (void)param;
    TickType_t lastWake = xTaskGetTickCount();
    Serial.printf("[Main] Control task started on core %d\n", xPortGetCoreID());
    uint32_t previousUs = micros();
    bool wasMaintenance = false;
    for (;;) {
        const uint32_t nowUs = micros();
        const bool maintenance = webUI.maintenance.busy();
        if (wasMaintenance && !maintenance) networkControlTiming = {};
        else if (!maintenance && millis() > 5000) networkControlTiming.record(nowUs - previousUs);
        previousUs = nowUs; wasMaintenance = maintenance;
        controlTick();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / BALANCE_LOOP_HZ));
    }
}

// ---------------------------------------------------------------------------
// Sentinel task (core 1, priority 24): stall forensics discriminator. If a
// control-task stall coincides with a sentinel gap, the whole core was dark
// (flash-cache stall or interrupts disabled); if the sentinel kept ticking,
// the control task was blocked in its own code or preempted by a
// priority-12..23 task.
// ---------------------------------------------------------------------------
static void sentinelTaskFunc(void* param) {
    (void)param;
    uint32_t lastUs = micros();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint32_t nowUs = micros();
        uint32_t gap = nowUs - lastUs;
        lastUs = nowUs;
        if (gap > sentinelGapMaxUs) sentinelGapMaxUs = gap;
    }
}

// ---------------------------------------------------------------------------
// Main loop (loopTask, core 1, priority 1): everything that is allowed to be
// late. Display and the periodic debug burst. Networking runs on core 0.
// ---------------------------------------------------------------------------
void loop() {
    uint32_t now = millis();
    const auto ota = webUI.otaProgress();
    if (ota.visible(now) && (ota.active() || (!motorMgr.isDriveArmed() && !motorMgr.isArmArmed()))) {
        // OTA owns the whole screen. This never takes the network mutex, which
        // can be held across a flash erase/write; no normal UI or log I/O runs.
        if (now - lastDisplayTick >= 100) {
            lastDisplayTick = now;
            display.renderOta(ota, now);
        }
        delay(5);
        return;
    }
    if (logDownloadActive || webUI.maintenance.busy()) { delay(5); return; }
    uint32_t nowUs = micros();

    // loopTask gap -- expected to grow under control-task preemption; only
    // interesting when it dwarfs the control gap.
    if (profLastLoopUs != 0) {
        uint32_t gap = nowUs - profLastLoopUs;
        if (gap > profLoopGapMaxUs) profLoopGapMaxUs = gap;
    }
    profLastLoopUs = nowUs;

    // Display shares the control core at low priority; reduce it during balance.
    bool balancing = balanceCtrl.isActive();
    if (!balancing && !webUI.maintenance.busy()) balanceCtrl.serviceLog();

    // -----------------------------------------------------------------------
    // ~25 fps display update (5 fps while balancing)
    // -----------------------------------------------------------------------
    uint32_t display_period = balancing ? 200 : DISPLAY_PERIOD_MS;
    if (now - lastDisplayTick >= display_period) {
        lastDisplayTick = now;

        wifiConnected = webUI.connected();
        char networkIp[32]; webUI.copyIp(networkIp, sizeof(networkIp));
        wifiIP = networkIp;

        uint32_t prof_start = micros();
        display.render(motorMgr, crsfRx,
                       wifiConnected, wifiIP.c_str(),
                       motorMgr.isDriveArmed(), motorMgr.isArmArmed(),
                       &armCtrl,
                       motorMgr.isArmingDrive(), motorMgr.isArmingArms());
        profRecord(PROF_DISP, prof_start);
    }

    // -----------------------------------------------------------------------
    // Periodic debug output (every 2 seconds). Toggle with 'd' over serial.
    // Reads control-task state without locks -- diagnostic output only.
    // -----------------------------------------------------------------------
    if (debugOutputEnabled && now - lastDebugTick >= 2000) {
        lastDebugTick = now;
        uint32_t ticks = controlTickCount - lastDebugTickCount;
        lastDebugTickCount = controlTickCount;
        uint32_t avgUs = (ticks > 0) ? (loopTotalUs / ticks) : 0;

        Serial.println("==================================================");
        Serial.printf("[Loop] t=%lu tick=%lu | avg=%luus max=%luus overruns=%lu ctlgap=%luus\n",
                      now, controlTickCount, avgUs, loopMaxUs, loopOverruns,
                      (unsigned long)profCtlGapMaxUs);
        Serial.printf("[Loop] link=%d rssi=%d lq=%d | drv_armed=%d arm_armed=%d | thr=%.2f str=%.2f | wifi=%d | sim=%s\n",
                      crsfRx.isLinkUp(), crsfRx.getRssi(), crsfRx.getLinkQuality(),
                      motorMgr.isDriveArmed(), motorMgr.isArmArmed(),
                      (float)dbgThrottle, (float)dbgSteering, wifiConnected,
                      simEnabled ? "ON" : "off");

        Serial.printf("[IMU] pitch=%+6.1f roll=%+6.1f yaw=%+6.1f  [CF] tilt=%+6.1f rate=%+6.1f\n",
                      ahrsFilter.getPitch(), ahrsFilter.getRoll(), ahrsFilter.getYaw(),
                      balanceCtrl.getTiltAngle(), balanceCtrl.getGyroRate());

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
                          balanceCtrl.getStateString(), balanceCtrl.getTiltAngle(),
                          (float)balanceCtrl.getEffectiveSetpoint());
        }

        // CAN bus diagnostics
        canBus.printBusStatus();
        Serial.println("==================================================");
    }

    // loopTask has nothing time-critical left -- yield the core
    delay(2);
}
