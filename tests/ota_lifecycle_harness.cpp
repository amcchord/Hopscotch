// The Python test inserts the real upload/progress/abort/watchdog methods and
// control-maintenance block. UART parsing is production code. Flash and SHA are
// stubbed: this checks lifecycle/safety ordering, not ESP flash or cryptography.
#include "crsf.h"
#include "network_safety.h"
#include "ota_progress.h"
#include <cassert>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct String : std::string {
    using std::string::string;
    bool isEmpty() const { return empty(); }
    void toLowerCase() { std::transform(begin(), end(), begin(), [](unsigned char c) { return std::tolower(c); }); }
};
struct Header { String text; String value() const { return text; } };
struct AsyncClient {
    uint32_t rx_timeout = 0, ack_timeout = 0;
    bool no_delay = false;
    void setRxTimeout(uint32_t value) { rx_timeout = value; }
    void setAckTimeout(uint32_t value) { ack_timeout = value; }
    void setNoDelay(bool value) { no_delay = value; }
};
struct AsyncWebServerRequest {
    AsyncClient connection;
    AsyncClient* client() { return &connection; }
    void* _tempObject = nullptr;
    std::map<std::string, Header> headers;
    std::function<void()> disconnect;
    bool token_ok = true;
    AsyncWebServerRequest() {
        headers["X-Firmware-Size"].text = "4096";
        headers["X-Firmware-SHA256"].text = String(64, '0');
    }
    ~AsyncWebServerRequest() { free(_tempObject); }
    bool hasHeader(const char* name) { return headers.count(name); }
    const Header* getHeader(const char* name) const { return &headers.at(name); }
    void send(int, const char*, const char*) {}
    void onDisconnect(std::function<void()> callback) { disconnect = callback; }
};
struct Guard { explicit Guard(int) {} };
struct ProgressQueue { OtaProgress value; std::vector<OtaPhase> phases; } queue;
void xQueueOverwrite(ProgressQueue* q, const OtaProgress* p) { q->value = *p; q->phases.push_back(p->phase); }
void xQueuePeek(ProgressQueue* q, OtaProgress* p, int) { *p = q->value; }
CrsfReceiver crsfRx;
HardwareSerial uart;
bool eligible = true, rcRearmRequired = false, waitingForDoubleTap = true;
bool prevCalTrigger = false, prevMoveTrigger = false, calHoldFired = true;
uint32_t calHoldStart = 90;
float dbgThrottle = 1, dbgSteering = 1;
unsigned motionTicks = 0, stopCalls = 0;
struct Drive { void emergencyStop() {} } driveCtrl;
struct Arms { void clearOverride() {} void holdPosition() {} } armCtrl;
struct Motors {
    void disarmAll() { assert(crsfRx.suspended()); ++stopCalls; }
    void processFeedback() {}
} motorMgr;
bool maintenanceAllowed() { return eligible; }
void serviceWebDisarm() {}
void pollSerialCommands() {}
void publishNetworkSnapshot(uint32_t) {}
void controlOnce();
struct Flash {
    bool begin_ok = true, write_ok = true, end_ok = true;
    bool began = false, ended = false, aborted = false;
    size_t bytes = 0;
    bool begin(size_t, int) {
        assert(crsfRx.suspended() && !uart.begun && rcRearmRequired && stopCalls == 1);
        assert(queue.value.phase == OtaPhase::Preparing && queue.value.received == 0);
        began = true; return begin_ok;
    }
    size_t write(uint8_t*, size_t count) {
        assert(crsfRx.suspended() && !uart.begun);
        controlOnce(); assert(motionTicks == 0);
        if (!write_ok) return 0;
        bytes += count; return count;
    }
    bool end() {
        assert(queue.value.phase == OtaPhase::Verifying && queue.value.percent() == 99);
        ended = end_ok; return end_ok;
    }
    void abort() { aborted = true; }
} Update;
constexpr int U_FLASH = 0;
struct Partition { size_t size = 3342336; } partition;
const Partition* esp_ota_get_next_update_partition(void*) { return &partition; }
struct mbedtls_sha256_context {};
void mbedtls_sha256_free(mbedtls_sha256_context*) {}
void mbedtls_sha256_init(mbedtls_sha256_context*) {}
void mbedtls_sha256_starts_ret(mbedtls_sha256_context*, int) {}
void mbedtls_sha256_update_ret(mbedtls_sha256_context*, uint8_t*, size_t) {}
void mbedtls_sha256_finish_ret(mbedtls_sha256_context*, uint8_t* digest) { memset(digest, 0, 32); }
struct UploadResult { int code; const char* message; };
struct WebUI {
    int _mutex = 0;
    MaintenanceGate maintenance;
    AsyncWebServerRequest* _ota_request = nullptr;
    size_t _ota_size = 0, _ota_received = 0;
    uint32_t _ota_last_ms = 0, _restart_ms = 0;
    const char* _ota_failure = "";
    size_t _ota_failed_bytes = 0;
    uint32_t _ota_failed_idle_ms = 0;
    String _ota_sha;
    mbedtls_sha256_context _sha;
    ProgressQueue* _otaProgress = &queue;
    OtaProgress _ota_progress;
    bool authorized(AsyncWebServerRequest* r) { return r->token_ok; }
    bool acquireMaintenance(bool ota) {
        if (!maintenance.request(ota)) return false;
        controlOnce();
        if (maintenance.granted()) return true;
        maintenance.release(); return false;
    }
    void releaseMaintenance() { maintenance.release(); }
    OtaProgress otaProgress() const;
    void publishOta(OtaPhase);
    void failOta(const char*);
    void upload(AsyncWebServerRequest*, size_t, uint8_t*, size_t, bool);
    void watchdog();
} webUI;

// IMPLEMENTATIONS

int main() {
    uint8_t data[4096] = {0xe9};
    auto fresh = [] {
        webUI.maintenance.release(); webUI._ota_request = nullptr;
        webUI._restart_ms = webUI._ota_size = webUI._ota_received = 0;
        queue = {}; Update = {}; eligible = true;
        rcRearmRequired = false; waitingForDoubleTap = true;
        motionTicks = stopCalls = 0; fake_ms = 100;
        uart.end(); crsfRx.begin(uart, 1, 2, 420000);
    };
    fresh();
    AsyncWebServerRequest upload, other;
    webUI.upload(&upload, 0, data, 1024, false);
    assert(webUI.maintenance.granted() && webUI.maintenance.ota() && motionTicks == 0);
    assert(upload.connection.rx_timeout == OTA_RX_TIMEOUT_S);
    assert(upload.connection.ack_timeout == OTA_ACK_TIMEOUT_MS && upload.connection.no_delay);
    assert(!waitingForDoubleTap && prevCalTrigger && prevMoveTrigger && calHoldStart == 0);
    assert(!calHoldFired && dbgThrottle == 0 && dbgSteering == 0);
    auto p = webUI.otaProgress();
    assert(p.phase == OtaPhase::Receiving && p.received == 1024 && p.percent() == 25);
    webUI.upload(&other, 0, data, 1024, false);
    assert(webUI._ota_request == &upload && webUI.otaProgress().received == 1024);
    fake_ms += 1000;
    webUI.upload(&upload, 1024, data + 1024, 3072, true);
    p = webUI.otaProgress();
    assert(Update.ended && p.phase == OtaPhase::Rebooting && p.percent() == 100);
    assert(p.received == 4096 && p.total == 4096 && p.bytesPerSecond(fake_ms) == 4096);
    upload.disconnect(); controlOnce();
    assert(crsfRx.suspended() && webUI.maintenance.granted() && motionTicks == 0);

    for (int failure = 0; failure < 8; ++failure) {
        fresh(); AsyncWebServerRequest request;
        if (failure == 0) Update.begin_ok = false;
        if (failure == 1) Update.write_ok = false;
        if (failure == 2) Update.end_ok = false;
        if (failure == 3) request.headers["X-Firmware-SHA256"].text = String(64, '1');
        const bool complete = failure < 4;
        webUI.upload(&request, 0, data, complete ? 4096 : 1024, complete);
        if (failure == 4) request.disconnect();
        if (failure == 5) { fake_ms += OTA_IDLE_TIMEOUT_MS + 1; webUI.watchdog(); }
        if (failure == 6) webUI.upload(&request, 0, data, 1024, false); // Multiple files.
        if (failure == 7) webUI.upload(&request, 999, data, 1024, false); // Wrong offset.
        p = webUI.otaProgress();
        assert(webUI._ota_failure[0] && webUI._ota_failed_bytes == p.received);
        assert(Update.aborted && !webUI.maintenance.busy() && !webUI._restart_ms);
        assert(p.phase == OtaPhase::Failed && p.visible(fake_ms) && p.percent() < 100);
        assert(p.received == (failure < 2 ? 0u : (complete ? 4096u : 1024u)));
        const auto bytes = Update.bytes;
        webUI.upload(&request, 1024, data, 1024, false); // Late data cannot restart writes.
        assert(Update.bytes == bytes && webUI.otaProgress().phase == OtaPhase::Failed);
        controlOnce();
        assert(!crsfRx.suspended() && uart.begun && !crsfRx.isLinkUp() && rcRearmRequired);
    }
    for (int rejected = 0; rejected < 3; ++rejected) {
        fresh(); AsyncWebServerRequest request;
        if (rejected == 0) request.token_ok = false;
        if (rejected == 1) eligible = false;
        if (rejected == 2) request.headers["X-Firmware-Size"].text = "0";
        webUI.upload(&request, 0, data, 1024, false);
        assert(!Update.began && !webUI.otaProgress().visible(fake_ms));
        controlOnce();
        assert(!crsfRx.suspended() && !webUI.maintenance.busy());
    }
    std::cout << "OTA production callback/interlock lifecycle tests passed\n";
}
