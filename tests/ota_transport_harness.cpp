// This harness receives the real upload/abort/watchdog and pinned TCP poll
// implementations from scripts/check_ota_transport.py. Hardware flash and SHA
// primitives are stubbed; this tests transport lifetime and maintenance, not crypto.
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include "network_safety.h"
#include "ota_progress.h"

uint32_t now_ms = 100;
uint32_t millis() { return now_ms; }
struct String : std::string {
    using std::string::string;
    String(const std::string& s) : std::string(s) {}
    bool isEmpty() const { return empty(); }
    void toLowerCase() { std::transform(begin(), end(), begin(), [](unsigned char c) { return std::tolower(c); }); }
};
struct tcp_pcb {};
constexpr int8_t ERR_OK = 0;
#define log_d(...) ((void)0)
struct AsyncClient {
    tcp_pcb pcb;
    tcp_pcb* _pcb = &pcb;
    uint32_t _rx_timeout = 3, _ack_timeout = 0, _rx_last_packet = 100, _rx_last_ack = 100, _tx_last_packet = 100;
    std::function<void(void*, AsyncClient*, uint32_t)> _timeout_cb;
    std::function<void(void*, AsyncClient*)> _poll_cb;
    void *_timeout_cb_arg = nullptr, *_poll_cb_arg = nullptr;
    bool closed = false, no_delay = false;
    void _close() { closed = true; }
    void setRxTimeout(uint32_t seconds) { _rx_timeout = seconds; }
    void setAckTimeout(uint32_t ms) { _ack_timeout = ms; }
    void setNoDelay(bool value) { no_delay = value; }
    int8_t _poll(tcp_pcb*);
};
struct Header { String text; String value() { return text; } };
struct AsyncWebServerRequest {
    void* _tempObject = nullptr;
    AsyncClient transport;
    std::map<std::string, Header> headers;
    std::function<void()> disconnect;
    bool token_ok = true;
    AsyncWebServerRequest() {
        headers["X-Firmware-Size"].text = "4096";
        headers["X-Firmware-SHA256"].text = String(64, '0');
    }
    ~AsyncWebServerRequest() { free(_tempObject); }
    bool hasHeader(const char* name) { return headers.count(name); }
    Header* getHeader(const char* name) { return &headers.at(name); }
    AsyncClient* client() { return &transport; }
    void send(int, const char*, const char*) {}
    void onDisconnect(std::function<void()> callback) { disconnect = callback; }
};
struct Guard { explicit Guard(int) {} };
struct Flash {
    bool began = false, ended = false, aborted = false, write_ok = true, end_ok = true;
    size_t bytes = 0;
    bool begin(size_t, int) { began = true; return true; }
    size_t write(uint8_t*, size_t count) { if (!write_ok) return 0; bytes += count; return count; }
    bool end() { ended = end_ok; return end_ok; }
    void abort() { aborted = true; }
} Update;
constexpr int U_FLASH = 0;
struct esp_partition_t { size_t size = 3342336; } partition;
const esp_partition_t* esp_ota_get_next_update_partition(void*) { return &partition; }
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
    bool eligible = true;
    AsyncWebServerRequest* _ota_request = nullptr;
    size_t _ota_size = 0, _ota_received = 0, _ota_failed_bytes = 0;
    uint32_t _ota_last_ms = 0, _restart_ms = 0, _ota_failed_idle_ms = 0;
    const char* _ota_failure = "";
    String _ota_sha;
    mbedtls_sha256_context _sha;
    bool authorized(AsyncWebServerRequest* r) { return r->token_ok; }
    bool acquireMaintenance(bool ota = false) { if (!maintenance.request(ota)) return false; maintenance.service(eligible); return maintenance.granted(); }
    void releaseMaintenance() { maintenance.release(); }
    // Real progress publication/control ordering are covered by test_ota_lifecycle.py.
    void publishOta(OtaPhase) {}
    void failOta(const char*);
    void upload(AsyncWebServerRequest*, size_t, uint8_t*, size_t, bool);
    void serviceWatchdog();
};

// IMPLEMENTATIONS

int main() {
    uint8_t data[4096] = {0xe9};
    auto fresh = [] { now_ms = 100; Update = {}; };
    fresh();
    AsyncClient baseline;
    now_ms += 4000;
    baseline._poll(baseline._pcb);
    assert(baseline.closed); // Reproduces the pinned HTTP server's three-second gap failure.

    fresh();
    WebUI server; AsyncWebServerRequest request;
    server.upload(&request, 0, data, 1024, false);
    assert(Update.began && server.maintenance.granted());
    assert(request.transport.no_delay);
    assert(request.transport._rx_timeout * 1000 > OTA_IDLE_TIMEOUT_MS);
    now_ms += 4000;
    request.transport._poll(request.transport._pcb);
    server.serviceWatchdog();
    assert(!request.transport.closed && !Update.aborted && server.maintenance.granted());
    server.upload(&request, 1024, data + 1024, 3072, true);
    assert(Update.bytes == 4096 && Update.ended && server._restart_ms > millis());
    assert(server.maintenance.granted()); // Successful application holds inhibit through reboot.
    request.disconnect();
    assert(!Update.aborted && server.maintenance.granted());

    fresh();
    WebUI abandoned; AsyncWebServerRequest pending;
    abandoned.upload(&pending, 0, data, 1024, false);
    now_ms += OTA_IDLE_TIMEOUT_MS + 1;
    abandoned.serviceWatchdog();
    assert(Update.aborted && !Update.ended && !abandoned.maintenance.busy());
    assert(!abandoned._ota_request && !abandoned._restart_ms);
    assert(std::string(abandoned._ota_failure) == "inactivity_timeout");
    assert(abandoned._ota_failed_bytes == 1024 && abandoned._ota_failed_idle_ms == OTA_IDLE_TIMEOUT_MS + 1);
    abandoned.upload(&pending, 1024, data, 1024, false); // Late network data cannot revive expired writes.
    assert(Update.bytes == 1024);

    fresh();
    WebUI disconnected; AsyncWebServerRequest lost;
    disconnected.upload(&lost, 0, data, 1024, false);
    now_ms += 50; lost.disconnect();
    assert(Update.aborted && !disconnected.maintenance.busy());
    assert(std::string(disconnected._ota_failure) == "client_disconnect");
    assert(disconnected._ota_failed_bytes == 1024 && disconnected._ota_failed_idle_ms == 50);

    for (int scenario = 0; scenario < 3; ++scenario) {
        fresh(); WebUI rejected; AsyncWebServerRequest bad;
        if (scenario == 0) bad.token_ok = false;
        if (scenario == 1) rejected.eligible = false;
        if (scenario == 2) bad.headers["X-Firmware-Size"].text = "0";
        rejected.upload(&bad, 0, data, 1024, false);
        assert(!Update.began && bad.transport._rx_timeout == 3); // Longer timeout is not global.
    }
    fresh();
    WebUI exclusive; AsyncWebServerRequest first, second;
    exclusive.upload(&first, 0, data, 1024, false);
    exclusive.upload(&second, 0, data, 1024, false);
    assert(exclusive._ota_request == &first && exclusive.maintenance.granted());
    assert(Update.bytes == 1024 && second.transport._rx_timeout == 3);

    for (int scenario = 0; scenario < 4; ++scenario) {
        fresh(); WebUI rejected; AsyncWebServerRequest bad;
        if (scenario == 0) bad.headers["X-Firmware-SHA256"].text = String(64, '1');
        if (scenario == 1) Update.write_ok = false;
        if (scenario == 2) Update.end_ok = false;
        if (scenario == 3) bad.headers["X-Firmware-Size"].text = "4097";
        rejected.upload(&bad, 0, data, 4096, true);
        assert(Update.aborted && !rejected._restart_ms && !rejected.maintenance.busy());
    }
    std::cout << "OTA gap recovery, bounded expiry, disconnect diagnostics, ownership and verification guards passed\n";
}
