#pragma once
#include <ESPAsyncWebServer.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <mbedtls/sha256.h>
#include <memory>
#include "network_snapshot.h"

class WebUI {
public:
    using ExportCallback = void(*)(Print*);
    using DisarmCallback = void(*)();
    void begin(ExportCallback export_log, DisarmCallback disarm);
    // Control task: bounded value copy, no allocation, formatting, I/O or waits.
    void publish(const NetworkSnapshot& snapshot);
    MaintenanceGate maintenance;
    bool connected() const { return _connected.load(); }
    void copyIp(char* out, size_t size);
private:
    AsyncWebServer _server{80};
    AsyncWebSocket _ws{"/ws"};
    QueueHandle_t _snapshots = nullptr;
    SemaphoreHandle_t _mutex = nullptr;
    ExportCallback _export = nullptr;
    DisarmCallback _disarm = nullptr;
    std::atomic<bool> _connected{false};
    String _json = "{}", _ip = "0.0.0.0";
    char _runningSlot[17] = "unknown", _imageSha[65] = {};
    uint32_t _otaCapacity = 0;
    int _networkCore = -1, _eventCore = -1;
    uint32_t _dropped = 0;
    bool _ap = false;
    uint32_t _reconnectAtMs = 0;
    AsyncWebServerRequest* _ota_request = nullptr;
    size_t _ota_size = 0, _ota_received = 0;
    uint32_t _ota_last_ms = 0, _restart_ms = 0;
    const char* _ota_failure = "";
    size_t _ota_failed_bytes = 0;
    uint32_t _ota_failed_idle_ms = 0;
    String _ota_sha;
    mbedtls_sha256_context _sha;
    std::weak_ptr<uint8_t> _download;
    void run();
    void setupRoutes();
    void refreshTelemetry();
    bool authorized(AsyncWebServerRequest* request);
    bool acquireMaintenance();
    void releaseMaintenance();
    void failOta(const char* reason);
    void upload(AsyncWebServerRequest* request, size_t index, uint8_t* data, size_t len, bool final);
};
