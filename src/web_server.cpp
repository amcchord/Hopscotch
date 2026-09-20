#include "web_server.h"
#if __has_include("network_secrets.h")
#include "network_secrets.h"
#else
#error "Copy network_secrets.example.h to network_secrets.h and configure your device"
#endif
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_heap_caps.h>
#include <algorithm>

static_assert(ARDUINO_EVENT_RUNNING_CORE == 0, "Network events must not preempt balance");

extern const uint8_t dashboard_start[] asm("_binary_data_network_html_start");
extern const uint8_t dashboard_end[] asm("_binary_data_network_html_end");

namespace {
struct UploadResult { int code; const char* message; };
class Guard {
    SemaphoreHandle_t mutex;
public:
    explicit Guard(SemaphoreHandle_t m) : mutex(m) { xSemaphoreTake(mutex, portMAX_DELAY); }
    ~Guard() { xSemaphoreGive(mutex); }
};
class BufferPrint : public Print {
public:
    uint8_t* data;
    size_t capacity, size = 0;
    bool failed = false;
    BufferPrint(uint8_t* p, size_t n) : data(p), capacity(n) {}
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t* p, size_t n) override {
        if (n > capacity - size) { failed = true; return 0; }
        memcpy(data + size, p, n); size += n; return n;
    }
};
void timingJson(JsonObject o, const LoopTiming& t) {
    o["ticks"] = t.ticks; o["max_us"] = t.max_us;
    o["over_7500_us"] = t.over_7500_us; o["over_10000_us"] = t.over_10000_us;
}
}

void WebUI::begin(ExportCallback export_log, DisarmCallback disarm) {
    _export = export_log; _disarm = disarm;
    _snapshots = xQueueCreate(1, sizeof(NetworkSnapshot));
    _mutex = xSemaphoreCreateMutex();
    if (!_snapshots || !_mutex) { Serial.println("[Network] Allocation failed; networking disabled"); return; }
    mbedtls_sha256_init(&_sha);
    if (xTaskCreatePinnedToCore([](void* p) { static_cast<WebUI*>(p)->run(); },
            "NetworkTask", 12288, this, 2, nullptr, 0) != pdPASS)
        Serial.println("[Network] Task creation failed; networking disabled");
}
void WebUI::publish(const NetworkSnapshot& s) {
    if (_snapshots) xQueueOverwrite(_snapshots, &s);
}
void WebUI::copyIp(char* out, size_t size) {
    if (!_mutex) { strlcpy(out, "offline", size); return; }
    Guard guard(_mutex); strlcpy(out, _ip.c_str(), size);
}
bool WebUI::authorized(AsyncWebServerRequest* r) {
    if (r->hasHeader("Authorization") &&
        r->getHeader("Authorization")->value() == String("Bearer ") + HOPSCOTCH_API_TOKEN) return true;
    r->send(401, "application/json", "{\"error\":\"device token required\"}");
    return false;
}
bool WebUI::acquireMaintenance() {
    if (!maintenance.request()) return false;
    const auto start = millis();
    while (maintenance.state() == MaintenanceGate::Requested && millis() - start < 300) delay(1);
    if (maintenance.granted()) return true;
    maintenance.release();
    return false;
}
void WebUI::releaseMaintenance() { maintenance.release(); }
void WebUI::failOta() {
    Update.abort(); _ota_request = nullptr; _ota_received = 0;
    mbedtls_sha256_free(&_sha); mbedtls_sha256_init(&_sha);
    releaseMaintenance();
}

void WebUI::setupRoutes() {
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        auto* response = r->beginResponse(200, "text/html; charset=utf-8", dashboard_start,
                                         dashboard_end - dashboard_start - 1);
        response->addHeader("Cache-Control", "no-store"); r->send(response);
    });
    _server.on("/api/telemetry", HTTP_GET, [this](AsyncWebServerRequest* r) {
        Guard guard(_mutex);
        auto* response = r->beginResponse(200, "application/json", _json);
        response->addHeader("Cache-Control", "no-store"); r->send(response);
    });
    _server.on("/api/info", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument d;
        d["firmware"] = "hopscotch-wifi-ota-1"; d["build"] = __DATE__ " " __TIME__;
        d["running_slot"] = _runningSlot;
        d["image_sha256"] = _imageSha;
        d["ota_capacity"] = _otaCapacity;
        d["flash_bytes"] = ESP.getFlashChipSize(); d["psram_bytes"] = ESP.getPsramSize();
        d["free_heap"] = ESP.getFreeHeap(); d["min_free_heap"] = ESP.getMinFreeHeap();
        d["free_psram"] = ESP.getFreePsram();
        d["automatic_boot_rollback"] = false;
        d["reset_reason"] = static_cast<int>(esp_reset_reason());
        d["network_core"] = _networkCore; d["http_core"] = xPortGetCoreID();
        d["wifi_event_core"] = _eventCore;
        String json; serializeJson(d, json); r->send(200, "application/json", json);
    });
    _server.on("/api/disarm", HTTP_POST, [this](AsyncWebServerRequest* r) {
        if (!authorized(r)) return;
        if (_disarm) _disarm();
        r->send(202, "application/json", "{\"status\":\"disarm requested\"}");
    });
    _server.on("/api/wifi/reconnect", HTTP_POST, [this](AsyncWebServerRequest* r) {
        if (!authorized(r)) return;
        Guard guard(_mutex);
        if (!acquireMaintenance()) { r->send(409, "text/plain", "Disarm before reconnecting WiFi"); return; }
        _reconnectAtMs = millis() + 5000;
        // Finish the HTTP acknowledgment before removing its WiFi transport.
        r->onDisconnect([this]() {
            Guard guard(_mutex);
            if (_reconnectAtMs) _reconnectAtMs = millis() + 100;
        });
        r->send(202, "text/plain", "WiFi reconnect requested; arming is inhibited during association");
    });
    _server.on("/api/log", HTTP_GET, [this](AsyncWebServerRequest* r) {
        if (!authorized(r)) return;
        Guard guard(_mutex);
        if (!_download.expired()) { r->send(429, "application/json", "{\"error\":\"download already active\"}"); return; }
        if (!acquireMaintenance()) { r->send(409, "application/json", "{\"error\":\"disarm both groups and wait for log save\"}"); return; }
        constexpr size_t capacity = 4 * 1024 * 1024;
        auto data = std::shared_ptr<uint8_t>(static_cast<uint8_t*>(heap_caps_malloc(capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)), free);
        if (!data) { releaseMaintenance(); r->send(503, "text/plain", "Insufficient PSRAM"); return; }
        r->client()->setNoDelay(true);
        BufferPrint out(data.get(), capacity);
        if (_export) _export(&out);
        // All flash reads completed under the interlock. The network drains an
        // immutable PSRAM copy; slow clients cannot hold up control or log save.
        releaseMaintenance();
        if (out.failed || out.size == 0) { r->send(500, "text/plain", "Log export failed"); return; }
        _download = data;
        const size_t size = out.size;
        auto* response = r->beginResponse("text/csv", size,
            [data, size](uint8_t* buffer, size_t maxLen, size_t index) -> size_t {
                if (index >= size) return 0;
                const auto n = std::min(maxLen, size - index);
                memcpy(buffer, data.get() + index, n); return n;
            });
        response->addHeader("Content-Disposition", "attachment; filename=balance.csv");
        response->addHeader("Cache-Control", "no-store"); r->send(response);
    });
    _server.on("/api/ota", HTTP_POST,
        [this](AsyncWebServerRequest* r) {
            Guard guard(_mutex);
            auto* result = static_cast<UploadResult*>(r->_tempObject);
            if (!result) {
                if (authorized(r)) r->send(400, "text/plain", "One firmware file is required");
            } else if (result->code) {
                r->send(result->code, "text/plain", result->message);
                if (r == _ota_request && _restart_ms) _ota_request = nullptr;
            }
        },
        [this](AsyncWebServerRequest* r, String, size_t index, uint8_t* data, size_t len, bool final) {
            upload(r, index, data, len, final);
        });
    // Old cross-core settings and CAN mutation routes cannot safely run beside
    // the control owner. Keep tuning on USB until a transactional command API exists.
    for (const char* path : {"/api/settings", "/api/change-can-id", "/api/reset-settings"})
        _server.on(path, HTTP_ANY, [](AsyncWebServerRequest* r) {
            r->send(410, "application/json", "{\"error\":\"use USB console for configuration\"}");
        });
    _server.onNotFound([](AsyncWebServerRequest* r) { r->send(404, "text/plain", "Not found"); });
    _ws.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType type, void*, uint8_t*, size_t) {
        if (type == WS_EVT_CONNECT) {
            c->client()->setNoDelay(true);
            c->setCloseClientOnQueueFull(false);
            if (_ws.count() > 4) c->close(1013, "Four telemetry clients maximum");
        }
        // WebSocket is deliberately receive-only from the operator's perspective.
    });
    _server.addHandler(&_ws); _server.begin();
}

void WebUI::upload(AsyncWebServerRequest* r, size_t index, uint8_t* data, size_t len, bool final) {
    Guard guard(_mutex);
    auto* result = static_cast<UploadResult*>(r->_tempObject);
    if (index == 0) {
        if (result) {
            if (_ota_request == r && !_restart_ms) failOta();
            if (!_restart_ms) { result->code = 400; result->message = "Only one firmware file is allowed"; }
            return;
        }
        result = static_cast<UploadResult*>(calloc(1, sizeof(UploadResult)));
        if (!result) { r->send(503, "text/plain", "Insufficient memory"); return; }
        r->_tempObject = result; // Freed by AsyncWebServerRequest, including disconnects.
        if (!authorized(r)) return;
        if (_ota_request || _restart_ms || !acquireMaintenance()) {
            result->code = 409; result->message = "Update busy or robot not disarmed"; return;
        }
        String sizeText = r->hasHeader("X-Firmware-Size") ? r->getHeader("X-Firmware-Size")->value() : "";
        _ota_sha = r->hasHeader("X-Firmware-SHA256") ? r->getHeader("X-Firmware-SHA256")->value() : "";
        _ota_sha.toLowerCase();
        bool valid = !sizeText.isEmpty() && _ota_sha.length() == 64;
        for (char c : sizeText) valid &= c >= '0' && c <= '9';
        for (char c : _ota_sha) valid &= isxdigit(static_cast<unsigned char>(c)) != 0;
        _ota_size = strtoul(sizeText.c_str(), nullptr, 10);
        const auto* next = esp_ota_get_next_update_partition(nullptr);
        if (!valid || !next || _ota_size < 1024 || _ota_size > next->size || !len || data[0] != 0xe9) {
            releaseMaintenance(); result->code = 400; result->message = "Invalid application, size or SHA-256"; return;
        }
        if (!Update.begin(_ota_size, U_FLASH)) {
            failOta(); result->code = 500; result->message = "Cannot begin update"; return;
        }
        _ota_request = r; _ota_received = 0;
        result->code = 408; result->message = "Update did not complete";
        mbedtls_sha256_starts_ret(&_sha, 0);
        r->onDisconnect([this, r]() {
            Guard guard(_mutex);
            if (_ota_request == r && !_restart_ms) failOta();
        });
    }
    if (_ota_request != r || _restart_ms) return;
    _ota_last_ms = millis();
    if (index != _ota_received || len > _ota_size - _ota_received || Update.write(data, len) != len) {
        failOta(); result->code = 400; result->message = "Update write or length failed"; return;
    }
    mbedtls_sha256_update_ret(&_sha, data, len); _ota_received += len;
    if (final) {
        uint8_t digest[32]; char hex[65];
        mbedtls_sha256_finish_ret(&_sha, digest);
        for (int i=0; i<32; ++i) snprintf(hex + i*2, 3, "%02x", digest[i]);
        if (_ota_received != _ota_size || _ota_sha != hex) {
            failOta(); result->code = 400; result->message = "Application size or SHA-256 mismatch"; return;
        }
        if (!Update.end()) { failOta(); result->code = 400; result->message = "ESP application verification failed"; return; }
        result->code = 200; result->message = "Firmware verified; rebooting";
        _restart_ms = millis() + 1000;
        // Maintenance remains latched through reboot; boot requires switch-low.
    }
}

void WebUI::refreshTelemetry() {
    NetworkSnapshot s;
    if (xQueuePeek(_snapshots, &s, 0) != pdTRUE) return;
    JsonDocument d;
    d["schema"] = 1; d["sequence"] = s.sequence; d["uptime_ms"] = s.uptime_ms;
    d["age_ms"] = millis() - s.uptime_ms;
    d["drive_armed"] = s.drive_armed; d["arm_armed"] = s.arm_armed; d["arming"] = s.arming;
    d["link_up"] = s.link_up; d["rssi"] = s.rssi; d["lq"] = s.lq; d["rc_age_ms"] = s.rc_age_ms;
    d["rearm_required"] = s.rearm_required; d["maintenance"] = maintenance.busy();
    d["maintenance_allowed"] = s.safe; d["saving_log"] = s.saving_log;
    d["calibration"] = s.calibration; d["test_mode"] = s.test; d["simulation"] = s.simulation;
    auto b = d["balance"].to<JsonObject>();
    b["state"] = s.balance_state; b["active"] = s.balance_active;
    b["tilt"] = s.tilt; b["rate"] = s.rate; b["setpoint"] = s.setpoint;
    b["imu_age_us"] = s.imu_age_us; b["fault"] = s.inner_fault; b["end_reason"] = s.end_reason;
    auto p = d["power"].to<JsonObject>();
    p["voltage"] = s.voltage; p["current"] = s.current;
    p["voltage_fresh"] = s.voltage_fresh; p["current_fresh"] = s.current_fresh;
    auto motors = d["motors"].to<JsonArray>();
    for (const auto& m : s.motors) {
        auto o = motors.add<JsonObject>();
        o["id"] = m.id; o["pos"] = m.pos; o["vel"] = m.vel; o["torque"] = m.torque;
        o["temp"] = m.temp; o["error"] = m.error; o["online"] = m.online;
        o["enabled"] = m.enabled; o["age_ms"] = m.age_ms;
    }
    auto channels = d["channels"].to<JsonArray>(); for (auto ch : s.channels) channels.add(ch);
    auto t = d["timing"].to<JsonObject>();
    timingJson(t["balance_200hz"].to<JsonObject>(), s.balance_timing);
    timingJson(t["control_200hz"].to<JsonObject>(), s.control_timing);
    d["free_heap"] = ESP.getFreeHeap(); d["min_free_heap"] = ESP.getMinFreeHeap();
    d["free_psram"] = ESP.getFreePsram();
    auto wifi = d["wifi"].to<JsonObject>();
    wifi["connected"] = WiFi.status() == WL_CONNECTED;
    wifi["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
    wifi["ip"] = _ip; wifi["recovery_ap"] = _ap;
    d["ws_dropped"] = _dropped;
    _json = ""; serializeJson(d, _json);
    _ws.cleanupClients(4);
    // A stalled client must not suppress delivery to every other client.
    // Each connection has a two-frame limit and independently drops old work.
    for (const auto& client : _ws.getClients()) {
        if (!client.queueIsFull()) _ws.text(client.id(), _json);
        else ++_dropped;
    }
}

void WebUI::run() {
    // Wait for the control task to exist and latch the boot interlock before
    // starting any association or channel scan.
    while (!acquireMaintenance()) delay(100);
    // Cache partition identity while disarmed. Even the first HTTP info read
    // must not trigger a flash scan or application-hash calculation mid-run.
    const auto* running = esp_ota_get_running_partition();
    const auto* next = esp_ota_get_next_update_partition(nullptr);
    if (running) {
        strlcpy(_runningSlot, running->label, sizeof(_runningSlot));
        uint8_t digest[32];
        if (esp_partition_get_sha256(running, digest) == ESP_OK)
            for (int i=0; i<32; ++i) snprintf(_imageSha + i*2, 3, "%02x", digest[i]);
    }
    _otaCapacity = next ? next->size : 0;
    WiFi.persistent(false); WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA); WiFi.setHostname("hopscotch"); WiFi.setSleep(false);
    WiFi.begin(HOPSCOTCH_WIFI_SSID, HOPSCOTCH_WIFI_PASSWORD);
    _networkCore = xPortGetCoreID();
    const auto events = xTaskGetHandle("arduino_events");
    if (events) _eventCore = xTaskGetAffinity(events);
    MDNS.begin("hopscotch"); MDNS.addService("http", "tcp", 80);
    setupRoutes();
    Serial.println("[Network] WiFi / telemetry / OTA task on core 0");
    uint32_t lastRetry = millis(), lastTelemetry = 0;
    uint32_t connectingUntil = millis() + 12000;
    for (;;) {
        {
            Guard guard(_mutex);
            const bool online = WiFi.status() == WL_CONNECTED;
            const String ip = online ? WiFi.localIP().toString() : (_ap ? WiFi.softAPIP().toString() : "0.0.0.0");
            if (ip != _ip) { _ip = ip; Serial.printf("[Network] %s\n", _ip.c_str()); }
            _connected.store(online || _ap);
            NetworkSnapshot s;
            // Reconfiguration/active scans only while the control task grants
            // maintenance. No network reconnect attempts during arm/balance.
            if (_reconnectAtMs && static_cast<int32_t>(millis() - _reconnectAtMs) >= 0) {
                _reconnectAtMs = 0;
                WiFi.disconnect(false, false);
                WiFi.begin(HOPSCOTCH_WIFI_SSID, HOPSCOTCH_WIFI_PASSWORD);
                connectingUntil = millis() + 12000;
                lastRetry = millis();
            } else if (connectingUntil && (online || static_cast<int32_t>(millis() - connectingUntil) >= 0)) {
                if (!online) WiFi.disconnect(false, false); // cancel outstanding association / scans
                connectingUntil = 0;
                releaseMaintenance();
            }
            if (!online && !connectingUntil && millis() - lastRetry >= 30000 && xQueuePeek(_snapshots, &s, 0) && s.safe) {
                lastRetry = millis();
                if (acquireMaintenance()) {
                    if (!_ap) {
                        WiFi.mode(WIFI_AP_STA);
                        _ap = WiFi.softAP("Hopscotch-Recovery", HOPSCOTCH_AP_PASSWORD);
                    }
                    WiFi.disconnect(false, false);
                    WiFi.begin(HOPSCOTCH_WIFI_SSID, HOPSCOTCH_WIFI_PASSWORD);
                    connectingUntil = millis() + 12000;
                }
            }
            if (_ota_request && !_restart_ms && millis() - _ota_last_ms > 15000) failOta();
            if (_restart_ms && static_cast<int32_t>(millis() - _restart_ms) >= 0) ESP.restart();
            if (millis() - lastTelemetry >= 100) { lastTelemetry = millis(); refreshTelemetry(); }
        }
        delay(5);
    }
}
