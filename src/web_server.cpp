#include "web_server.h"
#include <ArduinoJson.h>
#include <LittleFS.h>

void WebUI::begin(SettingsManager* settings, MotorManager* motors, CrsfReceiver* crsf) {
    _settings = settings;
    _motors = motors;
    _crsf = crsf;

    setupRoutes();

    _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
        onWsEvent(s, c, type, arg, data, len);
    });

    _server.addHandler(&_ws);
    _server.begin();
    Serial.println("[Web] Server started on port 80");
}

void WebUI::setWifiState(bool connected, const char* ip) {
    _wifi_connected = connected;
    strlcpy(_ip_address, ip, sizeof(_ip_address));
}

void WebUI::setupRoutes() {
    // Serve static files from LittleFS
    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // GET settings
    _server.on("/api/settings", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", _settings->toJson());
    });

    // POST settings
    _server.on("/api/settings", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        nullptr,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            String body;
            body.reserve(total);
            body += String(reinterpret_cast<const char*>(data), len);

            if (index + len >= total) {
                if (_settings->fromJson(body)) {
                    _settings->save();
                    if (_settings_cb) _settings_cb();
                    request->send(200, "application/json", "{\"status\":\"ok\"}");
                } else {
                    request->send(400, "application/json", "{\"error\":\"invalid json\"}");
                }
            }
        }
    );

    // POST disarm all
    _server.on("/api/disarm", HTTP_POST, [this](AsyncWebServerRequest* request) {
        if (_disarm_cb) _disarm_cb();
        request->send(200, "application/json", "{\"status\":\"disarmed\"}");
    });

    // POST change CAN ID
    _server.on("/api/change-can-id", HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        nullptr,
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            String body;
            body.reserve(total);
            body += String(reinterpret_cast<const char*>(data), len);

            if (index + len >= total) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, body);
                if (err) {
                    request->send(400, "application/json", "{\"error\":\"invalid json\"}");
                    return;
                }

                uint8_t old_id = doc["oldId"] | 0;
                uint8_t new_id = doc["newId"] | 0;

                if (old_id == 0 || new_id == 0) {
                    request->send(400, "application/json", "{\"error\":\"missing oldId or newId\"}");
                    return;
                }

                bool ok = false;
                if (_canid_cb) {
                    ok = _canid_cb(old_id, new_id);
                }

                if (ok) {
                    request->send(200, "application/json", "{\"status\":\"ok\"}");
                } else {
                    request->send(500, "application/json", "{\"error\":\"CAN ID change failed\"}");
                }
            }
        }
    );

    // POST reset settings
    _server.on("/api/reset-settings", HTTP_POST, [this](AsyncWebServerRequest* request) {
        _settings->resetDefaults();
        if (_settings_cb) _settings_cb();
        request->send(200, "application/json", "{\"status\":\"reset\"}");
    });
}

void WebUI::onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[Web] WS client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[Web] WS client #%u disconnected\n", client->id());
    }
}

void WebUI::sendTelemetry() {
    if (_ws.count() == 0) return;

    String json = buildTelemetryJson();
    _ws.textAll(json);
    _ws.cleanupClients();
}

String WebUI::buildTelemetryJson() {
    JsonDocument doc;

    JsonArray motors = doc["motors"].to<JsonArray>();
    for (int i = 0; i < _motors->motorCount(); i++) {
        const MotorState& m = _motors->getMotor(i);
        JsonObject mo = motors.add<JsonObject>();
        mo["id"] = m.can_id;
        mo["pos"] = serialized(String(m.position, 2));
        mo["vel"] = serialized(String(m.velocity, 2));
        mo["torque"] = serialized(String(m.torque, 2));
        mo["temp"] = serialized(String(m.temperature, 1));
        mo["error"] = m.errors;
        mo["online"] = m.online;
        mo["enabled"] = m.enabled;
    }

    JsonArray channels = doc["channels"].to<JsonArray>();
    for (int i = 0; i < 16; i++) {
        channels.add(_crsf->getChannel(i));
    }

    doc["drive_armed"] = _motors->isDriveArmed();
    doc["arm_armed"] = _motors->isArmArmed();
    doc["link_up"] = _crsf->isLinkUp();
    doc["rssi"] = _crsf->getRssi();
    doc["lq"] = _crsf->getLinkQuality();

    JsonObject wifi = doc["wifi"].to<JsonObject>();
    wifi["connected"] = _wifi_connected;
    wifi["ip"] = _ip_address;

    String output;
    serializeJson(doc, output);
    return output;
}
