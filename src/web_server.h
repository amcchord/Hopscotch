#pragma once

#include <ESPAsyncWebServer.h>
#include "motor_manager.h"
#include "crsf.h"
#include "settings.h"

class WebUI {
public:
    void begin(SettingsManager* settings, MotorManager* motors, CrsfReceiver* crsf);

    // Push telemetry to connected WebSocket clients (call at ~10Hz)
    void sendTelemetry();

    // Callback to be set by main.cpp for when settings change
    using SettingsChangedCallback = void(*)();
    void onSettingsChanged(SettingsChangedCallback cb) { _settings_cb = cb; }

    // Callback for emergency disarm
    using DisarmCallback = void(*)();
    void onDisarmRequested(DisarmCallback cb) { _disarm_cb = cb; }

    // Callback for CAN ID change
    using CanIdChangeCallback = bool(*)(uint8_t old_id, uint8_t new_id);
    void onCanIdChange(CanIdChangeCallback cb) { _canid_cb = cb; }

    // WiFi state for telemetry
    void setWifiState(bool connected, const char* ip);

private:
    AsyncWebServer _server{80};
    AsyncWebSocket _ws{"/ws"};

    SettingsManager* _settings = nullptr;
    MotorManager*    _motors = nullptr;
    CrsfReceiver*    _crsf = nullptr;

    SettingsChangedCallback _settings_cb = nullptr;
    DisarmCallback          _disarm_cb = nullptr;
    CanIdChangeCallback     _canid_cb = nullptr;

    bool _wifi_connected = false;
    char _ip_address[32] = "0.0.0.0";

    void setupRoutes();
    void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                   AwsEventType type, void* arg, uint8_t* data, size_t len);
    String buildTelemetryJson();
};
