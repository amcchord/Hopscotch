#pragma once

#include <M5Unified.h>
#include <M5GFX.h>
#include "motor_manager.h"
#include "crsf.h"

class Display {
public:
    void begin();

    // Render a full frame to the sprite then push to screen
    void render(const MotorManager& motors,
                const CrsfReceiver& crsf,
                bool wifi_connected,
                const char* ip_address,
                bool drive_armed,
                bool arm_armed);

private:
    M5Canvas _sprite;
    bool _initialized = false;

    void drawStatusBar(bool wifi_connected, const char* ip_address,
                       bool drive_armed, bool arm_armed);
    void drawMotorDiagram(const MotorManager& motors);
    void drawChannelBars(const CrsfReceiver& crsf);

    // Color helpers
    uint16_t motorColor(const MotorState& m) const;
};
