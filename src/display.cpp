#include "display.h"

static constexpr int SCREEN_W = 128;
static constexpr int SCREEN_H = 128;

static constexpr uint16_t COL_BG       = 0x0000;  // black
static constexpr uint16_t COL_TEXT     = 0xFFFF;  // white
static constexpr uint16_t COL_GREEN    = 0x07E0;
static constexpr uint16_t COL_RED      = 0xF800;
static constexpr uint16_t COL_YELLOW   = 0xFFE0;
static constexpr uint16_t COL_GRAY     = 0x7BEF;
static constexpr uint16_t COL_DARKGRAY = 0x4208;
static constexpr uint16_t COL_BLUE     = 0x001F;
static constexpr uint16_t COL_ORANGE   = 0xFD20;

void Display::begin() {
    _sprite.createSprite(SCREEN_W, SCREEN_H);
    _sprite.setTextSize(1);
    _initialized = true;
    Serial.println("[Display] Sprite initialized 128x128");
}

uint16_t Display::motorColor(const MotorState& m) const {
    if (!m.online) return COL_GRAY;
    if (m.has_fault) return COL_RED;
    if (!m.enabled) return COL_YELLOW;
    return COL_GREEN;
}

void Display::render(const MotorManager& motors,
                     const CrsfReceiver& crsf,
                     bool wifi_connected,
                     const char* ip_address,
                     bool drive_armed,
                     bool arm_armed,
                     const ArmController* arm_ctrl,
                     bool drive_arming,
                     bool arm_arming) {
    if (!_initialized) return;

    _sprite.fillSprite(COL_BG);

    if (arm_ctrl && arm_ctrl->isInCalMode()) {
        drawCalMode(*arm_ctrl);
    } else {
        drawStatusBar(wifi_connected, ip_address, drive_armed, arm_armed,
                      drive_arming, arm_arming);
        drawMotorDiagram(motors);
        drawChannelBars(crsf);
    }

    _sprite.pushSprite(&M5.Display, 0, 0);
}

void Display::drawStatusBar(bool wifi_connected, const char* ip_address,
                             bool drive_armed, bool arm_armed,
                             bool drive_arming, bool arm_arming) {
    // Row 0-13: status bar
    if (wifi_connected) {
        _sprite.setTextColor(COL_GREEN);
        _sprite.setCursor(1, 1);
        _sprite.print("W");
        _sprite.setTextColor(COL_TEXT);
        _sprite.setCursor(10, 1);
        _sprite.print(ip_address);
    } else {
        _sprite.setTextColor(COL_RED);
        _sprite.setCursor(1, 1);
        _sprite.print("WiFi: --");
    }

    // Blink phase for arming indicators (~2 Hz)
    bool blink_on = (millis() / 250) % 2 == 0;

    // Drive arm status on right
    _sprite.setCursor(90, 1);
    if (drive_arming) {
        if (blink_on) {
            _sprite.setTextColor(COL_ORANGE);
            _sprite.print("DRV");
        } else {
            _sprite.setTextColor(COL_DARKGRAY);
            _sprite.print("DRV");
        }
    } else if (drive_armed) {
        _sprite.setTextColor(COL_GREEN);
        _sprite.print("DRV");
    } else {
        _sprite.setTextColor(COL_GRAY);
        _sprite.print("drv");
    }

    // Arm motor status on right
    _sprite.setCursor(110, 1);
    if (arm_arming) {
        if (blink_on) {
            _sprite.setTextColor(COL_ORANGE);
            _sprite.print("AR");
        } else {
            _sprite.setTextColor(COL_DARKGRAY);
            _sprite.print("AR");
        }
    } else if (arm_armed) {
        _sprite.setTextColor(COL_GREEN);
        _sprite.print("AR");
    } else {
        _sprite.setTextColor(COL_GRAY);
        _sprite.print("ar");
    }

    _sprite.drawLine(0, 12, SCREEN_W, 12, COL_DARKGRAY);
}

void Display::drawMotorDiagram(const MotorManager& motors) {
    // Layout: robot viewed from above
    //   FL(40)  [body]  FR(10)
    //   BL(30)  [body]  BR(20)
    //      LA(1)    RA(2)

    int body_x = 32;
    int body_y = 22;
    int body_w = 64;
    int body_h = 40;

    // Body rectangle
    _sprite.drawRect(body_x, body_y, body_w, body_h, COL_DARKGRAY);

    // Forward arrow inside body
    int cx = body_x + body_w / 2;
    int cy = body_y + body_h / 2;
    _sprite.drawLine(cx, cy + 8, cx, cy - 8, COL_DARKGRAY);
    _sprite.drawLine(cx, cy - 8, cx - 4, cy - 3, COL_DARKGRAY);
    _sprite.drawLine(cx, cy - 8, cx + 4, cy - 3, COL_DARKGRAY);

    // Motor blocks: 12x8 rectangles
    int mw = 12, mh = 8;

    // Front Left (role 3, idx 3)
    uint16_t cFL = motorColor(motors.getMotor(MotorRole::FrontLeft));
    _sprite.fillRect(body_x - mw - 2, body_y, mw, mh, cFL);
    _sprite.setTextColor(COL_TEXT);
    _sprite.setCursor(body_x - mw - 2, body_y + mh + 1);
    _sprite.setTextSize(0.8);
    _sprite.print("FL");

    // Front Right (role 0, idx 0)
    uint16_t cFR = motorColor(motors.getMotor(MotorRole::FrontRight));
    _sprite.fillRect(body_x + body_w + 2, body_y, mw, mh, cFR);
    _sprite.setCursor(body_x + body_w + 2, body_y + mh + 1);
    _sprite.print("FR");

    // Back Left (role 2, idx 2)
    uint16_t cBL = motorColor(motors.getMotor(MotorRole::BackLeft));
    _sprite.fillRect(body_x - mw - 2, body_y + body_h - mh, mw, mh, cBL);
    _sprite.setCursor(body_x - mw - 2, body_y + body_h + 1);
    _sprite.print("BL");

    // Back Right (role 1, idx 1)
    uint16_t cBR = motorColor(motors.getMotor(MotorRole::BackRight));
    _sprite.fillRect(body_x + body_w + 2, body_y + body_h - mh, mw, mh, cBR);
    _sprite.setCursor(body_x + body_w + 2, body_y + body_h + 1);
    _sprite.print("BR");

    // Arm Left (role 4)
    int arm_y = body_y + body_h + 8;
    uint16_t cLA = motorColor(motors.getMotor(MotorRole::ArmLeft));
    _sprite.fillRect(body_x + 4, arm_y, mw, mh, cLA);
    _sprite.setCursor(body_x + 4, arm_y + mh + 1);
    _sprite.print("LA");

    // Arm Right (role 5)
    uint16_t cRA = motorColor(motors.getMotor(MotorRole::ArmRight));
    _sprite.fillRect(body_x + body_w - mw - 4, arm_y, mw, mh, cRA);
    _sprite.setCursor(body_x + body_w - mw - 4, arm_y + mh + 1);
    _sprite.print("RA");

    _sprite.setTextSize(1);
}

void Display::drawChannelBars(const CrsfReceiver& crsf) {
    // Bottom section: channel input bars
    int bar_y = 88;
    int bar_h = 8;
    int bar_x = 24;
    int bar_w = 80;
    int gap = 12;

    bool link_up = crsf.isLinkUp();
    _sprite.setTextColor(link_up ? COL_TEXT : COL_RED);

    // Labels and bars for first 6 channels
    const char* labels[] = {"T", "S", "A", "B", "L", "R"};
    for (int i = 0; i < 6; i++) {
        int y = bar_y + i * gap;
        if (y + bar_h > SCREEN_H) break;

        _sprite.setCursor(1, y);
        _sprite.print(labels[i]);

        // Bar background
        _sprite.drawRect(bar_x, y, bar_w, bar_h, COL_DARKGRAY);

        if (link_up) {
            float norm = crsf.getChannelNormalized(i);
            // Map -1..+1 to bar range
            int fill_center = bar_x + bar_w / 2;
            int fill_w = static_cast<int>(fabsf(norm) * (bar_w / 2));
            int fill_x;
            if (norm >= 0) {
                fill_x = fill_center;
            } else {
                fill_x = fill_center - fill_w;
            }
            _sprite.fillRect(fill_x, y + 1, fill_w, bar_h - 2, COL_BLUE);

            // Center mark
            _sprite.drawLine(fill_center, y, fill_center, y + bar_h, COL_GRAY);
        }

        // Value text
        _sprite.setCursor(bar_x + bar_w + 3, y);
        if (link_up) {
            _sprite.printf("%4d", crsf.getChannel(i));
        } else {
            _sprite.print("----");
        }
    }
}

void Display::drawCalMode(const ArmController& arm_ctrl) {
    CalStep step = arm_ctrl.getCalStep();

    // Title
    _sprite.setTextColor(COL_ORANGE);
    _sprite.setTextSize(1);
    _sprite.setCursor(16, 2);
    _sprite.print("ARM CALIBRATION");
    _sprite.drawLine(0, 12, SCREEN_W, 12, COL_ORANGE);

    // Step indicators
    const char* step_labels[] = {"1 FORWARD", "2 CENTER", "3 BACKWARD"};
    CalStep steps[] = {CalStep::WaitForward, CalStep::WaitCenter, CalStep::WaitBackward};

    for (int i = 0; i < 3; i++) {
        int y = 20 + i * 18;
        bool is_current = (step == steps[i]);
        bool is_done = (static_cast<uint8_t>(step) > static_cast<uint8_t>(steps[i]));

        if (is_current) {
            _sprite.fillRect(0, y - 1, SCREEN_W, 14, COL_DARKGRAY);
            _sprite.setTextColor(COL_GREEN);
            _sprite.setCursor(4, y);
            _sprite.print("> ");
        } else if (is_done) {
            _sprite.setTextColor(COL_GRAY);
            _sprite.setCursor(4, y);
            _sprite.print("  ");
        } else {
            _sprite.setTextColor(COL_GRAY);
            _sprite.setCursor(4, y);
            _sprite.print("  ");
        }
        _sprite.print(step_labels[i]);

        if (is_done) {
            _sprite.setTextColor(COL_GREEN);
            _sprite.print(" OK");
        }
    }

    // Instructions at bottom
    _sprite.setTextColor(COL_TEXT);
    int instr_y = 82;

    if (step == CalStep::WaitForward) {
        _sprite.setCursor(4, instr_y);
        _sprite.print("Move arms to");
        _sprite.setCursor(4, instr_y + 12);
        _sprite.setTextColor(COL_GREEN);
        _sprite.print("FORWARD position");
        _sprite.setTextColor(COL_TEXT);
        _sprite.setCursor(4, instr_y + 28);
        _sprite.print("Press Ch11");
    } else if (step == CalStep::WaitCenter) {
        _sprite.setCursor(4, instr_y);
        _sprite.print("Move arms to");
        _sprite.setCursor(4, instr_y + 12);
        _sprite.setTextColor(COL_GREEN);
        _sprite.print("CENTER position");
        _sprite.setTextColor(COL_TEXT);
        _sprite.setCursor(4, instr_y + 28);
        _sprite.print("Press Ch11");
    } else if (step == CalStep::WaitBackward) {
        _sprite.setCursor(4, instr_y);
        _sprite.print("Move arms to");
        _sprite.setCursor(4, instr_y + 12);
        _sprite.setTextColor(COL_GREEN);
        _sprite.print("BACKWARD position");
        _sprite.setTextColor(COL_TEXT);
        _sprite.setCursor(4, instr_y + 28);
        _sprite.print("Press Ch11");
    }
}
