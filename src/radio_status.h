#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>

// Experimental, project-private extended CRSF envelope. Not a TBS allocation.
// Destination handset, origin FC; EdgeTX forwards unknown types to Lua.
// See docs/RADIO_TELEMETRY.md for byte layout and compatibility constraints.
namespace radio_status {
constexpr uint8_t FRAME_TYPE = 0x7E;
constexpr size_t STATUS_SIZE = 48;
constexpr size_t DETAIL_SIZE = 60;
enum Flag : uint16_t {
    DriveArmed = 1, ArmsArmed = 2, DriveArming = 4, ArmsArming = 8,
    RcLinked = 16, ArmsMoving = 32, RearmRequired = 64, SavingLog = 128,
    ImuFresh = 256, Simulation = 512, VoltageFresh = 1024, CurrentFresh = 2048,
};
struct Snapshot {
    uint16_t sequence = 0, flags = 0;
    uint8_t mode = 0, phase = 0;
    uint16_t motion = 0;
    uint8_t online = 0, faults = 0, enabled = 0, max_temp = 255;
    int16_t tilt = INT16_MIN, error = INT16_MIN; // centidegrees, MIN = unknown
    uint16_t voltage = 0, motor_current = 0, inner_fault = 0;
    uint16_t capabilities = 0x0007; // status, run detail, motion IDs (not commands)
    uint8_t calibration = 0, progress = 255; // progress unavailable in current controller
    const char* label = "IDLE";
};
inline void u16(uint8_t* p, uint16_t value) { p[0] = value >> 8; p[1] = value; }
inline uint16_t unsignedScaled(float value, float scale) {
    if (!std::isfinite(value) || value <= 0) return 0;
    if (value * scale >= 65535) return 65535;
    return static_cast<uint16_t>(value * scale + 0.5f);
}
inline int16_t angle(float value) {
    if (!std::isfinite(value)) return INT16_MIN;
    if (value <= -327.67f) return -32767;
    if (value >= 327.67f) return 32767;
    return static_cast<int16_t>(std::round(value * 100.0f));
}
inline void prefix(uint8_t* out, uint8_t kind, uint16_t sequence) {
    out[0] = 0xEA; out[1] = 0xC8; out[2] = 'H'; out[3] = 'S';
    out[4] = 1; out[5] = kind; u16(out + 6, sequence);
}
inline void text(uint8_t* out, size_t length, const char* value) {
    bool end = !value;
    for (size_t i = 0; i < length; ++i) {
        const char c = end ? 0 : *value++;
        if (!c) end = true;
        out[i] = c >= 32 && c <= 126 ? c : 0;
    }
}
inline void encode(const Snapshot& s, uint8_t (&out)[STATUS_SIZE]) {
    prefix(out, 1, s.sequence);
    u16(out + 8, s.flags); out[10] = s.mode; out[11] = s.phase;
    u16(out + 12, s.motion); out[14] = s.online; out[15] = s.faults;
    out[16] = s.enabled; out[17] = s.max_temp;
    u16(out + 18, static_cast<uint16_t>(s.tilt));
    u16(out + 20, static_cast<uint16_t>(s.error));
    u16(out + 22, s.voltage); u16(out + 24, s.motor_current);
    u16(out + 26, s.inner_fault); u16(out + 28, s.capabilities);
    out[30] = s.calibration; out[31] = s.progress;
    text(out + 32, 16, s.label);
}
inline void encodeDetail(uint16_t sequence, uint32_t end_ms, const char* reason,
                         uint8_t (&out)[DETAIL_SIZE]) {
    prefix(out, 2, sequence);
    u16(out + 8, end_ms >> 16); u16(out + 10, end_ms);
    text(out + 12, 48, reason);
}
} // namespace radio_status
