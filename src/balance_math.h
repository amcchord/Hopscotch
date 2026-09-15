#pragma once
#include <cmath>
#include <cstdint>

// Hardware-independent controller logic shared with host regression tests.
namespace balance_math {
inline float clamp(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
// Bound the measured total curve correction, then express it relative to the
// trim already included in the base. Bounding the relative correction instead
// makes the same physical capture depend on a previous run's stored trim.
inline float captureCurveShift(float tilt, float scheduled, float stored_trim,
                               float absolute_limit) {
    return clamp(tilt - scheduled, -absolute_limit, absolute_limit) - stored_trim;
}
struct WheelCommands { float left, right, yaw; };
inline WheelCommands mix(float common, float yaw, float limit) {
    common = clamp(common, -limit, limit);
    // Balance has first claim on authority. Yaw uses the remaining headroom
    // on BOTH wheels, including during the reduced-authority dead-man.
    const float headroom = limit - std::fabs(common);
    yaw = clamp(yaw, -headroom, headroom);
    return {common - yaw, common + yaw, yaw};
}
inline bool fresh(uint32_t now, uint32_t sample, uint32_t max_age) {
    return uint32_t(now - sample) <= max_age;  // rollover-safe
}
inline bool finiteImu(float ax, float ay, float az, float gx, float gy, float gz) {
    return std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az)
        && std::isfinite(gx) && std::isfinite(gy) && std::isfinite(gz)
        && ax * ax + ay * ay + az * az > 0.01f;
}
}  // namespace balance_math
