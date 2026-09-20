#pragma once
#include "balance_math.h"
namespace balance_math {
struct DriveStopConfig {
    float speed, angle, rate, confirm_ms;
};
// A centered stop may hand back to stationary PD once slow and upright,
// before the longer hold-position calm gate finishes. New throttle resets it.
class DriveStopCapture {
public:
    void reset() { _captured=false; _confirm_ms=0; }
    bool update(bool moving, bool neutral, bool target_zero, float speed,
                float error, float rate, float dt, const DriveStopConfig& c) {
        if (!moving || !neutral || !std::isfinite(speed) || !std::isfinite(error)
            || !std::isfinite(rate) || !std::isfinite(dt) || dt<=0 || dt>.1f) {
            reset(); return false;
        }
        if (_captured) return true;
        const bool slow=target_zero && std::fabs(speed)<c.speed
            && std::fabs(error)<c.angle && std::fabs(rate)<c.rate;
        _confirm_ms=slow ? _confirm_ms+dt*1000 : 0;
        _captured=_confirm_ms+.001f>=c.confirm_ms;
        return _captured;
    }
    bool captured() const { return _captured; }
private:
    bool _captured=false;
    float _confirm_ms=0;
};

}
