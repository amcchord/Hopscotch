#pragma once

#include <cmath>
#include <cstdint>
#include "config.h"

namespace balance_math {

inline bool fastTipSelected(float channel6) {
    return std::isfinite(channel6) && channel6 > 0.5f;
}

struct TipInput {
    float tilt = 0, rate = 0;
    float left = 0, right = 0; // radians relative to calibrated forward pose
    float left_velocity = 0, right_velocity = 0;
    float wheel_left = 0, wheel_right = 0;
    bool healthy = false;
};

// A finite-time arm trajectory, not a prediction of contact or free-body motion.
// Preserve the slow routine's constant-speed/exponential left/right path, then
// approach both endpoints smoothly instead of its minimum-speed tail.
class FastTipUp {
public:
    static const char* startIssue(const TipInput& in) {
        if (!valid(in)) return "Fast start blocked: motor/IMU feedback";
        if (std::fabs(in.tilt) > BALANCE_FAST_TIP_START_TILT_DEG)
            return "Fast start blocked: robot not flat";
        if (std::fabs(in.left) > BALANCE_ARM_REACHED_RAD
            || std::fabs(in.right) > BALANCE_ARM_REACHED_RAD)
            return "Fast start blocked: arms not at Forward";
        if (std::fabs(in.rate) > BALANCE_FAST_TIP_CAPTURE_RATE_DPS
            || std::fabs(in.left_velocity) > BALANCE_FAST_TIP_CAPTURE_ARM_RAD_S
            || std::fabs(in.right_velocity) > BALANCE_FAST_TIP_CAPTURE_ARM_RAD_S
            || std::fabs(in.wheel_left) > BALANCE_FAST_TIP_START_WHEEL_RAD_S
            || std::fabs(in.wheel_right) > BALANCE_FAST_TIP_START_WHEEL_RAD_S)
            return "Fast start blocked: robot moving";
        return nullptr;
    }

    bool begin(uint32_t now, const TipInput& in) {
        *this = FastTipUp{};
        _fault = startIssue(in);
        if (_fault) return false;
        _start_ms = _last_step_ms = now;
        _left = in.left;
        _right = in.right;
        _distance_l = BALANCE_ARM_TIP_LEFT - in.left;
        _distance_r = BALANCE_ARM_TIP_RIGHT - in.right;
        _distance = std::fmax(_distance_l, _distance_r);
        _active = true;
        return true;
    }

    void step(uint32_t now, float dt, const TipInput& in) {
        if (!_active || _fault) return;
        _ready = false; // readiness always describes the current measured state
        if (!valid(in)) { fail("fast_tip_feedback_or_RC"); return; }
        if (!std::isfinite(dt) || dt <= 0 || dt > BALANCE_FAST_TIP_MAX_DT_S
            || now - _last_step_ms > static_cast<uint32_t>(BALANCE_FAST_TIP_MAX_DT_S * 1000 + .5f)) {
            fail("fast_tip_control_gap"); return;
        }
        _last_step_ms = now;
        if (now - _start_ms > BALANCE_FAST_TIP_TIMEOUT_MS) {
            fail("fast_tip_timeout"); return;
        }
        if (in.tilt < -20 || in.tilt > BALANCE_FAST_TIP_MAX_TILT_DEG
            || std::fabs(in.rate) > BALANCE_FAST_TIP_MAX_RATE_DPS) {
            fail("fast_tip_motion_limit"); return;
        }

        const float next = std::fmin(_elapsed + dt, BALANCE_FAST_TIP_DURATION_S);
        const float u = next / BALANCE_FAST_TIP_DURATION_S;
        const float progress = std::fmin(1.0f, std::fmax(0.0f,
            u*u*u * (10 + u*(-15 + 6*u)))); // zero endpoint speed/acceleration
        const float remaining = _distance * (1 - progress);
        const float left = BALANCE_ARM_TIP_LEFT - pathRemaining(_distance_l, remaining);
        const float right = BALANCE_ARM_TIP_RIGHT - pathRemaining(_distance_r, remaining);
        // Pause the shared trajectory clock if either arm cannot follow. Never
        // catch up wall time or accumulate a distant target behind an obstruction.
        if (std::fabs(left - in.left) <= BALANCE_FAST_TIP_LEAD_RAD
            && std::fabs(right - in.right) <= BALANCE_FAST_TIP_LEAD_RAD) {
            _left = left; _right = right; _elapsed = next; _blocked = false;
        } else {
            if (!_blocked) { _blocked = true; _blocked_ms = now; }
            if (now - _blocked_ms >= BALANCE_FAST_TIP_STALL_MS) {
                fail("fast_tip_arm_tracking"); return;
            }
        }

        const bool quiet = _elapsed >= BALANCE_FAST_TIP_DURATION_S
            && std::fabs(in.left - BALANCE_ARM_TIP_LEFT) <= BALANCE_ARM_REACHED_RAD
            && std::fabs(in.right - BALANCE_ARM_TIP_RIGHT) <= BALANCE_ARM_REACHED_RAD
            && std::fabs(in.left_velocity) <= BALANCE_FAST_TIP_CAPTURE_ARM_RAD_S
            && std::fabs(in.right_velocity) <= BALANCE_FAST_TIP_CAPTURE_ARM_RAD_S
            && std::fabs(in.wheel_left) <= BALANCE_FAST_TIP_START_WHEEL_RAD_S
            && std::fabs(in.wheel_right) <= BALANCE_FAST_TIP_START_WHEEL_RAD_S
            && std::fabs(in.tilt - BALANCE_SETPOINT_ARMS_TIP) < BALANCE_ENGAGE_THRESHOLD_DEG
            && std::fabs(in.rate) <= BALANCE_FAST_TIP_CAPTURE_RATE_DPS;
        if (quiet) {
            if (!_quiet) { _quiet = true; _quiet_ms = now; }
            _ready = now - _quiet_ms >= BALANCE_FAST_TIP_CAPTURE_MS;
        } else { _quiet = false; }
    }

    float left() const { return _left; }
    float right() const { return _right; }
    bool ready() const { return _ready; }
    const char* fault() const { return _fault; }

private:
    static bool valid(const TipInput& in) {
        return in.healthy && std::isfinite(in.tilt) && std::isfinite(in.rate)
            && std::isfinite(in.left) && std::isfinite(in.right)
            && std::isfinite(in.left_velocity) && std::isfinite(in.right_velocity)
            && std::isfinite(in.wheel_left) && std::isfinite(in.wheel_right);
    }

    float pathRemaining(float initial, float reference_remaining) const {
        if (reference_remaining <= 0) return 0;
        // Distance-equivalent legacy clock: 0.7 * seconds. Its speed cancels
        // because this function preserves geometry, while the quintic sets time.
        constexpr float knee = 1.5f;
        const float linear = std::fmax(0.0f, _distance - knee);
        const float clock = reference_remaining >= knee
            ? _distance - reference_remaining
            : linear + knee * std::log(std::fmin(_distance, knee) / reference_remaining);
        const float other_linear = std::fmax(0.0f, initial - knee);
        return clock <= other_linear ? initial - clock
            : std::fmin(initial, knee) * std::exp(-(clock - other_linear) / knee);
    }

    void fail(const char* reason) { _fault = reason; _ready = false; }
    bool _active = false, _ready = false, _blocked = false, _quiet = false;
    uint32_t _start_ms = 0, _last_step_ms = 0, _blocked_ms = 0, _quiet_ms = 0;
    float _left = 0, _right = 0;
    float _distance_l = 0, _distance_r = 0, _distance = 0, _elapsed = 0;
    const char* _fault = nullptr;
};

// Fade a supported capture only as the measured arms return. Latch progress so
// feedback noise, a paused return or an arm rebound cannot restore old support
// bias after release. The controller retains its existing base slew limiter.
class FastTipRelease {
public:
    void reset() { _weight = 1; }
    float weight(bool returning, bool returned, float engage_fraction, float fraction) {
        if (returned) _weight = 0;
        else if (returning && std::isfinite(engage_fraction) && std::isfinite(fraction)) {
            const float progress = (engage_fraction - fraction) / BALANCE_FAST_TIP_RELEASE_FRACTION;
            _weight = std::fmin(_weight, std::fmax(0.0f, std::fmin(1.0f, 1 - progress)));
        }
        return _weight;
    }
private:
    float _weight = 1;
};

// Wheel mode setup temporarily blocks feedback processing. Return to the normal
// control loop before qualifying a start; hold the current targets throughout.
// This does not relax the fresh-feedback or physical starting-pose requirements.
class FastTipStart {
public:
    void request(uint32_t now) {
        *this = FastTipStart{};
        _started = now;
    }
    bool step(uint32_t now, const TipInput& in) {
        if (_failed) return false;
        const char* issue = FastTipUp::startIssue(in);
        if (issue) { _status = issue; _quiet = false; }
        else {
            _status = "Fast start: confirming stationary pose";
            if (!_quiet) { _quiet = true; _quiet_since = now; }
        }
        if (now - _started >= BALANCE_FAST_TIP_START_TIMEOUT_MS) {
            if (!issue) _status = "Fast start blocked: pose did not stay quiet";
            _failed = true;
            return false;
        }
        return _quiet && now - _quiet_since >= BALANCE_FAST_TIP_START_QUIET_MS;
    }
    bool failed() const { return _failed; }
    const char* status() const { return _status; }
private:
    uint32_t _started = 0, _quiet_since = 0;
    bool _quiet = false, _failed = false;
    const char* _status = "Fast start blocked: motor/IMU feedback";
};

} // namespace balance_math
