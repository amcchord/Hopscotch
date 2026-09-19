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
struct RunawayConfig {
    float minimum_speed, unconditional_speed, minimum_acceleration, acceleration_tau;
    uint32_t confirm_ms;
};
// Signed confirmation rejects spikes and alternating noise. Steady high speed
// also qualifies, even when its acceleration has stopped increasing.
class RunawayDetector {
public:
    void reset() { *this = RunawayDetector{}; }
    float update(bool eligible, float velocity, float dt, const RunawayConfig& c) {
        if (!std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0 || dt > .1f) {
            reset();
            return 0;
        }
        const float acceleration = _initialized ? (velocity - _previous_velocity) / dt : 0;
        _previous_velocity = velocity;
        _initialized = true;
        _acceleration += clamp(dt / c.acceleration_tau, 0, 1) * (acceleration - _acceleration);
        if (_triggered) return 0;
        const float direction = velocity > 0 ? 1.0f : -1.0f;
        const float speed = std::fabs(velocity);
        const bool growing = direction * _acceleration >= c.minimum_acceleration;
        const bool candidate = eligible && speed >= c.minimum_speed
                            && (growing || speed >= c.unconditional_speed);
        if (!candidate) { _qualifying_ms = 0; _direction = 0; return 0; }
        if (direction != _direction) { _qualifying_ms = 0; _direction = direction; }
        _qualifying_ms += dt * 1000;
        if (_qualifying_ms + .001f < c.confirm_ms) return 0;
        _triggered = true;
        return direction;
    }
private:
    float _previous_velocity = 0, _acceleration = 0, _qualifying_ms = 0, _direction = 0;
    bool _initialized = false, _triggered = false;
};
class StartupRecovery {
public:
    void reset() { *this = StartupRecovery{}; }
    bool start(uint32_t now) {
        if (_triggered) return false;
        _triggered = _active = true;
        _start_ms = now;
        return true;
    }
    bool triggered() const { return _triggered; }
    bool active() const { return _active; }
    bool boosting(uint32_t now, bool ramp_complete, uint32_t duration_ms) const {
        return _active && !ramp_complete && uint32_t(now - _start_ms) < duration_ms;
    }
    // Called only with fresh motion feedback and a completed arm/base ramp.
    // A new stand-up must explicitly reset the one-shot state.
    bool settle(bool calm, float dt, float required_ms) {
        if (!_active) return false;
        _calm_ms = calm && std::isfinite(dt) && dt > 0 && dt <= .1f
                 ? _calm_ms + dt * 1000 : 0;
        if (_calm_ms + .001f < required_ms) return false;
        _active = false;
        return true;
    }
private:
    bool _triggered = false, _active = false;
    uint32_t _start_ms = 0;
    float _calm_ms = 0;
};
struct RecoilConfig {
    float enter_speed, exit_speed, confirm_ms, blend_ms, multiplier;
};
// Release the transient correction only after confirmed motion against it.
// Direction changes, stale feedback and leaving recovery reset confirmation.
// Speed hysteresis and a gain ramp avoid chasing the first near-zero crossing.
class RecoilUnwind {
public:
    void reset() { *this = RecoilUnwind{}; }
    float update(bool eligible, float velocity, float integral, float dt,
                 const RecoilConfig& c) {
        if (!eligible || !std::isfinite(velocity) || !std::isfinite(integral)
            || !std::isfinite(dt) || dt <= 0 || dt > .1f || velocity * integral >= 0) {
            reset();
            return 1;
        }
        const float direction = velocity > 0 ? 1.0f : -1.0f;
        if (direction != _direction) { reset(); _direction = direction; }
        const float speed = std::fabs(velocity);
        if (_confirmed && speed <= c.exit_speed) {
            _confirmed = false;
            _qualifying_ms = 0;
        }
        if (!_confirmed) {
            _qualifying_ms = speed >= c.enter_speed ? _qualifying_ms + dt * 1000 : 0;
            _confirmed = _qualifying_ms + .001f >= c.confirm_ms;
        }
        _blend = clamp(_blend + (_confirmed ? 1 : -1) * dt * 1000 / c.blend_ms, 0, 1);
        return 1 + _blend * (c.multiplier - 1);
    }
private:
    float _direction = 0, _qualifying_ms = 0, _blend = 0;
    bool _confirmed = false;
};
struct IntegralUpdate { float value; bool limited; };
// Use the existing equilibrium integrator. Rate/angle bounds and conditional
// integration prevent a saturated correction accumulating hidden windup;
// reversing velocity can always unwind it. Invalid/stale inputs hold it.
inline IntegralUpdate recoveryIntegral(float current, float velocity_error, float ki,
                                      float dt, float offset, float limit, float rate,
                                      bool feedback_fresh, float unwind_multiplier = 1) {
    if (!feedback_fresh || !std::isfinite(velocity_error) || !std::isfinite(dt)
        || dt <= 0 || dt > .1f) return {current, true};
    float desired_rate = ki * velocity_error;
    if (unwind_multiplier > 1 && desired_rate * current < 0) {
        // Extra learning may remove existing correction, never accumulate a
        // correction of the opposite sign. Ordinary integration can cross zero.
        const float ceiling = std::fmax(std::fabs(current) / dt, std::fabs(desired_rate));
        desired_rate = std::copysign(std::fmin(std::fabs(desired_rate) * unwind_multiplier,
                                             ceiling), desired_rate);
    }
    const float bounded_rate = clamp(desired_rate, -rate, rate);
    const float change = bounded_rate * dt;
    if (std::fabs(offset) >= limit - .05f && change * offset >= 0)
        return {current, true};
    const float desired = current + change;
    const float result = clamp(desired, -limit, limit);
    return {result, result != desired || bounded_rate != desired_rate};
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
