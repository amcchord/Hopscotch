#pragma once
#include "balance_math.h"

namespace balance_math {
struct PilotConfig {
    float deadband, max_velocity, max_turn, acceleration, deceleration, turn_acceleration;
    float ready_ms, stop_ms;
    float velocity_lead, accel_tau, accel_lean, lean_limit;
};

// CH2 commands average wheel velocity through the balance cascade. CH1 commands
// a differential speed (positive = left wheel faster, as in ground drive).
// This class never arms motors or bypasses balance control.
class BalancePilot {
public:
    void reset() { *this = BalancePilot{}; }
    static float stick(float value, float deadband) {
        if (!std::isfinite(value)) return 0;
        value = clamp(value, -1, 1);
        return std::fabs(value) <= deadband ? 0
             : std::copysign((std::fabs(value)-deadband)/(1-deadband), value);
    }
    void update(bool allowed, bool calm, float throttle, float steering, float dt,
                const PilotConfig& c, float measured_velocity = NAN) {
        const bool timing_ok = std::isfinite(dt) && dt > 0 && dt <= .1f;
        allowed = allowed && timing_ok && std::isfinite(throttle) && std::isfinite(steering);
        allowed = allowed && (c.velocity_lead <= 0 || std::isfinite(measured_velocity));
        _forward_stick = stick(throttle, c.deadband);
        _turn_stick = stick(steering, c.deadband);
        const bool neutral = _forward_stick == 0 && _turn_stick == 0;
        if (!allowed) {
            _ready = false;
            _neutral_ms = 0;
        } else if (!_ready) {
            _neutral_ms = neutral && calm ? _neutral_ms + dt * 1000 : 0;
            _ready = _neutral_ms + .001f >= c.ready_ms;
        }
        float requested_velocity = _ready ? _forward_stick * c.max_velocity : 0;
        const float requested_turn = _ready ? _turn_stick * c.max_turn : 0;
        if (requested_velocity != 0 || requested_turn != 0) _moving = true;
        // A late tick cannot create a large command jump. Input loss requests
        // a bounded stop and requires centered sticks before accepting more input.
        const float step_dt = timing_ok ? dt : .02f;
        const bool braking = _velocity * requested_velocity < 0
                          || std::fabs(requested_velocity) < std::fabs(_velocity);
        _lead_limited = false;
        if (!braking && c.velocity_lead > 0 && std::isfinite(measured_velocity)) {
            // Limit only further acceleration, never delay a center/stop request
            // or jump the reference backward when wheel feedback changes.
            float bounded = requested_velocity;
            if (requested_velocity > 0)
                bounded = std::fmin(requested_velocity, std::fmax(_velocity, measured_velocity+c.velocity_lead));
            else if (requested_velocity < 0)
                bounded = std::fmax(requested_velocity, std::fmin(_velocity, measured_velocity-c.velocity_lead));
            _lead_limited = bounded != requested_velocity;
            requested_velocity = bounded;
        }
        const float previous_velocity = _velocity;
        _velocity = approach(_velocity, requested_velocity,
                             braking ? c.deceleration : c.acceleration, step_dt);
        const float acceleration = (_velocity-previous_velocity)/step_dt;
        const float alpha = c.accel_tau > 0 ? step_dt/(c.accel_tau+step_dt) : 1;
        _acceleration += alpha * (acceleration-_acceleration);
        _lean_ff = clamp(-c.accel_lean*_acceleration, -c.lean_limit, c.lean_limit);
        _turn = approach(_turn, requested_turn, c.turn_acceleration, step_dt);
        const bool was_turning = _turning;
        _turning = _turn != 0;
        _capture_heading = _turning || was_turning;
        if (_moving) {
            _stop_ms = timing_ok && calm && _velocity == 0 && _turn == 0
                     && requested_velocity == 0 && requested_turn == 0
                     ? _stop_ms + dt * 1000 : 0;
            if (_stop_ms + .001f >= c.stop_ms) _moving = false;
        } else {
            _stop_ms = 0;
        }
    }
    bool ready() const { return _ready; }
    bool moving() const { return _moving; } // includes braking until calm
    bool turning() const { return _turning; }
    bool captureHeading() const { return _capture_heading; }
    float velocity() const { return _velocity; }
    float turn() const { return _turn; }
    float forwardStick() const { return _forward_stick; }
    float turnStick() const { return _turn_stick; }
    float leanFeedforward() const { return _moving ? _lean_ff : 0; }
    bool leadLimited() const { return _lead_limited; }
    float velocityCorrection(float error, float hold_low, float high, float knee,
                             float drive_low, bool ramp_complete) const {
        const float low = _moving && ramp_complete ? drive_low : hold_low;
        if (std::fabs(error) <= knee || !ramp_complete) return low * error;
        return std::copysign(low * knee + high * (std::fabs(error)-knee), error);
    }
private:
    static float approach(float value, float target, float rate, float dt) {
        return value + clamp(target-value, -rate*dt, rate*dt);
    }
    float _velocity=0, _turn=0, _neutral_ms=0, _stop_ms=0;
    float _forward_stick=0, _turn_stick=0;
    float _acceleration=0, _lean_ff=0;
    bool _lead_limited=false;
    bool _ready=false, _moving=false, _turning=false, _capture_heading=false;
};
}  // namespace balance_math
