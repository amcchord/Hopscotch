#pragma once
#include "balance_math.h"

namespace balance_math {
struct PilotConfig {
    float deadband, max_velocity, max_turn, acceleration, deceleration, turn_acceleration;
    float ready_ms, stop_ms;
    float arm_gain, arm_limit, arm_tau;
    float fast_deceleration, brake_start_speed, brake_full_speed;
    float arm_brake_limit; // zero retains arm_limit for older offline bridges
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
                const PilotConfig& c) {
        const bool timing_ok = std::isfinite(dt) && dt > 0 && dt <= .1f;
        allowed = allowed && timing_ok && std::isfinite(throttle) && std::isfinite(steering);
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
        const float requested_velocity = _ready ? _forward_stick * c.max_velocity : 0;
        const float requested_turn = _ready ? _turn_stick * c.max_turn : 0;
        if (requested_velocity != 0 || requested_turn != 0) _moving = true;
        // A late tick cannot create a large command jump. Input loss requests
        // a bounded stop and requires centered sticks before accepting more input.
        const float step_dt = timing_ok ? dt : .02f;
        const bool braking = _velocity * requested_velocity < 0
                          || std::fabs(requested_velocity) < std::fabs(_velocity);
        float brake_rate = c.deceleration;
        if (c.fast_deceleration > c.deceleration && c.brake_full_speed > c.brake_start_speed) {
            brake_rate += (c.fast_deceleration-c.deceleration)
                * clamp((std::fabs(_velocity)-c.brake_start_speed)
                        / (c.brake_full_speed-c.brake_start_speed),0,1);
        }
        _fast_braking = braking && brake_rate > c.deceleration;
        const float previous_velocity = _velocity;
        _velocity = approach(_velocity, requested_velocity,
                             braking ? brake_rate : c.acceleration, step_dt);
        // Faster reference braking must not make a larger shoulder swing than
        // the smooth v4 release. Keep the tested braking-arm amplitude.
        const float arm_limit = _fast_braking && c.arm_brake_limit > 0
                              ? std::fmin(c.arm_limit,c.arm_brake_limit) : c.arm_limit;
        const float arm_target = clamp(-c.arm_gain*(_velocity-previous_velocity)/step_dt,
                                       -arm_limit,arm_limit);
        _arm += step_dt/(c.arm_tau+step_dt)*(arm_target-_arm);
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
    bool fastBraking() const { return _fast_braking; }
    bool turning() const { return _turning; }
    bool captureHeading() const { return _capture_heading; }
    float velocity() const { return _velocity; }
    float turn() const { return _turn; }
    float forwardStick() const { return _forward_stick; }
    float turnStick() const { return _turn_stick; }
    float armAssist() const { return _moving ? _arm : 0; }
    float plannedArm(uint8_t recovery_stage) const {
        return recovery_stage == 1 || recovery_stage == 2 ? 0 : armAssist();
    }
    float yawCorrection(float hold_correction, float hold_limit) const {
        // Intentional turns have their own limit; the smaller heading-hold
        // clamp must not silently cap the requested steering range.
        return _turning ? -_turn : clamp(hold_correction, -hold_limit, hold_limit);
    }
    static float armVelocityError(float speed, float target) {
        return speed-clamp(speed,std::fmin(0,target),std::fmax(0,target));
    }
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
    float _arm=0;
    bool _fast_braking=false;
    bool _ready=false, _moving=false, _turning=false, _capture_heading=false;
};
}  // namespace balance_math
