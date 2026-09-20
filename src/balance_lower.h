#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace balance_math {

// Experimental forward fall and arm catch. Angles are body tilt (larger is
// backward); arm positions are offsets from calibrated flat Forward. Negative
// center-axis travel places the arms physically forward when standing.
// Preserve exported v1 phase numbers; 9 now initiates a fall, 10 awaits catch.
enum class LowerPhase : uint8_t {
    Idle, Stopping, Reaching, Descending, GroundHold, Retracting, Canceling,
    Complete, Fault, Committing, Catching
};

struct LowerInput {
    float tilt = 0, rate = 0, error = 0;
    float wheel_left = 0, wheel_right = 0;
    float arm_left = 0, arm_right = 0;
    float velocity_left = 0, velocity_right = 0;
    float torque_left = 0, torque_right = 0;
    bool healthy = false;
};

struct LowerConfig {
    float prepare_rad = 1.30f, prepare_speed = .25f;
    float catch_rad = 2.40f, catch_speed = 1.5f;
    float lower_speed = .16f, retract_speed = .30f;
    float catch_yield = .06f, catch_yield_speed = .12f;
    float max_target_lead = .12f;
    float contact_torque = .40f;
    float launch_accel = 4.0f, launch_speed = 2.0f; // rear rad/s^2 and rad/s
    float wheel_stop_accel = .75f;
    float forward_drop = 1.f, forward_rate = 2.f, launch_rate = 6.f;
    float calm_wheel = .65f, calm_rate = 4, calm_error = 2;
    float descent_rate = 12, abort_rate = 65, abort_wheel = 3;
    float flat_angle = 5, flat_rate = 5;
    uint32_t calm_ms = 500, prepare_ms = 200, contact_ms = 80, flat_ms = 600;
    uint32_t stop_timeout_ms = 14000, prepare_timeout_ms = 16000;
    uint32_t launch_timeout_ms = 1800, catch_timeout_ms = 2500;
    uint32_t descent_timeout_ms = 20000, retract_timeout_ms = 6000;
    uint32_t progress_timeout_ms = 3000;
};

struct LowerWheelOutput { float value; bool fault; };
inline LowerWheelOutput lowerWheelCommand(float command, uint32_t owner_age_ms) {
    if (!std::isfinite(command) || owner_age_ms > 100 || std::fabs(command) > 2.0f)
        return {0, true};
    return {command, false};
}

class BalanceLower {
public:
    LowerPhase phase() const { return _phase; }
    bool active() const { return _phase != LowerPhase::Idle && _phase != LowerPhase::Complete && _phase != LowerPhase::Fault; }
    bool committed() const { return _committed; } // owns wheels, even before catch
    bool supported() const { return _supported; } // catch observed, not just commanded
    bool overridesArms() const { return active() && _phase != LowerPhase::Stopping; }
    float left() const { return _left; }
    float right() const { return _right; }
    float wheelCommand() const { return _wheel_command; }
    float armSpeed() const {
        const LowerConfig c;
        return _phase == LowerPhase::Committing || _phase == LowerPhase::Catching
            ? c.catch_speed : std::max(c.prepare_speed, c.retract_speed);
    }
    const char* reason() const { return _reason; }
    void reset() { *this = BalanceLower{}; }

    bool request(uint32_t now, const LowerInput& in, float center_left, float center_right) {
        if (active() || !valid(in) || !in.healthy
            || !std::isfinite(center_left) || !std::isfinite(center_right)
            || center_left * center_right >= 0
            || std::fabs(center_left) < 1 || std::fabs(center_left) > 2.5f
            || std::fabs(center_right) < 1 || std::fabs(center_right) > 2.5f
            || in.tilt < 65 || in.tilt > 105) return false;
        reset();
        _sign_left = std::copysign(1.f, center_left);
        _sign_right = std::copysign(1.f, center_right);
        _left = in.arm_left; _right = in.arm_right;
        enter(LowerPhase::Stopping, now);
        return true;
    }

    void step(uint32_t now, float dt, const LowerInput& in, const LowerConfig& c = {}) {
        if (!active()) return;
        if (!valid(in) || !in.healthy || !std::isfinite(dt) || dt <= 0 || dt > .1f) {
            fail("lower_invalid_feedback"); return;
        }
        if (in.tilt < -12 || in.tilt > 115 || std::fabs(in.rate) > c.abort_rate
            || (_committed && (std::fabs(in.wheel_left) > c.abort_wheel
                              || std::fabs(in.wheel_right) > c.abort_wheel))) {
            fail("lower_motion_limit"); return;
        }
        const bool calm = std::fabs(in.rate) <= c.calm_rate
            && std::fabs(in.error) <= c.calm_error
            && std::fabs(in.wheel_left) <= c.calm_wheel
            && std::fabs(in.wheel_right) <= c.calm_wheel;
        const bool flat = std::fabs(in.tilt) <= c.flat_angle
            && std::fabs(in.rate) <= c.flat_rate
            && std::fabs(in.wheel_left) <= c.calm_wheel
            && std::fabs(in.wheel_right) <= c.calm_wheel;
        const uint32_t elapsed = now - _phase_ms;
        switch (_phase) {
        case LowerPhase::Stopping:
            _left = in.arm_left; _right = in.arm_right;
            if (confirmed(calm, dt, c.calm_ms)) enter(LowerPhase::Reaching, now);
            else if (elapsed > c.stop_timeout_ms) finish("lower_stop_timeout");
            break;
        case LowerPhase::Reaching: {
            // Prepare interception before intentionally departing upright
            // control. Do not continue all the way to the floor while the
            // balance loop counteracts the arms (failed v1 physical trial).
            if (std::fabs(in.rate) > 15 || std::fabs(in.error) > 5
                || std::fabs(in.wheel_left) > 3 || std::fabs(in.wheel_right) > 3) {
                fail("lower_prepare_disturbed"); break;
            }
            if (calm) {
                _left = approach(_left, -_sign_left*c.prepare_rad, in.arm_left, c.prepare_speed*dt, c.max_target_lead);
                _right = approach(_right, -_sign_right*c.prepare_rad, in.arm_right, c.prepare_speed*dt, c.max_target_lead);
            }
            const bool ready = std::fabs(in.arm_left + _sign_left*c.prepare_rad) < .04f
                && std::fabs(in.arm_right + _sign_right*c.prepare_rad) < .04f
                && std::fabs(in.velocity_left) < .12f && std::fabs(in.velocity_right) < .12f;
            if (confirmed(ready && calm, dt, c.prepare_ms)) {
                _launch_tilt = in.tilt;
                _wheel_command = (in.wheel_left + in.wheel_right)*.5f;
                _committed = true; _committed_ms = now;
                _peak_fall_rate = 0;
                enter(LowerPhase::Committing, now);
            } else if (elapsed > c.prepare_timeout_ms) fail("lower_prepare_timeout");
            break;
        }
        case LowerPhase::Committing:
        case LowerPhase::Catching: {
            // Upright PD/position PI no longer own the wheels. A bounded rear
            // acceleration starts the forward fall; arm motion simultaneously
            // positions the catch. Stop accelerating once forward motion is
            // measured. Preserve that small wheel speed until support catches
            // the body instead of abruptly braking it back upright.
            if (in.tilt > _launch_tilt + 2 || in.rate > 12) {
                fail("lower_wrong_direction"); break;
            }
            _peak_fall_rate = std::min(_peak_fall_rate, in.rate);
            if (!_catch_started && in.rate <= -1.f && in.tilt < _launch_tilt) {
                _catch_started = true; _catch_start_ms = now;
            }
            if (_phase == LowerPhase::Committing) {
                if (in.rate > -c.launch_rate)
                    _wheel_command = toward(_wheel_command, -c.launch_speed, c.launch_accel*dt);
                if (in.tilt <= _launch_tilt-c.forward_drop && in.rate <= -c.forward_rate
                    && in.arm_left*_sign_left <= -1.8f && in.arm_right*_sign_right <= -1.8f)
                    enter(LowerPhase::Catching, now);
                else if (elapsed > c.launch_timeout_ms) { fail("lower_no_forward_fall"); break; }
            }
            const bool seek_load = _catch_started && now - _catch_start_ms >= 100; // exclude the initial arm acceleration impulse
            const bool left_load = seek_load && loaded(in.arm_left, in.velocity_left, in.torque_left, _sign_left, c);
            const bool right_load = seek_load && loaded(in.arm_right, in.velocity_right, in.torque_right, _sign_right, c);
            // Remove preload independently when an arm is loaded. Never drive
            // a stopped arm farther through the floor to seek a tracking stall.
            if (!_left_touched && left_load) { _left = _touch_left = in.arm_left; _left_touched = true; }
            if (!_right_touched && right_load) { _right = _touch_right = in.arm_right; _right_touched = true; }
            if (_catch_started && !_left_touched) _left = approach(_left, -_sign_left*c.catch_rad, in.arm_left, c.catch_speed*dt, c.max_target_lead);
            if (_catch_started && !_right_touched) _right = approach(_right, -_sign_right*c.catch_rad, in.arm_right, c.catch_speed*dt, c.max_target_lead);
            // Yield a bounded few degrees after both impacts so an early
            // catch can accept forward weight instead of jacking the body up.
            // Stop yielding during fast descent; this is not a timed release.
            if (_left_touched && _right_touched && in.rate >= -c.descent_rate) {
                _left = approach(_left, _touch_left + _sign_left*c.catch_yield,
                                 in.arm_left, c.catch_yield_speed*dt, c.max_target_lead);
                _right = approach(_right, _touch_right + _sign_right*c.catch_yield,
                                  in.arm_right, c.catch_yield_speed*dt, c.max_target_lead);
            }
            const bool caught = _left_touched && _right_touched
                && std::fabs(in.torque_left) >= c.contact_torque*.5f
                && std::fabs(in.torque_right) >= c.contact_torque*.5f
                && in.tilt <= _launch_tilt-c.forward_drop*.5f
                && std::fabs(in.velocity_left) <= .20f && std::fabs(in.velocity_right) <= .20f
                && in.rate >= -c.descent_rate && in.rate <= c.calm_rate
                && _peak_fall_rate < -c.forward_rate
                && in.rate >= _peak_fall_rate + 1.f;
            if (confirmed(caught, dt, c.contact_ms)) {
                _supported = true;
                _left = in.arm_left; _right = in.arm_right;
                _progress_tilt = in.tilt; _progress_ms = now;
                enter(LowerPhase::Descending, now);
            } else if (now - _committed_ms > c.catch_timeout_ms || in.tilt < 40) {
                fail("lower_missed_catch");
            }
            break;
        }
        case LowerPhase::Descending:
            _wheel_command = toward(_wheel_command, 0, c.wheel_stop_accel*dt);
            if (flat) { enter(LowerPhase::GroundHold, now); break; }
            _support_lost_ms = in.tilt > 15 && in.rate < -25
                && (std::fabs(in.torque_left) < c.contact_torque*.5f || std::fabs(in.torque_right) < c.contact_torque*.5f)
                ? _support_lost_ms + dt*1000 : 0;
            if (_support_lost_ms >= 100) { fail("lower_support_lost"); break; }
            if (in.tilt < _progress_tilt - 2) { _progress_tilt = in.tilt; _progress_ms = now; }
            if (now - _committed_ms > c.descent_timeout_ms || now - _progress_ms > c.progress_timeout_ms) {
                fail("lower_descent_timeout"); break;
            }
            if (in.rate >= -c.descent_rate && in.rate <= c.calm_rate) {
                _left = approach(_left, 0, in.arm_left, c.lower_speed*dt, c.max_target_lead);
                _right = approach(_right, 0, in.arm_right, c.lower_speed*dt, c.max_target_lead);
            }
            break;
        case LowerPhase::GroundHold:
            _wheel_command = toward(_wheel_command, 0, c.wheel_stop_accel*dt);
            if (now - _committed_ms > c.descent_timeout_ms) { fail("lower_descent_timeout"); break; }
            if (!flat) { enter(LowerPhase::Descending, now); _progress_ms = now; _progress_tilt = in.tilt; break; }
            if (confirmed(flat, dt, c.flat_ms)) enter(LowerPhase::Retracting, now);
            break;
        case LowerPhase::Retracting:
            _wheel_command = 0;
            if (std::fabs(in.tilt) > c.flat_angle + 4 || std::fabs(in.rate) > 20) {
                fail("lower_ground_unstable"); break;
            }
            _left = approach(_left, 0, in.arm_left, c.retract_speed*dt, c.max_target_lead);
            _right = approach(_right, 0, in.arm_right, c.retract_speed*dt, c.max_target_lead);
            if (at(_left, 0) && at(_right, 0) && std::fabs(in.arm_left) < .06f
                && std::fabs(in.arm_right) < .06f && flat) finish("lower_complete");
            else if (elapsed > c.retract_timeout_ms) fail("lower_retract_timeout");
            break;
        default: break;
        }
    }

private:
    volatile LowerPhase _phase = LowerPhase::Idle;
    bool _committed = false, _supported = false, _left_touched = false, _right_touched = false, _catch_started = false;
    float _left = 0, _right = 0, _sign_left = 0, _sign_right = 0, _wheel_command = 0;
    float _confirm_ms = 0, _progress_tilt = 0, _support_lost_ms = 0;
    float _launch_tilt = 0, _peak_fall_rate = 0, _touch_left = 0, _touch_right = 0;
    uint32_t _phase_ms = 0, _progress_ms = 0, _committed_ms = 0, _catch_start_ms = 0;
    const char* _reason = "lower_requested";
    static bool at(float a, float b) { return std::fabs(a-b) < .001f; }
    static bool valid(const LowerInput& i) {
        const float values[] = {i.tilt,i.rate,i.error,i.wheel_left,i.wheel_right,
            i.arm_left,i.arm_right,i.velocity_left,i.velocity_right,i.torque_left,i.torque_right};
        for (float v : values) if (!std::isfinite(v)) return false;
        return true;
    }
    static float toward(float value, float goal, float step) {
        return value + std::max(-step, std::min(step, goal-value));
    }
    static float approach(float value, float goal, float measured, float step, float lead) {
        const float next = toward(value, goal, step);
        return std::max(measured-lead, std::min(measured+lead, next));
    }
    static bool loaded(float measured, float /*velocity*/, float torque, float sign, const LowerConfig& c) {
        return measured*sign < -1.5f && std::fabs(torque) >= c.contact_torque;
    }
    bool confirmed(bool condition, float dt, uint32_t duration) {
        _confirm_ms = condition ? _confirm_ms+dt*1000 : 0;
        return _confirm_ms+.01f >= duration;
    }
    void enter(LowerPhase phase, uint32_t now) { _phase=phase; _phase_ms=now; _confirm_ms=0; }
    void fail(const char* reason) { _reason=reason; _phase=LowerPhase::Fault; _wheel_command=0; }
    void finish(const char* reason) { _reason=reason; _phase=LowerPhase::Complete; _wheel_command=0; }
};
} // namespace balance_math
