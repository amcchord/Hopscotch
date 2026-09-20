#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace balance_math {

// Experimental contact-assisted stand-down. Angles/torques are measured;
// targets are offsets from the calibrated flat Forward reference. Positive
// center-axis travel points backward when standing; negative reaches forward.
// This policy is shared by firmware, native regressions and the contact model.
enum class LowerPhase : uint8_t {
    Idle, Stopping, Reaching, Descending, GroundHold, Retracting, Canceling, Complete, Fault, Loading
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
    float reach_speed = .25f;         // arm rad/s; deliberately slow approach
    float lower_speed = .16f;         // arm rad/s (~9 deg/s body descent)
    float retract_speed = .30f;
    float max_forward_fraction = 1.40f; // relative to calibrated 90-degree center
    float max_forward_rad = 2.60f;      // absolute cap if calibration magnitude is large
    float max_target_lead = .12f;      // rad; do not build a large stalled target
    float contact_torque = .40f;       // Nm, BOTH arms plus tracking stall required
    float contact_error = .06f;        // rad of resisted forward target
    float contact_velocity = .12f;     // rad/s
    float load_lean = -3.f, load_travel = .035f, load_speed = .06f;
    float calm_wheel = .65f, calm_rate = 4, calm_error = 2;
    float descent_rate = 12;           // stop retracting above this body rate
    float abort_rate = 65, abort_wheel = 3;
    float flat_angle = 5, flat_rate = 5;
    uint32_t calm_ms = 500, contact_ms = 240, flat_ms = 600;
    uint32_t stop_timeout_ms = 14000, reach_timeout_ms = 18000;
    uint32_t descent_timeout_ms = 20000, retract_timeout_ms = 6000;
    uint32_t progress_timeout_ms = 3000;
};

class BalanceLower {
public:
    LowerPhase phase() const { return _phase; }
    bool active() const { return _phase != LowerPhase::Idle && _phase != LowerPhase::Complete && _phase != LowerPhase::Fault; }
    bool supported() const { return _committed; }
    bool overridesArms() const { return active() && _phase != LowerPhase::Stopping; }
    float left() const { return _left; }
    float right() const { return _right; }
    float leanOffset() const { return _phase == LowerPhase::Loading ? LowerConfig{}.load_lean : 0; }
    const char* reason() const { return _reason; }
    void reset() { *this = BalanceLower{}; }

    bool request(uint32_t now, const LowerInput& in, float center_left, float center_right) {
        if (active() || !valid(in) || !in.healthy
            || !std::isfinite(center_left) || !std::isfinite(center_right)
            || center_left * center_right >= 0
            || std::fabs(center_left) < 1 || std::fabs(center_left) > 2.5f
            || std::fabs(center_right) < 1 || std::fabs(center_right) > 2.5f
            || in.tilt < 65 || in.tilt > 110) return false;
        reset();
        _center_left = center_left;
        _center_right = center_right;
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
            const float left_limit = -std::copysign(std::min(c.max_forward_rad, c.max_forward_fraction*std::fabs(_center_left)), _center_left);
            const float right_limit = -std::copysign(std::min(c.max_forward_rad, c.max_forward_fraction*std::fabs(_center_right)), _center_right);
            // Lose quiet balance while reaching: retract under the ordinary
            // balance controller. Never release wheel control on a timer.
            if (std::fabs(in.rate) > 20 || std::fabs(in.error) > 8
                || std::fabs(in.wheel_left) > 4 || std::fabs(in.wheel_right) > 4) {
                cancel(now, "lower_reach_disturbed"); break;
            }
            const bool left_contact = contact(_left, in.arm_left, in.velocity_left, in.torque_left, _center_left, c);
            const bool right_contact = contact(_right, in.arm_right, in.velocity_right, in.torque_right, _center_right, c);
            if (confirmed(left_contact && right_contact && calm, dt, c.contact_ms)) {
                // Remove the small contact-seeking preload before transferring
                // weight. Keeping a target beyond the floor can jack the body
                // backward when the wheel balance controller releases it.
                _left = in.arm_left;
                _right = in.arm_right;
                _contact_tilt = in.tilt;
                _contact_left = _left; _contact_right = _right;
                enter(LowerPhase::Loading, now);
                break;
            }
            if (!left_contact) _left = approach(_left, left_limit, in.arm_left, c.reach_speed * dt, c.max_target_lead);
            if (!right_contact) _right = approach(_right, right_limit, in.arm_right, c.reach_speed * dt, c.max_target_lead);
            if (elapsed > c.reach_timeout_ms
                || (at(_left, left_limit)
                    && at(_right, right_limit)
                    && !left_contact && !right_contact)) cancel(now, "lower_no_support");
            break;
        }
        case LowerPhase::Loading: {
            // Keep wheel balancing while leaning onto the support. Contact at
            // the exact unstable equilibrium is not enough: retract only a
            // few degrees and require a measured forward weight transfer.
            _left = approach(_left, _contact_left + std::copysign(c.load_travel, _center_left), in.arm_left, c.load_speed*dt, c.max_target_lead);
            _right = approach(_right, _contact_right + std::copysign(c.load_travel, _center_right), in.arm_right, c.load_speed*dt, c.max_target_lead);
            const bool loaded = in.tilt <= _contact_tilt - 2 && std::fabs(in.rate) <= c.calm_rate
                && std::fabs(in.wheel_left) <= c.calm_wheel && std::fabs(in.wheel_right) <= c.calm_wheel
                && std::fabs(in.torque_left) >= c.contact_torque*.5f
                && std::fabs(in.torque_right) >= c.contact_torque*.5f;
            if (confirmed(loaded, dt, c.contact_ms)) {
                _committed = true; _committed_ms = now;
                _progress_tilt = in.tilt; _progress_ms = now;
                enter(LowerPhase::Descending, now);
            } else if (elapsed > 5000) cancel(now, "lower_weight_transfer_failed");
            break;
        }
        case LowerPhase::Descending:
            if (flat) { enter(LowerPhase::GroundHold, now); break; }
            _support_lost_ms = in.tilt > 15 && in.rate < -25
                && (std::fabs(in.torque_left) < c.contact_torque*.5f || std::fabs(in.torque_right) < c.contact_torque*.5f)
                ? _support_lost_ms + dt*1000 : 0;
            if (_support_lost_ms >= 100) { fail("lower_support_lost"); break; }
            if (in.tilt < _progress_tilt - 2) { _progress_tilt = in.tilt; _progress_ms = now; }
            if (now - _committed_ms > c.descent_timeout_ms || now - _progress_ms > c.progress_timeout_ms) {
                fail("lower_descent_timeout"); break;
            }
            // Pausing the arm trajectory retains its support when the body
            // moves too quickly. No wheel kick or open-loop timed free fall.
            if (in.rate >= -c.descent_rate && in.rate <= c.calm_rate) {
                _left = approach(_left, 0, in.arm_left, c.lower_speed * dt, c.max_target_lead);
                _right = approach(_right, 0, in.arm_right, c.lower_speed * dt, c.max_target_lead);
            }
            break;
        case LowerPhase::GroundHold:
            if (now - _committed_ms > c.descent_timeout_ms) { fail("lower_descent_timeout"); break; }
            if (!flat) { enter(LowerPhase::Descending, now); _progress_ms = now; _progress_tilt = in.tilt; break; }
            if (confirmed(flat, dt, c.flat_ms)) enter(LowerPhase::Retracting, now);
            break;
        case LowerPhase::Retracting:
            if (std::fabs(in.tilt) > c.flat_angle + 4 || std::fabs(in.rate) > 20) {
                fail("lower_ground_unstable"); break;
            }
            _left = approach(_left, 0, in.arm_left, c.retract_speed * dt, c.max_target_lead);
            _right = approach(_right, 0, in.arm_right, c.retract_speed * dt, c.max_target_lead);
            if (at(_left, 0) && at(_right, 0) && std::fabs(in.arm_left) < .06f
                && std::fabs(in.arm_right) < .06f && flat) finish("lower_complete");
            else if (elapsed > c.retract_timeout_ms) fail("lower_retract_timeout");
            break;
        case LowerPhase::Canceling:
            _left = approach(_left, 0, in.arm_left, c.retract_speed * dt, c.max_target_lead);
            _right = approach(_right, 0, in.arm_right, c.retract_speed * dt, c.max_target_lead);
            if (at(_left, 0) && at(_right, 0) && std::fabs(in.arm_left) < .06f && std::fabs(in.arm_right) < .06f)
                finish(_reason);
            else if (elapsed > c.reach_timeout_ms) fail("lower_cancel_timeout");
            break;
        default: break;
        }
    }

private:
    volatile LowerPhase _phase = LowerPhase::Idle; // read-only status from display/network snapshots
    bool _committed = false;
    float _left = 0, _right = 0, _center_left = 0, _center_right = 0;
    float _confirm_ms = 0, _progress_tilt = 0, _support_lost_ms = 0;
    float _contact_tilt = 0, _contact_left = 0, _contact_right = 0;
    uint32_t _phase_ms = 0, _progress_ms = 0, _committed_ms = 0;
    const char* _reason = "lower_requested";
    static bool at(float a, float b) { return std::fabs(a - b) < .001f; }
    static bool valid(const LowerInput& i) {
        const float values[] = {i.tilt,i.rate,i.error,i.wheel_left,i.wheel_right,
            i.arm_left,i.arm_right,i.velocity_left,i.velocity_right,i.torque_left,i.torque_right};
        for (float v : values) if (!std::isfinite(v)) return false;
        return true;
    }
    static float approach(float value, float goal, float measured, float step, float lead) {
        const float next = value + std::max(-step, std::min(step, goal - value));
        return std::max(measured - lead, std::min(measured + lead, next));
    }
    static bool contact(float target, float measured, float velocity, float torque, float center, const LowerConfig& c) {
        const float direction = center > 0 ? 1.f : -1.f;
        return measured / center < -.65f
            && (measured - target) * direction >= c.contact_error
            && std::fabs(torque) >= c.contact_torque && std::fabs(velocity) <= c.contact_velocity;
    }
    bool confirmed(bool condition, float dt, uint32_t duration) {
        _confirm_ms = condition ? _confirm_ms + dt * 1000 : 0;
        return _confirm_ms + .01f >= duration;
    }
    void enter(LowerPhase phase, uint32_t now) { _phase = phase; _phase_ms = now; _confirm_ms = 0; }
    void cancel(uint32_t now, const char* reason) { _reason = reason; enter(LowerPhase::Canceling, now); }
    void fail(const char* reason) { _reason = reason; _phase = LowerPhase::Fault; }
    void finish(const char* reason) { _reason = reason; _phase = LowerPhase::Complete; }
};
} // namespace balance_math
