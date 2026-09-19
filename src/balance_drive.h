#pragma once
#include "balance_math.h"

namespace balance_math {
struct DriveConfig {
    float angle_gain, rate_gain, speed_gain, speed_error_limit;
    float acceleration_limit, handoff_rate;
};
struct DriveResult { float raw, speed, acceleration; bool active; };

// While driving, turn tilt/rate/speed error into wheel ACCELERATION, then
// integrate the motor velocity reference. This actuator state is not a second
// equilibrium estimator; the existing outer integral remains the learned bias.
class BalanceDrive {
public:
    void reset() { _speed=0; _active=false; _handoff=false; }
    DriveResult step(bool enabled, float angle_error, float body_rate,
                     float measured_speed, float target_speed, float pd_command,
                     float previous_command, float dt, float max_speed,
                     const DriveConfig& c) {
        if (!std::isfinite(dt) || dt <= 0 || dt > .02f
            || !std::isfinite(angle_error) || !std::isfinite(body_rate)
            || !std::isfinite(measured_speed) || !std::isfinite(target_speed)
            || !std::isfinite(pd_command) || !std::isfinite(previous_command)) {
            reset(); return {0,0,0,false};
        }
        if (enabled) {
            if (!_active) _speed=clamp(previous_command,-max_speed,max_speed);
            _active=true; _handoff=false;
            // Bound the travel demand separately; balance still has the full
            // configured acceleration and common-speed authority to catch lean.
            float acceleration = c.angle_gain*angle_error-c.rate_gain*body_rate
                + c.speed_gain*clamp(measured_speed-target_speed,
                                     -c.speed_error_limit,c.speed_error_limit);
            acceleration=clamp(acceleration,-c.acceleration_limit,c.acceleration_limit);
            const float raw=_speed+acceleration*dt;
            _speed=clamp(raw,-max_speed,max_speed); // actuator anti-windup
            return {raw,_speed,acceleration,true};
        }
        if (_active) { _active=false; _handoff=true; }
        if (_handoff) {
            const float target=clamp(pd_command,-max_speed,max_speed);
            _speed+=clamp(target-_speed,-c.handoff_rate*dt,c.handoff_rate*dt);
            if (_speed==target) _handoff=false;
            return {_speed,_speed,0,_handoff};
        }
        _speed=clamp(pd_command,-max_speed,max_speed);
        return {pd_command,_speed,0,false};
    }
private:
    float _speed=0;
    bool _active=false, _handoff=false;
};
}
