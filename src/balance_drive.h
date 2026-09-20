#pragma once
#include "balance_math.h"

namespace balance_math {
// Separate driving-rate filter. The original stationary/startup rate remains
// untouched. Call only on a fresh IMU sample, with that sample's actual dt.
class DriveRateFilter {
public:
    float update(float raw, float dt, float tau) {
        if (!std::isfinite(raw) || !std::isfinite(dt) || dt<=0 || dt>.02f
            || !std::isfinite(tau) || tau<0) {
            reset(); return 0;
        }
        if (!_initialized) { _value=raw; _initialized=true; }
        else _value+=dt/(tau+dt)*(raw-_value);
        return _value;
    }
    void reset() { _value=0; _initialized=false; }
private:
    float _value=0;
    bool _initialized=false;
};

struct DriveArmConfig {
    float ordinary_scale, ordinary_limit, headroom, error, rate, confirm_ms;
    float severe_error, severe_rate;
};
class DriveArmRecovery {
public:
    static float ordinary(float demand,const DriveArmConfig& c) {
        return clamp(demand*c.ordinary_scale,-c.ordinary_limit,c.ordinary_limit);
    }
    void reset() { _qualifying_ms=0; _direction=0; }
    bool update(bool driving,float command,float speed_error,float angle_error,
                float rate,float dt,float limit,float speed_threshold,
                const DriveArmConfig& c) {
        if (!driving || !std::isfinite(command) || !std::isfinite(speed_error)
            || !std::isfinite(angle_error) || !std::isfinite(rate)
            || !std::isfinite(dt) || dt<=0 || dt>.1f) { reset(); return false; }
        // Positive outward rate means the body is moving away from its target.
        const float outward=angle_error>0 ? -rate : rate;
        const bool moving_away=std::fabs(angle_error)>=c.error && outward>=c.rate;
        const bool velocity_disturbance=std::fabs(speed_error)>speed_threshold;
        const bool near_limit=limit-std::fabs(command)<=c.headroom;
        const bool qualifying=velocity_disturbance && moving_away && near_limit;
        const float direction=angle_error>0 ? 1.0f : -1.0f;
        if (!qualifying) reset();
        else {
            if (direction!=_direction) _qualifying_ms=0;
            _direction=direction;
            _qualifying_ms+=dt*1000;
        }
        // A large and worsening lean is urgent even before a wheel rails.
        const bool severe=velocity_disturbance && std::fabs(angle_error)>=c.severe_error
                          && outward>=c.severe_rate;
        return severe || _qualifying_ms+.001f>=c.confirm_ms;
    }
private:
    float _qualifying_ms=0, _direction=0;
};

struct DriveConfig {
    float angle_gain, rate_gain, speed_gain, speed_error_limit;
    float acceleration_limit, handoff_rate;
    float brake_gain, brake_accel_limit, brake_fade_start, brake_fade_full, brake_tau;
    constexpr DriveConfig(float angle, float rate, float speed, float error_limit,
                          float accel_limit, float handoff, float brake=0,
                          float brake_limit=0, float fade_start=1,
                          float fade_full=4, float tau=.08f)
        : angle_gain(angle), rate_gain(rate), speed_gain(speed),
          speed_error_limit(error_limit), acceleration_limit(accel_limit),
          handoff_rate(handoff), brake_gain(brake), brake_accel_limit(brake_limit),
          brake_fade_start(fade_start), brake_fade_full(fade_full), brake_tau(tau) {}
};
struct DriveResult {
    float raw, speed, acceleration;
    bool active;
    float brake_acceleration;
    DriveResult(float raw_command,float velocity,float accel,bool enabled,float brake=0)
        : raw(raw_command), speed(velocity), acceleration(accel), active(enabled),
          brake_acceleration(brake) {}
};

// While driving, turn tilt/rate/speed error into wheel ACCELERATION, then
// integrate the motor velocity reference. This actuator state is not a second
// equilibrium estimator; the existing outer integral remains the learned bias.
class BalanceDrive {
public:
    void reset() { _speed=0; _brake_acceleration=0; _active=false; _handoff=false; }
    DriveResult step(bool enabled, float angle_error, float body_rate,
                     float measured_speed, float target_speed, float pd_command,
                     float previous_command, float dt, float max_speed,
                     const DriveConfig& c, bool stopping=false) {
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
            // Centered throttle / input-loss braking only. Add bounded speed
            // feedback while rolling faster than the ramped target, then fade
            // it out near zero to avoid a strong opposite correction at rest.
            const float speed_error=measured_speed-target_speed;
            float brake_target=0;
            if (stopping && measured_speed*speed_error>0) {
                const float fade=clamp((std::fabs(measured_speed)-c.brake_fade_start)
                    / (c.brake_fade_full-c.brake_fade_start),0,1);
                brake_target=clamp(c.brake_gain*speed_error*fade,
                    -c.brake_accel_limit,c.brake_accel_limit);
            }
            if (!stopping) _brake_acceleration=0;
            else _brake_acceleration+=dt/(c.brake_tau+dt)*(brake_target-_brake_acceleration);
            float acceleration = _brake_acceleration+c.angle_gain*angle_error-c.rate_gain*body_rate
                + c.speed_gain*clamp(measured_speed-target_speed,
                                     -c.speed_error_limit,c.speed_error_limit);
            acceleration=clamp(acceleration,-c.acceleration_limit,c.acceleration_limit);
            const float raw=_speed+acceleration*dt;
            _speed=clamp(raw,-max_speed,max_speed); // actuator anti-windup
            return {raw,_speed,acceleration,true,_brake_acceleration};
        }
        _brake_acceleration=0;
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
    float _speed=0, _brake_acceleration=0;
    bool _active=false, _handoff=false;
};
}
