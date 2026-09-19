// Exercise the actual firmware pilot class from the approximate planar model.
#include "balance_pilot.h"
#include "config.h"
static const balance_math::PilotConfig config = {
    BALANCE_PILOT_DEADBAND, BALANCE_PILOT_MAX_VEL, BALANCE_PILOT_MAX_TURN,
    BALANCE_PILOT_ACCEL, BALANCE_PILOT_DECEL, BALANCE_PILOT_TURN_ACCEL,
    BALANCE_PILOT_READY_MS, BALANCE_PILOT_STOP_MS
};
extern "C" {
void* pilot_create() { return new balance_math::BalancePilot; }
void pilot_destroy(void* p) { delete static_cast<balance_math::BalancePilot*>(p); }
void pilot_tick(void* ptr, int phase, int fresh, float speed, float rate, float err,
                float forward, float turn, float dt, float* result) {
    auto& p=*static_cast<balance_math::BalancePilot*>(ptr);
    // Planar plant supplies fresh wheel feedback and has no yaw motion model.
    const bool allowed=phase && fresh && err<BALANCE_PILOT_PAUSE_ERR
                       && std::fabs(rate)<BALANCE_PILOT_PAUSE_RATE;
    const bool calm=phase && std::fabs(speed)<BALANCE_PILOT_STOP_SPEED
                    && std::fabs(rate)<BALANCE_START_RECOVERY_CALM_RATE
                    && err<BALANCE_START_RECOVERY_CALM_ERR;
    p.update(allowed,calm,forward,turn,dt,config);
    result[0]=p.ready();result[1]=p.moving();result[2]=p.velocity();result[3]=p.turn();
}
}
extern "C" float pilot_correction(void* ptr, float error, float low, float high, float knee, int ramp) {
    return static_cast<balance_math::BalancePilot*>(ptr)->velocityCorrection(
        error, low, high, knee, BALANCE_PILOT_VEL_KP_LOW, ramp);
}
