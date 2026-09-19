#include "balance_pilot.h"
#include "balance_drive.h"
#include "config.h"
static const balance_math::PilotConfig config = {
    BALANCE_PILOT_DEADBAND, BALANCE_PILOT_MAX_VEL, BALANCE_PILOT_MAX_TURN,
    BALANCE_PILOT_ACCEL, BALANCE_PILOT_DECEL, BALANCE_PILOT_TURN_ACCEL,
    BALANCE_PILOT_READY_MS, BALANCE_PILOT_STOP_MS,
    BALANCE_PILOT_ARM_GAIN, BALANCE_PILOT_ARM_LIMIT, BALANCE_PILOT_ARM_TAU
};
static const balance_math::DriveConfig dc = {
    BALANCE_DRIVE_ANGLE_K,BALANCE_DRIVE_RATE_K,BALANCE_DRIVE_SPEED_K,
    BALANCE_DRIVE_ERROR_LIMIT,BALANCE_DRIVE_ACCEL_LIMIT,BALANCE_DRIVE_HANDOFF_RATE
};
struct Trial { balance_math::BalancePilot p; balance_math::BalanceDrive drive; };
extern "C" {
void* pilot_create() { return new Trial; }
void pilot_destroy(void* p) { delete static_cast<Trial*>(p); }
void pilot_tick(void* ptr, int phase, int fresh, float speed, float rate, float err,
                float forward, float turn, float dt, float* result) {
    auto& p=static_cast<Trial*>(ptr)->p;
    const bool allowed=phase && fresh && err<BALANCE_PILOT_PAUSE_ERR
                       && std::fabs(rate)<BALANCE_PILOT_PAUSE_RATE;
    const bool calm=phase && std::fabs(speed)<BALANCE_PILOT_STOP_SPEED
                    && std::fabs(rate)<BALANCE_START_RECOVERY_CALM_RATE
                    && err<BALANCE_START_RECOVERY_CALM_ERR;
    p.update(allowed,calm,forward,turn,dt,config);
    result[0]=p.ready();result[1]=p.moving();result[2]=p.velocity();result[3]=p.turn();
    result[4]=p.armAssist();result[5]=0;
}
float pilot_correction(void*,float,float,float,float,int) { return 0; }
int pilot_learning(void*,float error) { return std::fabs(error)<BALANCE_DRIVE_LEARN_ERR; }
float pilot_arm_error(float speed,float target) { return balance_math::BalancePilot::armVelocityError(speed,target); }
void pilot_drive(void* ptr,float error,float rate,float speed,float pd,float previous,float dt,float limit,float* out) {
 auto& t=*static_cast<Trial*>(ptr);
 auto r=t.drive.step(t.p.moving(),error,rate,speed,t.p.velocity(),pd,previous,dt,limit,dc);
 out[0]=r.raw;out[1]=r.active;
}
}
