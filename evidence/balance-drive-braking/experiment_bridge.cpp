#include "rejected-balance-pilot.h"
#include "rejected-balance-drive.h"
#include "rejected_stop_capture.h"
#include "rejected-config.h"
static const balance_math::PilotConfig baseconfig = {
    BALANCE_PILOT_DEADBAND, BALANCE_PILOT_MAX_VEL, BALANCE_PILOT_MAX_TURN,
    BALANCE_PILOT_ACCEL, BALANCE_PILOT_DECEL, BALANCE_PILOT_TURN_ACCEL,
    BALANCE_PILOT_READY_MS, BALANCE_PILOT_STOP_MS,
    BALANCE_PILOT_ARM_GAIN, BALANCE_PILOT_ARM_LIMIT, BALANCE_PILOT_ARM_TAU
};
static const balance_math::DriveConfig basedc = {
    BALANCE_DRIVE_ANGLE_K,BALANCE_DRIVE_RATE_K,BALANCE_DRIVE_SPEED_K,
    BALANCE_DRIVE_ERROR_LIMIT,BALANCE_DRIVE_ACCEL_LIMIT,BALANCE_DRIVE_HANDOFF_RATE,
        BALANCE_DRIVE_BRAKE_K, BALANCE_DRIVE_BRAKE_LIMIT,
        BALANCE_DRIVE_BRAKE_FADE_START, BALANCE_DRIVE_BRAKE_FADE_FULL,
        BALANCE_DRIVE_BRAKE_TAU
};
struct Trial {
 balance_math::BalancePilot p; balance_math::BalanceDrive drive;
 balance_math::DriveStopCapture stop;
 balance_math::DriveStopConfig stopconfig={1,2.5f,8,60};
 bool capture_enabled=true;
 balance_math::DriveRateFilter rate; balance_math::DriveArmRecovery arms;
 balance_math::PilotConfig config=baseconfig; balance_math::DriveConfig dc=basedc;
 balance_math::DriveArmConfig ac={BALANCE_DRIVE_ARM_SCALE,BALANCE_DRIVE_ARM_LIMIT,
 BALANCE_DRIVE_ARM_HEADROOM,BALANCE_DRIVE_ARM_ERROR,BALANCE_DRIVE_ARM_OUTWARD_RATE,
 BALANCE_DRIVE_ARM_CONFIRM_MS,BALANCE_DRIVE_ARM_SEVERE_ERROR,BALANCE_DRIVE_ARM_SEVERE_RATE};
 float tau=BALANCE_DRIVE_RATE_TAU; float filtered_rate=0; bool revised=true;
};
extern "C" {
void* pilot_create() { return new Trial; }
void pilot_params(void* ptr,const float* v) {
 auto& t=*static_cast<Trial*>(ptr);
 t.dc.angle_gain=v[0];t.dc.rate_gain=v[1];t.dc.speed_gain=v[2];t.tau=v[3];
 t.config.acceleration=v[4];t.config.deceleration=v[5];
 t.ac.ordinary_scale=v[6];t.ac.ordinary_limit=v[7];t.revised=v[8]>0;
}
void pilot_brake_params(void* ptr,float gain,float limit,float fade_start,float fade_full,float tau) {
 auto& t=*static_cast<Trial*>(ptr);t.dc.brake_gain=gain;t.dc.brake_accel_limit=limit;
 t.dc.brake_fade_start=fade_start;t.dc.brake_fade_full=fade_full;t.dc.brake_tau=tau;
}
void pilot_arm_cap(void* ptr,float limit) { static_cast<Trial*>(ptr)->config.arm_brake_limit=limit; }
void pilot_stop_params(void* ptr,int enabled,float speed,float angle,float rate,float ms) {
 auto& t=*static_cast<Trial*>(ptr);t.capture_enabled=enabled;t.stopconfig={speed,angle,rate,ms};
}
void pilot_destroy(void* p) { delete static_cast<Trial*>(p); }
void pilot_tick(void* ptr, int phase, int fresh, float speed, float rate, float err,
                float forward, float turn, float dt, float* result) {
    auto& t=*static_cast<Trial*>(ptr); auto& p=t.p;
    const bool allowed=phase && fresh && err<BALANCE_PILOT_PAUSE_ERR
                       && std::fabs(rate)<BALANCE_PILOT_PAUSE_RATE;
    const bool calm=phase && std::fabs(speed)<BALANCE_PILOT_STOP_SPEED
                    && std::fabs(rate)<BALANCE_START_RECOVERY_CALM_RATE
                    && err<BALANCE_START_RECOVERY_CALM_ERR;
    p.update(allowed,calm,forward,turn,dt,t.config);
    t.stop.update(t.capture_enabled && p.moving(),p.forwardStick()==0,p.velocity()==0,
                  speed,err,rate,dt,t.stopconfig);
    result[0]=p.ready();result[1]=p.moving();result[2]=p.velocity();result[3]=p.turn();
    result[4]=t.stop.captured()?0:p.armAssist();result[5]=t.stop.captured();
}
float pilot_correction(void* ptr,float error,float low,float high,float knee,int ramp) {
 auto& t=*static_cast<Trial*>(ptr);
 return t.stop.captured() ? t.p.velocityCorrection(error,low,high,knee,low,ramp) : 0;
}
int pilot_learning(void*,float error) { return std::fabs(error)<BALANCE_DRIVE_LEARN_ERR; }
float pilot_arm_error(float speed,float target) { return balance_math::BalancePilot::armVelocityError(speed,target); }
void pilot_drive(void* ptr,float error,float raw_rate,float old_rate,float speed,float pd,float previous,float dt,float limit,float* out) {
 auto& t=*static_cast<Trial*>(ptr);
 float filtered=t.filtered_rate=t.rate.update(raw_rate,dt,t.tau);
 auto r=t.drive.step(t.p.moving() && !t.stop.captured(),error,t.revised?filtered:old_rate,speed,t.p.velocity(),pd,previous,dt,limit,t.dc,t.p.stopping());
 out[0]=r.raw;out[1]=r.active;
}
float pilot_arm_demand(void* ptr,float demand) {
 auto& t=*static_cast<Trial*>(ptr);
 return t.revised ? balance_math::DriveArmRecovery::ordinary(demand,t.ac) : demand;
}
int pilot_emergency(void* ptr,int moving,float cmd,float velerr,float err,float rate,float dt,float limit,int old) {
 auto& t=*static_cast<Trial*>(ptr);
 bool active=t.arms.update(moving,cmd,velerr,err,t.revised?t.filtered_rate:rate,dt,limit,BALANCE_ARM_ASSIST_THRESH,t.ac);
 return t.revised && moving ? active : old;
}
}
