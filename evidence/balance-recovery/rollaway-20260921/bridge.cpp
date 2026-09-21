#include "balance_drive.h"
#include "config.h"
extern "C" {
struct Recovery { balance_math::BalanceDrive drive; balance_math::DriveRateFilter filter; };
float arm_speed(){return BALANCE_ARM_ASSIST_SPEED;}
void* create(){return new Recovery;}
void destroy(void* p){delete static_cast<Recovery*>(p);}
float step(void* p,bool enabled,float error,float rate,float velocity,float target,float pd,float prev,float dt,float limit){
 auto& r=*static_cast<Recovery*>(p);
 const balance_math::DriveConfig c={BALANCE_DRIVE_ANGLE_K,BALANCE_DRIVE_RATE_K,
  BALANCE_DRIVE_SPEED_K,BALANCE_DRIVE_ERROR_LIMIT,BALANCE_DRIVE_ACCEL_LIMIT,
  BALANCE_DRIVE_HANDOFF_RATE};
 return r.drive.step(enabled,error,r.filter.update(rate,dt,BALANCE_DRIVE_RATE_TAU),velocity,target,pd,prev,dt,limit,c).raw;
}
}
