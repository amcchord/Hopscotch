#include "balance_drive.h"
#include "config.h"
#include <cassert>
#include <iostream>
#include <random>

static const balance_math::DriveConfig c = {
    BALANCE_DRIVE_ANGLE_K, BALANCE_DRIVE_RATE_K, BALANCE_DRIVE_SPEED_K,
    BALANCE_DRIVE_ERROR_LIMIT, BALANCE_DRIVE_ACCEL_LIMIT, BALANCE_DRIVE_HANDOFF_RATE
};
int main() {
    balance_math::BalanceDrive drive;
    auto tick=[&](bool enabled,float err,float rate,float speed,float target,
                  float pd=0,float previous=0,float dt=.005f,float limit=30) {
        return drive.step(enabled,err,rate,speed,target,pd,previous,dt,limit,c);
    };
    for(float pd : {-40.f,-2.f,0.f,2.f,40.f}) {
        auto r=tick(false,0,0,0,0,pd);
        assert(r.raw==pd && r.speed==balance_math::clamp(pd,-30,30) && !r.active);
    }
    for(int sign : {-1,1}) {
        drive.reset();
        // From equilibrium, a speed request immediately initiates the lean
        // with opposite wheel acceleration, independent of equilibrium I.
        auto r=tick(true,0,0,0,sign*20.f,0,sign*2.f);
        assert(r.active && sign*r.acceleration<0);
        assert(std::fabs(r.speed-sign*2.f-r.acceleration*.005f)<.000001f);
        assert(std::fabs(r.acceleration)==c.speed_gain*c.speed_error_limit);
        // Large tilt must retain full balance authority even while the
        // intended travel error is clipped. A small combined cap loses catches.
        r=tick(true,sign*20.f,0,0,sign*20.f);
        assert(r.acceleration==sign*c.acceleration_limit);
        for(int i=0;i<1000;++i) r=tick(true,sign*20.f,0,0,0);
        assert(r.speed==sign*30.f && sign*r.raw>30);
        // Stored actuator state cannot wind up beyond the motor cap.
        r=tick(true,0,0,0,sign*20.f);
        assert(sign*r.speed<30 && sign*r.speed>29);
        // Soft-deadman reduced wheel limit is honored immediately.
        r=tick(true,0,0,0,0,0,0,.005f,4);
        assert(std::fabs(r.speed)<=4);
        float previous=r.speed;
        for(int i=0;i<100;++i) {
            r=tick(false,0,0,0,0,-sign*2.f);
            assert(std::fabs(r.speed-previous)<=c.handoff_rate*.005f+.000001f);
            previous=r.speed;
        }
        assert(r.speed==-sign*2.f && !r.active);
        r=tick(false,0,0,0,0,1.25f);
        assert(r.raw==1.25f && !r.active); // original PD after completed handoff
    }
    for(float bad : {0.f,-.1f,.021f,NAN,INFINITY}) {
        tick(true,0,0,0,20);
        const auto r=tick(true,0,0,0,20,0,0,bad);
        assert(!r.active && r.raw==0 && r.speed==0);
    }
    for(int field=0;field<6;++field) {
        float args[6]={0,0,0,20,0,0};args[field]=NAN;
        auto r=tick(true,args[0],args[1],args[2],args[3],args[4],args[5]);
        assert(!r.active && r.raw==0);
    }
    std::mt19937 rng(19);std::uniform_real_distribution<float> input(-40,40);
    float previous=0;
    for(int i=0;i<50000;++i) {
        const auto r=tick(i%17!=0,input(rng),input(rng),input(rng),input(rng),input(rng),previous);
        assert(std::isfinite(r.speed) && std::fabs(r.speed)<=30);
        assert(std::fabs(r.acceleration)<=c.acceleration_limit);
        const auto mixed=balance_math::mix(r.speed,input(rng),30);
        assert(std::fabs(mixed.left)<=30 && std::fabs(mixed.right)<=30);
        assert(std::fabs((mixed.left+mixed.right)*.5f-r.speed)<.00001f);
        previous=r.speed;
    }
    const auto mixed=balance_math::mix(20,-BALANCE_PILOT_MAX_TURN,30);
    assert(mixed.left==24.5f && mixed.right==15.5f);
    std::cout<<"Acceleration drive checks passed: untouched PD, immediate lean, balance authority, anti-windup, handoff, invalid inputs, 50000 bounded mixer updates\n";
}
