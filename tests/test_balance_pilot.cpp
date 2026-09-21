#include "balance_pilot.h"
#include "balance_drive.h"
#include "balance_telemetry.h"
#include "config.h"
#include <cassert>
#include <iostream>
#include <random>

static const balance_math::PilotConfig config = {
    BALANCE_PILOT_DEADBAND, BALANCE_PILOT_MAX_VEL, BALANCE_PILOT_MAX_TURN,
    BALANCE_PILOT_ACCEL, BALANCE_PILOT_DECEL, BALANCE_PILOT_TURN_ACCEL,
    BALANCE_PILOT_READY_MS, BALANCE_PILOT_STOP_MS,
    BALANCE_PILOT_ARM_GAIN, BALANCE_PILOT_ARM_LIMIT, BALANCE_PILOT_ARM_TAU,
    BALANCE_PILOT_FAST_DECEL, BALANCE_PILOT_BRAKE_START_SPEED,
    BALANCE_PILOT_BRAKE_FULL_SPEED, BALANCE_PILOT_ARM_BRAKE_LIMIT
};
static void unlock(balance_math::BalancePilot& p) {
    for(int i=0;i<20;++i) p.update(true,true,0,0,.02f,config);
    assert(p.ready() && !p.moving());
}
int main() {
    // A CH11 stop must not wait indefinitely for calm in the driving
    // controller that is producing the neutral oscillation. Finish the same
    // reference ramps, then use the existing bounded handoff to stationary PD.
    for(int sign : {-1,1}) {
        balance_math::BalancePilot stopping, ordinary;
        unlock(stopping); unlock(ordinary);
        for(int i=0;i<80;++i) {
            stopping.update(true,false,sign*.3f,sign*.3f,.02f,config);
            ordinary.update(true,false,sign*.3f,sign*.3f,.02f,config);
        }
        balance_math::BalanceDrive drive;
        const balance_math::DriveConfig dc={BALANCE_DRIVE_ANGLE_K,BALANCE_DRIVE_RATE_K,
            BALANCE_DRIVE_SPEED_K,BALANCE_DRIVE_ERROR_LIMIT,BALANCE_DRIVE_ACCEL_LIMIT,
            BALANCE_DRIVE_HANDOFF_RATE};
        float command=sign*2.f;
        bool handed_off=false;
        for(int i=0;i<200;++i) {
            stopping.update(true,false,0,0,.02f,config);
            ordinary.update(true,false,0,0,.02f,config);
            const float velocity=stopping.velocity(),turn=stopping.turn();
            stopping.handoffStoppedForLowering();
            assert(stopping.velocity()==velocity && stopping.turn()==turn);
            if(velocity!=0 || turn!=0) assert(stopping.moving());
            else { assert(!stopping.moving() && !stopping.ready()); handed_off=true; }
            assert(ordinary.moving()); // unchanged ordinary calm-stop contract
            for(int tick=0;tick<4;++tick) {
                auto result=drive.step(stopping.moving(),0,0,command,velocity,0,
                    command,.005f,30,dc);
                if(handed_off) assert(std::fabs(result.speed-command)<=dc.handoff_rate*.005f+.00001f);
                command=result.speed;
            }
        }
        assert(handed_off && command==0);
    }
    static_assert(BALANCE_PILOT_FAST_DECEL > BALANCE_PILOT_DECEL, "faster high-speed braking");
    static_assert(BALANCE_PILOT_BRAKE_FULL_SPEED > BALANCE_PILOT_BRAKE_START_SPEED, "valid blend");
    for (int sign : {-1,1}) {
        balance_math::BalancePilot candidate, previous;
        auto old=config;
        old.fast_deceleration=old.deceleration;
        old.arm_brake_limit=0;
        // Low-speed motions, including reversals and their arm feedforward,
        // remain bit-for-bit the same as the smooth v4 behavior.
        for(int i=0;i<500;++i) {
            float stick=i<25 ? 0 : i<125 ? sign*.15f : i<225 ? 0
                         : i<325 ? -sign*.15f : 0;
            candidate.update(true,true,stick,0,.02f,config);
            previous.update(true,true,stick,0,.02f,old);
            assert(candidate.velocity()==previous.velocity());
            assert(candidate.armAssist()==previous.armAssist());
            assert(!candidate.fastBraking());
        }
        candidate.reset();unlock(candidate);
        for(int i=0;i<230;++i) candidate.update(true,false,sign,0,.02f,config);
        int ticks=0;
        while(candidate.velocity()!=0 && ticks<150) {
            const float before=candidate.velocity();
            candidate.update(true,false,0,0,.02f,config);
            assert(sign*candidate.velocity()>=0);
            assert(std::fabs(candidate.velocity()-before)<=config.fast_deceleration*.02f+.00001f);
            assert(std::fabs(candidate.armAssist())<=BALANCE_PILOT_ARM_BRAKE_LIMIT+.00001f);
            if(std::fabs(before)<=BALANCE_PILOT_BRAKE_START_SPEED) {
                assert(!candidate.fastBraking());
                assert(std::fabs(candidate.velocity()-before)<=config.deceleration*.02f+.00001f);
            }
            ++ticks;
        }
        assert(ticks>=65 && ticks<=75); // 20→0 reference: about 1.4 s, formerly 2.5 s
        assert(candidate.moving()); // measured calm still required before hold
    }
    balance_math::BalancePilot p;
    for(int i=0;i<500;++i) {
        p.update(false,true,1,1,.02f,config);
        assert(!p.ready() && !p.moving() && p.velocity()==0 && p.turn()==0);
    }
    for(int i=0;i<500;++i) {
        p.update(true,true,1,0,.02f,config);
        assert(!p.ready() && p.velocity()==0); // stick held through stand-up
    }
    for(int i=0;i<19;++i) p.update(true,true,0,0,.02f,config);
    assert(!p.ready());
    p.update(true,false,0,0,.02f,config); // calm gate must be continuous
    assert(!p.ready());
    unlock(p);
    assert(p.forwardStick()==0 && p.turnStick()==0);
    assert(!p.fastBraking()); // an untouched standing robot never enters braking
    auto correction = [&](float error, bool ramp=true) {
        return p.velocityCorrection(error, BALANCE_VEL_SP_KP_LOW,
            BALANCE_VEL_SP_KP, BALANCE_VEL_SP_KNEE, BALANCE_PILOT_VEL_KP_LOW, ramp);
    };
    assert(std::fabs(correction(.5f)-.5f*BALANCE_VEL_SP_KP_LOW)<.00001f);
    p.update(true,true,.05f,-.05f,.02f,config);
    assert(!p.moving()); // center jitter stays inside deadband
    for(int sign : {-1,1}) {
        p.reset();unlock(p);
        p.update(true,false,sign,sign,.02f,config);
        assert(p.moving() && p.turning());
        assert(!p.fastBraking());
        assert(sign*p.armAssist()<0); // start: shift opposite the braking swing
        assert(p.plannedArm(1)==0 && p.plannedArm(2)==0); // catch always wins
        assert(p.plannedArm(0)==p.armAssist() && p.plannedArm(3)==p.armAssist());
        assert(std::fabs(correction(sign*.5f)-sign*.5f*BALANCE_PILOT_VEL_KP_LOW)<.00001f);
        assert(std::fabs(correction(sign*.5f,false)-sign*.5f*BALANCE_VEL_SP_KP_LOW)<.00001f);
        const float knee=BALANCE_VEL_SP_KNEE;
        assert(std::fabs(correction(knee+.0001f)-correction(knee-.0001f))<.001f);
        // Starting at equilibrium: the P response must overcome cruising FF,
        // briefly moving wheels opposite the request to initiate the lean.
        assert(sign*(sign*.5f + 2.f*correction(-sign*.5f)) < 0);
        assert(std::fabs(p.velocity()-sign*config.acceleration*.02f)<.00001f);
        // Allow the configured ramp to finish, then verify planned arms relax.
        const int cruise_ticks=static_cast<int>(std::ceil(config.max_velocity/(config.acceleration*.02f)))+60;
        for(int i=0;i<cruise_ticks;++i) p.update(true,false,sign,sign,.02f,config);
        assert(p.velocity()==sign*config.max_velocity && p.turn()==sign*config.max_turn);
        assert(p.yawCorrection(99,BALANCE_YAW_SYNC_MAX)==-sign*config.max_turn);
        assert(std::fabs(p.armAssist())<.00001f); // relax during steady cruise
        // Actual motor signs, including balance-first clipping at the limits.
        auto cmd=balance_math::mix(0,-p.turn(),30);
        assert(cmd.left==sign*config.max_turn && cmd.right==-sign*config.max_turn);
        cmd=balance_math::mix(29.8f,-p.turn(),30);
        assert(std::fabs(cmd.left)<=30 && std::fabs(cmd.right)<=30);
        assert(std::fabs((cmd.left+cmd.right)*.5f-29.8f)<.00001f);
        // Cruising feedforward stays bounded; balance corrections retain the
        // common-channel cap and yaw only gets the remaining wheel headroom.
        cmd=balance_math::mix(balance_math::clamp(p.velocity()+29.8f,-30,30),-p.turn(),30);
        assert(std::fabs(cmd.left)<=30 && std::fabs(cmd.right)<=30);
        p.update(false,false,sign,sign,.02f,config); // fresh RC/feedback lost
        assert(sign*p.armAssist()>0); // brake: shift in the other direction
        assert(!p.ready() && p.moving());
        assert(p.fastBraking()); // input loss uses the faster ramp at high speed
        assert(std::fabs(p.velocity()-sign*(config.max_velocity-config.fast_deceleration*.02f))<.00001f);
        const int brake_ticks=static_cast<int>(std::ceil(config.max_velocity/(config.deceleration*.02f)))+1;
        for(int i=0;i<brake_ticks;++i) p.update(true,false,sign,sign,.02f,config);
        assert(!p.ready() && p.velocity()==0 && p.turn()==0);
        assert(p.moving()); // do not capture hold while still physically rolling
        for(int i=0;i<20;++i) p.update(true,true,0,0,.02f,config);
        assert(p.ready() && !p.moving());
        assert(std::fabs(correction(.5f)-.5f*BALANCE_VEL_SP_KP_LOW)<.00001f);
    }
    p.reset();unlock(p);
    p.update(true,true,0,1,.02f,config);
    assert(p.captureHeading());
    p.update(true,true,0,0,.02f,config);
    assert(p.turn()==0 && p.captureHeading()); // capture heading once as turn ends
    p.update(true,true,0,0,.02f,config);
    assert(!p.captureHeading()); // straight motion/hold keeps this new heading
    assert(p.yawCorrection(99,BALANCE_YAW_SYNC_MAX)==BALANCE_YAW_SYNC_MAX);
    assert(p.yawCorrection(-99,BALANCE_YAW_SYNC_MAX)==-BALANCE_YAW_SYNC_MAX);
    assert(p.yawCorrection(.4f,BALANCE_YAW_SYNC_MAX)==.4f);
    // Intended acceleration is not a disturbance; overspeed, wrong direction,
    // braking lag and stationary motion still request arm recovery.
    using balance_math::BalancePilot;
    assert(BalancePilot::armVelocityError(5,20)==0);
    assert(BalancePilot::armVelocityError(-5,-20)==0);
    assert(BalancePilot::armVelocityError(23,20)==3);
    assert(BalancePilot::armVelocityError(-23,-20)==-3);
    assert(BalancePilot::armVelocityError(-3,20)==-3);
    assert(BalancePilot::armVelocityError(3,-20)==3);
    assert(BalancePilot::armVelocityError(5,0)==5);
    assert(BalancePilot::armVelocityError(5,2)==3);
    for(float dt : {0.f,-1.f,.2f,NAN}) {
        p.reset();unlock(p);
        p.update(true,true,1,1,dt,config);
        assert(!p.ready() && p.velocity()==0 && p.turn()==0);
    }
    for(float bad : {NAN,INFINITY,-INFINITY}) {
        p.reset();unlock(p);
        p.update(true,true,bad,0,.02f,config);
        assert(!p.ready() && p.velocity()==0 && std::isfinite(p.forwardStick()));
    }
    std::mt19937 rng(11);std::uniform_real_distribution<float> input(-2,2);
    for(int i=0;i<50000;++i) {
        if(i%50==0) { p.reset(); unlock(p); }
        const float v=p.velocity(),y=p.turn();
        p.update(i%317!=0,false,input(rng),input(rng),.02f,config);
        assert(std::fabs(p.velocity())<=config.max_velocity+.00001f && std::fabs(p.turn())<=config.max_turn+.00001f);
        assert(std::fabs(p.velocity()-v)<=config.deceleration*.02f+.00001f);
        assert(std::fabs(p.turn()-y)<=config.turn_acceleration*.02f+.00001f);
        assert(std::fabs(p.armAssist())<=config.arm_limit+.000001f);
    }
    // Schema-2 prefix reads cannot consume bytes belonging to the next sample.
    BalanceSample old={};old.t_ms=123;old.yaw_corr=.75f;old.total_current=1.25f;
    uint8_t bytes[440]={};std::memcpy(bytes,&old,220);bytes[220]=0xA5;
    BalanceSample expanded={};std::memcpy(&expanded,bytes,220);
    assert(expanded.t_ms==123 && expanded.total_current==1.25f && expanded.yaw_corr==.75f);
    assert(expanded.pilot_forward==0 && expanded.pilot_flags==0);
    assert(balance_log::supported(2,220) && balance_log::supported(3,236));
    assert(!balance_log::supported(2,236) && !balance_log::supported(3,220));
    assert(!balance_log::supported(4,236));
    assert(balance_log::supported(4,240) && balance_log::supported(5,240));
    assert(!balance_log::supported(5,236) && !balance_log::supported(6,236));
    assert(balance_log::supported(6,240) && balance_log::supported(7,240) && balance_log::supported(8,240) && balance_log::supported(9,240) && balance_log::supported(10,240));
    assert(!balance_log::supported(7,236));
    assert(balance_log::supported(11,240));
    assert(!balance_log::supported(3,240) && !balance_log::supported(12,240));
    old.pilot_forward=.5f;old.pilot_flags=15;old.pilot_arm=.1f;
    expanded={};std::memcpy(&expanded,&old,236);
    assert(expanded.pilot_forward==.5f && expanded.pilot_flags==15 && expanded.pilot_arm==0);
    std::cout<<"Standing drive checks passed: gates, reversal, input loss, heading/turn limits, planned arms/recovery priority, invalid input, 50000 bounded updates, v2/v3/v4 layout\n";
}
