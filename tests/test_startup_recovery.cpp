#include "balance_math.h"
#include "config.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

static const balance_math::RunawayConfig config = {
    BALANCE_START_RECOVERY_SPEED, BALANCE_START_RECOVERY_FORCE_SPEED,
    BALANCE_START_RECOVERY_ACCEL, BALANCE_START_RECOVERY_ACCEL_TAU,
    BALANCE_START_RECOVERY_CONFIRM_MS
};
static const balance_math::RecoilConfig recoil_config = {
    BALANCE_RECOIL_ENTER_SPEED, BALANCE_RECOIL_EXIT_SPEED, BALANCE_RECOIL_CONFIRM_MS,
    BALANCE_RECOIL_BLEND_MS, BALANCE_RECOIL_MULTIPLIER
};

static void recoil_tests() {
    balance_math::RecoilUnwind u;
    for (int sign : {-1,1}) {
        u.reset();
        for(int i=0;i<100;++i) assert(u.update(false,-sign,2*sign,.02f,recoil_config)==1);
        for(int i=0;i<100;++i) assert(u.update(true,sign,2*sign,.02f,recoil_config)==1);
        for(int i=0;i<100;++i) assert(u.update(true,-sign*.3f,2*sign,.02f,recoil_config)==1);
        // Alternating correction/motion signs cannot share a confirmation.
        for(int i=0;i<100;++i) {
            const int flip=i%2 ? -1 : 1;
            assert(u.update(true,-flip,2*flip,.02f,recoil_config)==1);
        }
        u.reset();
        assert(u.update(true,-sign,2*sign,.02f,recoil_config)==1);
        assert(u.update(true,-sign,2*sign,.02f,recoil_config)==1);
        float prior=1;
        for(int i=0;i<6;++i) {
            const float gain=u.update(true,-sign,2*sign,.02f,recoil_config);
            assert(gain>prior && gain-prior<=1.f/6+.00001f);
            prior=gain;
        }
        assert(std::fabs(prior-2)<.00001f);
        // Inside the hysteresis band, remain confirmed. Below exit, blend out.
        assert(u.update(true,-sign*.2f,2*sign,.02f,recoil_config)==2);
        const float fade=u.update(true,-sign*.1f,2*sign,.02f,recoil_config);
        assert(fade>1 && fade<2);
        for(int i=0;i<6;++i) u.update(true,-sign*.1f,2*sign,.02f,recoil_config);
        assert(u.update(true,-sign*.2f,2*sign,.02f,recoil_config)==1);
        for(int i=0;i<10;++i) u.update(true,-sign,2*sign,.02f,recoil_config);
        assert(u.update(true,sign,2*sign,.02f,recoil_config)==1); // no outward boost
        for(int i=0;i<10;++i) u.update(true,-sign,2*sign,.02f,recoil_config);
        assert(u.update(false,-sign,2*sign,.02f,recoil_config)==1); // settled/stale
        assert(u.update(true,-sign,2*sign,.02f,recoil_config)==1); // reconfirm
        assert(u.update(true,-sign,2*sign,.2f,recoil_config)==1); // long tick
        assert(u.update(true,-sign,2*sign,.02f,recoil_config)==1);
        assert(u.update(true,NAN,2*sign,.02f,recoil_config)==1);
        assert(u.update(true,-sign,NAN,.02f,recoil_config)==1);
        assert(u.update(true,-sign,2*sign,NAN,recoil_config)==1);
        assert(u.update(true,-sign,0,.02f,recoil_config)==1);
    }
    // Boosted updates can reach zero sooner, but only ordinary integration
    // can accumulate an opposite correction. Sign/rate/angle limits still hold.
    auto next=balance_math::recoveryIntegral(.015f,-1,.5f,.02f,0,6,6,true,2);
    assert(std::fabs(next.value)<.000001f);
    next=balance_math::recoveryIntegral(.005f,-1,.5f,.02f,0,6,6,true,2);
    assert(std::fabs(next.value+.005f)<.000001f);
    assert(balance_math::recoveryIntegral(2,1,.5f,.02f,0,6,6,true,2).value==2.01f);
    assert(balance_math::recoveryIntegral(2,-1,.5f,.02f,0,6,6,false,2).value==2);
    std::mt19937 rng(9);
    std::uniform_real_distribution<float> value(-30,30);
    for(int i=0;i<50000;++i) {
        const float current=balance_math::clamp(value(rng),-6,6), v=value(rng);
        const auto normal=balance_math::recoveryIntegral(current,v,.231f,.02f,0,6,6,true);
        const auto fast=balance_math::recoveryIntegral(current,v,.231f,.02f,0,6,6,true,2);
        assert(std::fabs(fast.value)<=6.00001f);
        assert(std::fabs(fast.value-current)<=.12001f);
        if(current*v>=0) assert(fast.value==normal.value);
        else assert(std::fabs(fast.value)<=std::fabs(normal.value)+.00001f);
    }
    std::cout << "Recoil release checks passed: both directions, confirmation, hysteresis, blend, stale/settled reset, no extra opposite windup, 50000 bounded updates\n";
}

static void tests() {
    balance_math::RunawayDetector d;
    for (int i=0;i<1000;++i) assert(d.update(true, i%2 ? 1.0f : -1.0f,.02f,config)==0);
    assert(d.update(true, 12,.02f,config)==0); // single spike
    assert(d.update(true, 0,.02f,config)==0);
    for (int i=0;i<100;++i) assert(d.update(false, 10,.02f,config)==0);
    for (int sign : {-1,1}) {
        d.reset();
        int trigger_count=0;
        for (int i=0;i<50;++i) {
            const float result=d.update(true, sign*.2f*i,.02f,config);
            if (result) { assert(result==sign); assert(i<20); ++trigger_count; }
        }
        assert(trigger_count==1);
    }
    d.reset();
    assert(d.update(true, 4.5f,.02f,config)==0);
    assert(d.update(true, 4.5f,.02f,config)==0);
    assert(d.update(true, 4.5f,.02f,config)==1); // high steady-speed fallback
    d.reset();
    for (int i=0;i<1000;++i) assert(d.update(true, i%2 ? 5 : -5,.02f,config)==0);
    d.reset();
    assert(d.update(true,5,.02f,config)==0);
    assert(d.update(true,5,.2f,config)==0); // stale timing resets confirmation
    assert(d.update(true,5,.02f,config)==0);
    assert(d.update(true,std::numeric_limits<float>::quiet_NaN(),.02f,config)==0);
    assert(d.update(true,5,.02f,config)==0);
    assert(d.update(true,5,.02f,config)==0);
    assert(d.update(true,5,.02f,config)==1);

    // Boost expires at the earlier of timeout and ramp completion, including
    // clock rollover. Calm confirmation cannot bridge disturbances or stalls.
    balance_math::StartupRecovery recovery;
    assert(!recovery.active() && !recovery.triggered());
    assert(recovery.start(UINT32_MAX-100));
    assert(!recovery.start(50));
    assert(recovery.boosting(100,false,800));
    assert(!recovery.boosting(100,true,800));
    assert(!recovery.boosting(699,false,800));
    for(int i=0;i<10;++i) assert(!recovery.settle(true,.02f,400));
    assert(!recovery.settle(false,.02f,400));
    for(int i=0;i<19;++i) assert(!recovery.settle(true,.02f,400));
    assert(!recovery.settle(true,.2f,400));
    for(int i=0;i<19;++i) assert(!recovery.settle(true,.02f,400));
    assert(recovery.settle(true,.02f,400));
    assert(!recovery.active() && recovery.triggered());
    assert(!recovery.start(1000));
    recovery.reset();
    assert(!recovery.triggered() && recovery.start(1000));

    const auto integrate=[](float current,float velocity,float offset,bool fresh=true) {
        return balance_math::recoveryIntegral(current,velocity,1,.02f,offset,6,6,fresh);
    };
    assert(std::fabs(integrate(0,20,0).value-.12f)<.00001f);
    assert(integrate(0,20,0).limited);
    assert(integrate(2,3,6).value==2); // saturated target cannot wind up
    assert(integrate(-2,-3,-6).value==-2);
    assert(integrate(2,-3,6).value<2); // opposite travel unwinds
    assert(integrate(-2,3,-6).value>-2);
    assert(integrate(5.99f,20,0).value==6);
    assert(integrate(2,20,0,false).value==2);
    assert(integrate(2,std::numeric_limits<float>::quiet_NaN(),0).value==2);
    assert(balance_math::recoveryIntegral(2,3,1,.2f,0,6,6,true).value==2);
    // The boost-to-normal transition changes only the gain, never the learned
    // state. Sign changes remain rate bounded instead of resetting the catch.
    float integral=2;
    auto normal=balance_math::recoveryIntegral(integral,0,.231f,.02f,2,6,6,true);
    assert(normal.value==integral);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> value(-30,30);
    for(int i=0;i<50000;++i) {
        const float before=integral;
        auto next=integrate(integral,value(rng),std::fmax(-6.f,std::fmin(6.f,value(rng))));
        assert(std::fabs(next.value)<=6.00001f);
        assert(std::fabs(next.value-before)<=.12001f);
        integral=next.value;
    }
    std::cout << "Startup recovery checks passed: signed trigger, one-shot lifecycle, rollover, calm hysteresis, stale input, anti-windup, 50000 bounded updates\n";
}
static std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> result;std::stringstream s(text);std::string item;
    while(std::getline(s,item,',')) result.push_back(item);
    return result;
}
static void replay(const char* path) {
    std::ifstream input(path);assert(input.good());
    std::unordered_map<std::string,size_t> columns;std::string line;
    balance_math::RunawayDetector detector;
    double start=-1;bool found=false;
    while(std::getline(input,line)) {
        if(line.rfind("t_ms,",0)==0) {
            auto names=split(line);for(size_t i=0;i<names.size();++i)columns[names[i]]=i;
            continue;
        }
        if(columns.empty() || line.empty() || line[0]=='#')continue;
        const auto fields=split(line);if(fields.size()<columns.size())continue;
        const auto v=[&](const char* name) {return std::stod(fields.at(columns.at(name)));};
        if(v("state")!=2)continue;
        if(start<0)start=v("t_ms");
        const bool eligible=(int(v("flags"))&0x20) && !(int(v("flags"))&0x40)
            && v("arm_tip_frac")<=BALANCE_START_RECOVERY_TIP_MAX
            && v("feedback_age_l_ms")<=BALANCE_START_RECOVERY_FEEDBACK_MS
            && v("feedback_age_r_ms")<=BALANCE_START_RECOVERY_FEEDBACK_MS;
        float sign=detector.update(eligible,v("filtered_vel"),v("sample_dt_ms")/1000,config);
        if(sign) {
            std::cout << path << " trigger_s=" << (v("t_ms")-start)/1000
                << " filtered_velocity=" << v("filtered_vel") << " displacement=" << v("meas_drift")
                << " tip_fraction=" << v("arm_tip_frac") << " direction=" << sign << '\n';
            found=true;break;
        }
    }
    if(!found)std::cout<<path<<" no trigger\n";
}
static void replay_recoil(const char* path) {
    std::ifstream input(path);assert(input.good());
    std::unordered_map<std::string,size_t> columns;std::string line;
    balance_math::RecoilUnwind release;
    double start=-1, first=-1, last=-1, peak=1;int count=0;
    while(std::getline(input,line)) {
        if(line.rfind("t_ms,",0)==0) {
            auto names=split(line);for(size_t i=0;i<names.size();++i)columns[names[i]]=i;
            continue;
        }
        if(columns.empty() || line.empty() || line[0]=='#')continue;
        const auto fields=split(line);if(fields.size()<columns.size())continue;
        const auto v=[&](const char* name) {return std::stod(fields.at(columns.at(name)));};
        if(v("state")!=2)continue;
        if(start<0)start=v("t_ms");
        const bool eligible=(int(v("flags"))&0x40) && (int(v("diag_flags"))&0x1000)
            && v("feedback_age_l_ms")<=BALANCE_START_RECOVERY_FEEDBACK_MS
            && v("feedback_age_r_ms")<=BALANCE_START_RECOVERY_FEEDBACK_MS;
        const float gain=release.update(eligible,v("filtered_vel"),v("vel_integral"),
                                        v("sample_dt_ms")/1000,recoil_config);
        if(gain>1) {
            assert(eligible && v("filtered_vel")*v("vel_integral")<0);
            const double t=(v("t_ms")-start)/1000;
            if(first<0)first=t;
            last=t;++count;peak=std::fmax(peak,gain);
        }
    }
    std::cout<<path<<" observed-input recoil replay: first_s="<<first<<" last_s="<<last
             <<" samples="<<count<<" max_multiplier="<<peak<<" (not predicted motion)\n";
}
int main(int argc,char** argv) {
    tests();recoil_tests();
    for(int i=1;i<argc;++i) {replay(argv[i]);replay_recoil(argv[i]);}
}
