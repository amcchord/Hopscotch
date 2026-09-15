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
int main(int argc,char** argv) {tests();for(int i=1;i<argc;++i)replay(argv[i]);}
