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

    // An early positive catch cannot push the left arm beyond the tip pose;
    // clipping one shared fraction preserves the center-axis coupling.
    float f=balance_math::limitArmAssist(2.5f,1.8f,1.768f,-1.767f,2.71f,1.96f,.45f,-.3f,.45f);
    assert(f>.1f && f<.13f);
    assert(std::fabs(2.5f+f*1.768f-2.71f)<.00001f);
    assert(balance_math::limitArmAssist(0,0,1.768f,-1.767f,2.71f,1.96f,.45f,-.3f,.45f)==.45f);
    assert(balance_math::limitArmAssist(0,0,1.768f,-1.767f,2.71f,1.96f,-.3f,-.3f,.45f)==-.3f);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> unit(0,1),req(-1,1);
    for (int i=0;i<50000;++i) {
        const float tl=.5f+3*unit(rng),tr=.5f+3*unit(rng);
        const float cl=.3f+3*unit(rng),cr=-.3f-3*unit(rng);
        const float nl=unit(rng)*tl,nr=unit(rng)*tr;
        f=balance_math::limitArmAssist(nl,nr,cl,cr,tl,tr,req(rng),-.3f,.45f);
        assert(f>=-.30001f && f<=.45001f);
        assert(nl+f*cl >= std::fmin(0.f,-.3f*cl)-.00001f);
        assert(nl+f*cl <= std::fmax(tl,.45f*cl)+.00001f);
        assert(nr+f*cr >= std::fmin(0.f,.45f*cr)-.00001f);
        assert(nr+f*cr <= std::fmax(tr,-.3f*cr)+.00001f);
    }
    std::cout << "Startup recovery checks passed: signed confirmation, spike rejection, fresh timing, reset, 50000 coupled arm envelopes\n";
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
