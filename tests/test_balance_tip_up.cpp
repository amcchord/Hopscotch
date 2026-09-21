#include "balance_tip_up.h"
#include "balance_math.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using balance_math::FastTipUp;
using balance_math::TipInput;

static TipInput flat() { TipInput in; in.healthy = true; return in; }
static void track(TipInput& in, const FastTipUp& tip, float dt, float tau, float cap) {
    const auto velocity = [=](float target, float position) {
        return std::fmax(-cap, std::fmin(cap, (target - position) / std::fmax(dt, tau)));
    };
    in.left_velocity = velocity(tip.left(), in.left);
    in.right_velocity = velocity(tip.right(), in.right);
    in.left += in.left_velocity * dt;
    in.right += in.right_velocity * dt;
    // Only an arrival fixture. This does not simulate body/contact dynamics.
    in.tilt = 83 * .5f * (in.left / BALANCE_ARM_TIP_LEFT + in.right / BALANCE_ARM_TIP_RIGHT);
    in.rate = 0;
}

int main(int argc, char** argv) {
    const bool dump = argc == 2 && std::strcmp(argv[1], "--dump") == 0;
    if (dump) {
        std::cout << "t_s,left,right,left_velocity,right_velocity,ready\n";
        FastTipUp tip; auto in = flat(); assert(tip.begin(0, in));
        for (uint32_t now=20; now<=3000; now+=20) {
            const float l = tip.left(), r = tip.right();
            tip.step(now,.02f,in);
            assert(!tip.fault());
            std::cout << now*.001f << ',' << tip.left() << ',' << tip.right() << ','
                << (tip.left()-l)/.02f << ',' << (tip.right()-r)/.02f << ',' << tip.ready() << '\n';
            track(in,tip,.02f,.04f,BALANCE_FAST_TIP_MOTOR_RAD_S);
        }
        return 0;
    }

    for (float v : {-1.0f, 0.0f, .5f, std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()}) assert(!balance_math::fastTipSelected(v));
    assert(balance_math::fastTipSelected(.501f));
    assert(balance_math::fastTipSelected(1));

    // Integration regression: wheel setup blocked the feedback owner long
    // enough that the old immediate begin() rejected a healthy stationary pose.
    // Resume the owner, poll two motors/tick, then require continuously fresh
    // real samples and a quiet pose before the first trajectory command.
    for (uint32_t origin : {0U, UINT32_MAX-300U}) {
        FastTipUp old_attempt; auto measured=flat(); measured.healthy=false;
        assert(!old_attempt.begin(origin+140,measured));
        balance_math::FastTipStart start; start.request(origin+140);
        uint32_t sampled[6]={origin,origin,origin,origin,origin,origin};
        bool ready=false;
        for (uint32_t elapsed=20;elapsed<=240;elapsed+=20) {
            const uint32_t now=origin+140+elapsed;
            measured.healthy=true;
            for (uint32_t sample : sampled) measured.healthy &= uint32_t(now-sample)<=100;
            ready=start.step(now,measured);
            if (elapsed<180) assert(!ready);
            assert(!start.failed());
            // Replies are processed AFTER this iteration, not fabricated when
            // the request is enqueued. The next tick sees these samples.
            const int first=((elapsed/20-1)*2)%6;
            sampled[first]=sampled[first+1]=now;
        }
        assert(ready);
        FastTipUp restarted;
        assert(restarted.begin(origin+380,measured));
        assert(restarted.left()==0 && restarted.right()==0 && !restarted.ready());
    }

    // Never proceed with absent feedback, wrong pose or unstable motion, and
    // do not remain pending forever. A new request explicitly resets failure.
    for(int failure=0;failure<5;++failure) {
        balance_math::FastTipStart start; start.request(0); auto measured=flat();
        for(uint32_t now=20;now<=1000;now+=20) {
            measured=flat();
            if(failure==0) measured.healthy=false;
            if(failure==1) measured.tilt=14;
            if(failure==2) measured.left=.2f;
            if(failure==3) measured.left_velocity=.4f;
            if(failure==4 && (now/20)%3==0) measured.right_velocity=.4f;
            assert(!start.step(now,measured));
        }
        assert(start.failed());
        assert(std::strstr(start.status(), "blocked") != nullptr);
        assert(!start.step(1020,flat()));
        start.request(1100);
        for(uint32_t now=1120;now<1220;now+=20) assert(!start.step(now,flat()));
        assert(start.step(1220,flat()));
    }

    // The fast mode refuses partial/manual arm poses and movement at startup.
    for (int kind=0; kind<10; ++kind) {
        FastTipUp tip; auto in=flat();
        switch(kind) {
            case 0: in.healthy=false; break;
            case 1: in.left=.16f; break;
            case 2: in.right=-.16f; break;
            case 3: in.tilt=13; break;
            case 4: in.rate=9; break;
            case 5: in.left_velocity=.31f; break;
            case 6: in.right_velocity=-.31f; break;
            case 7: in.wheel_left=.76f; break;
            case 8: in.wheel_right=-.76f; break;
            case 9: in.left=std::numeric_limits<float>::quiet_NaN(); break;
        }
        assert(!tip.begin(0,in)); assert(!tip.ready());
    }

    int completed=0;
    float max_speed=0, max_acceleration=0;
    uint32_t max_capture_ms=0;
    // Independently vary arm pose, motor lag, cadence and clock wraparound.
    for (float start_l : {-.14f,0.0f,.14f})
    for (float start_r : {-.14f,0.0f,.14f})
    for (float tau : {.02f,.04f,.06f})
    for (uint32_t cadence : {10U,20U,30U})
    for (uint32_t start : {0U,UINT32_MAX-1200U}) {
        FastTipUp tip; auto in=flat(); in.left=start_l; in.right=start_r;
        assert(tip.begin(start,in));
        float last_vl=0,last_vr=0;
        const float dt=cadence*.001f;
        for(uint32_t elapsed=cadence;elapsed<4400;elapsed+=cadence) {
            const float old_l=tip.left(),old_r=tip.right();
            tip.step(start+elapsed,dt,in);
            assert(!tip.fault());
            assert(tip.left()>=old_l-2e-6f && tip.right()>=old_r-2e-6f);
            assert(tip.left()<=BALANCE_ARM_TIP_LEFT && tip.right()<=BALANCE_ARM_TIP_RIGHT);
            const float vl=(tip.left()-old_l)/dt,vr=(tip.right()-old_r)/dt;
            max_speed=std::fmax(max_speed,std::fmax(vl,vr));
            max_acceleration=std::fmax(max_acceleration,std::fmax(std::fabs(vl-last_vl),std::fabs(vr-last_vr))/dt);
            last_vl=vl; last_vr=vr;
            assert(vl<=BALANCE_FAST_TIP_MOTOR_RAD_S && vr<=BALANCE_FAST_TIP_MOTOR_RAD_S);
            assert(std::fabs(tip.left()-in.left)<=BALANCE_FAST_TIP_LEAD_RAD+1e-6f);
            assert(std::fabs(tip.right()-in.right)<=BALANCE_FAST_TIP_LEAD_RAD+1e-6f);
            if(tip.ready()) {
                assert(elapsed>=2700 && elapsed<3300);
                max_capture_ms=std::max(max_capture_ms,elapsed); ++completed; break;
            }
            track(in,tip,dt,tau,BALANCE_FAST_TIP_MOTOR_RAD_S);
        }
    }
    assert(completed==162);

    // Readiness must be requalified if the caller delays the handoff and the
    // body moves again; a once-quiet pose is not permanent permission to engage.
    {
        FastTipUp delayed; auto measured=flat(); assert(delayed.begin(0,measured));
        for(uint32_t now=20;now<=2800;now+=20) {
            delayed.step(now,.02f,measured); track(measured,delayed,.02f,.04f,2.2f);
        }
        assert(delayed.ready());
        measured.rate=20;
        delayed.step(2820,.02f,measured);
        assert(!delayed.ready() && !delayed.fault());
        measured.healthy=false;
        delayed.step(2840,.02f,measured);
        assert(delayed.fault() && !delayed.ready());
    }

    // A late body rebound or moving arm cannot qualify a capture based only on
    // generated targets. When motion stops, require a NEW continuous quiet dwell.
    for(int moving=0;moving<7;++moving) {
        FastTipUp tip; auto in=flat(); assert(tip.begin(0,in));
        for(uint32_t now=20;now<=3400;now+=20) {
            track(in,tip,.02f,.04f,2.2f);
            in.wheel_left = in.wheel_right = 0;
            if(now>2500 && now<3100) {
                if(moving==0) in.rate=9;
                if(moving==1) in.left_velocity=.31f;
                if(moving==2) in.right_velocity=-.31f;
                if(moving==3) in.wheel_left=.76f;
                if(moving==4) in.wheel_right=-.76f;
                if(moving==5) in.wheel_left=in.wheel_right=5;
                if(moving==6) { in.wheel_left=5; in.wheel_right=-5; }
            }
            tip.step(now,.02f,in);
            assert(!tip.fault());
            if(now<3220) assert(!tip.ready());
        }
        assert(tip.ready());
    }

    // Rolling never acquires capture, even with quiet body/arms, and it still
    // reaches the original deadline. The mean speed must not hide counterspin.
    for(float right : {-5.0f, 5.0f}) {
        FastTipUp tip; auto in=flat(); assert(tip.begin(0,in));
        for(uint32_t now=20;now<=4520;now+=20) {
            track(in,tip,.02f,.04f,2.2f);
            if(now>2400) { in.wheel_left=5; in.wheel_right=right; }
            tip.step(now,.02f,in);
            assert(!tip.ready());
            if(now<=4500) assert(!tip.fault());
        }
        assert(std::strcmp(tip.fault(),"fast_tip_timeout")==0);
    }

    // Recorded fast trial: supported 82.456-degree capture erased 2.86 degrees
    // of the saved 3.2398-degree equilibrium correction. Release must restore
    // that saved reference, not carry the support angle to Forward forever.
    {
        constexpr float tilt=82.456f, scheduled=82.0762f, stored=3.2398f;
        const float initial_shift=tilt-scheduled-stored;
        const auto fast=balance_math::captureOffsets(true,tilt,scheduled,stored,8,initial_shift);
        const auto slow=balance_math::captureOffsets(false,tilt,scheduled,stored,8,initial_shift);
        assert(std::fabs(slow.curve+2.86f)<.00002f && slow.transient==0);
        assert(fast.curve==0 && fast.transient==initial_shift);
        balance_math::FastTipRelease release;
        assert(release.weight(false,false,1,.95f)==1); // hold until return owns arms
        assert(release.weight(true,false,1,1)==1); // targets alone do not release
        float previous=1;
        for(float fraction : {1.f,.999f,.992f,.984f,.975f,.966f,.955f,.943f,.933f,.923f,.912f,.902f,.891f}) {
            const float weight=release.weight(true,false,1,fraction);
            assert(weight>=0 && weight<=previous); previous=weight;
        }
        assert(previous==0);
        assert(release.weight(true,false,1,.99f)==0); // bounce cannot reapply bias
        assert(release.weight(true,true,1,1)==0); // assistance cannot reapply it
        assert(std::fabs(84+stored+fast.curve+fast.transient*previous-87.2398f)<.00002f);
        release.reset();
        assert(std::fabs(release.weight(true,false,1,.95f)-.5f)<.00001f);
        assert(std::fabs(release.weight(true,false,1,.96f)-.5f)<.00001f);
        assert(std::fabs(release.weight(true,false,1,std::numeric_limits<float>::quiet_NaN())-.5f)<.00001f);
        assert(release.weight(false,true,1,.95f)==0);
    }

    // Slow capture retains the exact prior absolute-trim arithmetic, including
    // clipping and both signs of saved trim. Fast never writes a new curve.
    for(float tilt : {60.f,78.1f,82.456f,100.f})
    for(float stored : {-8.f,-2.f,0.f,3.2398f,8.f}) {
        const auto slow=balance_math::captureOffsets(false,tilt,82.1f,stored,8,2);
        assert(slow.curve==balance_math::captureCurveShift(tilt,82.1f,stored,8));
        assert(slow.transient==0);
        const auto fast=balance_math::captureOffsets(true,tilt,82.1f,stored,8,2);
        assert(fast.curve==0 && fast.transient==2);
    }

    // Stuck arms: bound target lead, keep the other arm on the shared path,
    // fault promptly, and latch that fault even when feedback later recovers.
    for(int frozen=0;frozen<3;++frozen) {
        FastTipUp tip; auto in=flat(); assert(tip.begin(0,in));
        uint32_t fault_ms=0;
        for(uint32_t now=20;now<4500;now+=20) {
            tip.step(now,.02f,in);
            if(tip.fault()) { fault_ms=now; break; }
            assert(!tip.ready());
            track(in,tip,.02f,.04f,2.2f);
            if(frozen!=1) in.left=0;
            if(frozen!=0) in.right=0;
        }
        assert(fault_ms>400 && fault_ms<2000);
        assert(std::strcmp(tip.fault(),"fast_tip_arm_tracking")==0);
        const auto left=tip.left(),right=tip.right();
        tip.step(fault_ms+20,.02f,flat());
        assert(tip.left()==left && tip.right()==right && !tip.ready());
    }

    for(int failure=0;failure<7;++failure) {
        FastTipUp tip; auto in=flat(); assert(tip.begin(0,in));
        uint32_t now=20; float dt=.02f;
        switch(failure) {
            case 0: in.healthy=false; break;
            case 1: in.rate=101; break;
            case 2: in.tilt=101; break;
            case 3: in.tilt=-21; break;
            case 4: in.right=std::numeric_limits<float>::quiet_NaN(); break;
            case 5: dt=.041f; break;
            case 6: now=41; break; // actual owner gap despite a nominal fixed dt
        }
        tip.step(now,dt,in); assert(tip.fault() && !tip.ready());
    }

    // Endpoint without a valid upright pose times out; no time-only engagement.
    FastTipUp tip; auto in=flat(); assert(tip.begin(0,in));
    for(uint32_t now=20;now<=4520;now+=20) {
        tip.step(now,.02f,in);
        track(in,tip,.02f,.04f,2.2f); in.tilt=50;
    }
    assert(tip.fault() && std::strcmp(tip.fault(),"fast_tip_timeout")==0 && !tip.ready());
    // A new deliberate attempt resets the latch; never automatic restart.
    assert(tip.begin(5000,flat()) && !tip.fault());
    std::cout << "Fast tip-up: " << completed << " tracking cases; max capture " << max_capture_ms
        << " ms, target speed " << max_speed << " rad/s, acceleration " << max_acceleration
        << " rad/s^2; input, capture, obstruction, timing and fault checks pass\n";
}
