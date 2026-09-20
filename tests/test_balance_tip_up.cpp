#include "balance_tip_up.h"
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
    for(int moving=0;moving<3;++moving) {
        FastTipUp tip; auto in=flat(); assert(tip.begin(0,in));
        for(uint32_t now=20;now<=3400;now+=20) {
            track(in,tip,.02f,.04f,2.2f);
            if(now>2500 && now<3100) {
                if(moving==0) in.rate=9;
                if(moving==1) in.left_velocity=.31f;
                if(moving==2) in.right_velocity=-.31f;
            }
            tip.step(now,.02f,in);
            assert(!tip.fault());
            if(now<3220) assert(!tip.ready());
        }
        assert(tip.ready());
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
