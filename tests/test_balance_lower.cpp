#include "balance_lower.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
using namespace balance_math;

struct Rig {
    BalanceLower lower;
    LowerInput in{88, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true};
    uint32_t now = UINT32_MAX - 1000; // Exercise confirmation/deadlines through rollover.
    void tick(bool follow = true) {
        now += 20;
        lower.step(now, .02f, in);
        if (follow && lower.overridesArms()) {
            in.arm_left = lower.left(); in.arm_right = lower.right();
        }
    }
    void start() { assert(lower.request(now, in, 1.77f, -1.77f)); }
    void reach() {
        start();
        for (int i = 0; i < 25; ++i) tick();
        assert(lower.phase() == LowerPhase::Reaching);
    }
    void support() {
        reach();
        for (int i = 0; i < 360; ++i) tick();
        assert(in.arm_left < -1.7f && in.arm_right > 1.7f);
        in.torque_left = .8f; in.torque_right = -.8f;
        for (int i = 0; i < 50 && lower.phase() == LowerPhase::Reaching; ++i) tick(false);
        assert(lower.phase() == LowerPhase::Loading && !lower.supported());
        for (int i = 0; i < 40; ++i) tick();
        assert(!lower.supported()); // A timer/contact alone cannot release balance.
        in.tilt -= 3;
        for (int i = 0; i < 12; ++i) tick();
        assert(lower.phase() == LowerPhase::Descending && lower.supported());
    }
};

int main() {
    { Rig r; r.in.healthy = false; assert(!r.lower.request(r.now, r.in, 1.77f, -1.77f)); }
    for (float bad : {0.f, .5f, 4.f, std::numeric_limits<float>::quiet_NaN()}) {
        Rig r; assert(!r.lower.request(r.now, r.in, bad, -1.77f));
    }
    { Rig r; assert(!r.lower.request(r.now, r.in, 1.77f, 1.77f)); }
    { // Driving/braking must finish before arms reach; repeated CH11 cannot restart.
        Rig r; r.in.wheel_left = r.in.wheel_right = 10; r.start();
        for (int i = 0; i < 100; ++i) r.tick();
        assert(r.lower.phase() == LowerPhase::Stopping && !r.lower.overridesArms());
        assert(!r.lower.request(r.now, r.in, 1.77f, -1.77f));
        r.in.wheel_left = r.in.wheel_right = 0;
        for (int i = 0; i < 24; ++i) r.tick();
        assert(r.lower.phase() == LowerPhase::Stopping);
        r.tick(); assert(r.lower.phase() == LowerPhase::Reaching);
    }
    { // No floor contact: return arms and retain wheel balancing, never drop.
        Rig r; r.reach();
        for (int i = 0; i < 1500 && r.lower.active(); ++i) r.tick();
        assert(r.lower.phase() == LowerPhase::Complete && !r.lower.supported());
        assert(std::strcmp(r.lower.reason(), "lower_no_support") == 0);
    }
    for (int failure = 0; failure < 4; ++failure) {
        Rig r; r.reach();
        for (int i = 0; i < 360; ++i) r.tick();
        // Torque alone, one-arm contact, movement, or upright arm stall are
        // insufficient support evidence. Targets cannot run away from feedback.
        r.in.torque_left = r.in.torque_right = .8f;
        if (failure == 1) r.in.torque_right = 0;
        if (failure == 2) r.in.velocity_left = .3f;
        if (failure == 3) r.in.arm_left = r.in.arm_right = 0;
        for (int i = 0; i < 100; ++i) {
            r.tick(failure == 0);
            assert(!r.lower.supported());
            assert(std::fabs(r.lower.left() - r.in.arm_left) <= .12001f);
            assert(std::fabs(r.lower.right() - r.in.arm_right) <= .12001f);
        }
    }
    { Rig r; r.support();
      const float target = r.lower.left();
      r.in.rate = -20; r.tick(false); assert(r.lower.left() == target);
      r.in.rate = -70; r.tick(false); assert(r.lower.phase() == LowerPhase::Fault); }
    { Rig r; r.support(); r.in.healthy = false; r.tick();
      assert(r.lower.phase() == LowerPhase::Fault); }
    { Rig r; r.support(); r.in.rate = -30; r.in.torque_left = r.in.torque_right = 0;
      for (int i = 0; i < 5; ++i) r.tick(false);
      assert(r.lower.phase() == LowerPhase::Fault);
      assert(std::strcmp(r.lower.reason(), "lower_support_lost") == 0); }
    { Rig r; r.in.wheel_left = 20; r.start();
      for (int i = 0; i < 702; ++i) r.tick();
      assert(r.lower.phase() == LowerPhase::Complete && !r.lower.supported());
      assert(std::strcmp(r.lower.reason(), "lower_stop_timeout") == 0); }
    { // A calibration with opposite motor sign uses the same physical direction.
      Rig r; assert(r.lower.request(r.now, r.in, -1.77f, 1.77f));
      for (int i = 0; i < 100; ++i) r.tick();
      assert(r.lower.left() > 0 && r.lower.right() < 0); }
    { Rig r; assert(r.lower.request(r.now, r.in, 2.5f, -2.5f));
      for (int i = 0; i < 1200; ++i) {
          r.tick();
          assert(std::fabs(r.lower.left()) <= 2.60001f && std::fabs(r.lower.right()) <= 2.60001f);
      } }
    { Rig r; r.support(); r.in.arm_left = std::numeric_limits<float>::quiet_NaN(); r.tick(false);
      assert(r.lower.phase() == LowerPhase::Fault); }
    { Rig r; r.support();
      for (int i = 0; i < 151; ++i) r.tick();
      assert(r.lower.phase() == LowerPhase::Fault);
      assert(std::strcmp(r.lower.reason(), "lower_descent_timeout") == 0); }
    { // Complete only after measured flat dwell and arm retraction.
        Rig r; r.support();
        for (int i = 0; i < 400; ++i) {
            r.in.tilt = std::max(0.f, r.in.tilt - .2f);
            r.in.rate = r.in.tilt > 0 ? -10 : 0;
            r.tick();
        }
        r.in.tilt = 0; r.in.rate = 0;
        r.tick(); // Enter GroundHold, then require a full 600ms of quiet data.
        for (int i = 0; i < 29; ++i) r.tick();
        assert(r.lower.phase() == LowerPhase::GroundHold);
        r.tick(); assert(r.lower.phase() == LowerPhase::Retracting);
        for (int i = 0; i < 300 && r.lower.active(); ++i) r.tick();
        assert(r.lower.phase() == LowerPhase::Complete && r.lower.supported());
        assert(std::strcmp(r.lower.reason(), "lower_complete") == 0);
    }
    std::cout << "Lowering policy checks passed: calibrated direction, calm entry, two-arm resisted contact, no-contact cancellation, bounded targets, rate pause, feedback faults, deadlines/rollover and measured landing\n";
}
