#include "balance_lower.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
using namespace balance_math;

struct Rig {
    BalanceLower lower;
    LowerInput in{88,0,0,0,0,0,0,0,0,0,0,true};
    uint32_t now = UINT32_MAX-1000; // Run confirmation/deadlines through rollover.
    void tick(bool follow=true) {
        now+=20;
        lower.step(now,.02f,in);
        if (follow && lower.overridesArms()) { in.arm_left=lower.left(); in.arm_right=lower.right(); }
    }
    void start() { assert(lower.request(now,in,1.768f,-1.767f)); }
    void prepare() {
        start();
        for(int i=0;i<25;++i) tick();
        assert(lower.phase()==LowerPhase::Reaching && !lower.committed());
    }
    void commit() {
        prepare();
        for(int i=0;i<400 && !lower.committed();++i) tick();
        assert(lower.phase()==LowerPhase::Committing && lower.committed() && !lower.supported());
        assert(in.arm_left<=-1.71f && in.arm_right>=1.71f);
    }
    void fall() {
        commit();
        in.tilt-=.2f; in.rate=-1.1f; tick();
        in.tilt-=1.1f; in.rate=-8;
        for(int i=0;i<27;++i) tick();
        assert(lower.phase()==LowerPhase::Catching && !lower.supported());
        assert(lower.wheelCommand()<0 && lower.wheelCommand()>=-2);
    }
    void catchFall() {
        fall();
        in.torque_left=.6f; in.torque_right=-.6f; in.rate=-6; tick();
        const float caught_left=lower.left(),caught_right=lower.right();
        in.rate=-2; in.torque_left=.3f; in.torque_right=-.3f;
        for(int i=0;i<2;++i) tick();
        assert(!lower.supported());
        tick();
        assert(lower.phase()==LowerPhase::Descending && lower.supported());
        assert(lower.armSpeed()==.3f);
        assert(lower.left()>=caught_left && lower.left()<=caught_left+.061f);
        assert(lower.right()<=caught_right && lower.right()>=caught_right-.061f);
    }
};

int main() {
    const float nan=std::numeric_limits<float>::quiet_NaN();
    { Rig r; r.in.healthy=false; assert(!r.lower.request(r.now,r.in,1.77f,-1.77f)); }
    for(float bad:{0.f,.5f,4.f,nan}) { Rig r; assert(!r.lower.request(r.now,r.in,bad,-1.77f)); }
    { Rig r; assert(!r.lower.request(r.now,r.in,1.77f,1.77f)); }
    { Rig r; r.in.wheel_left=r.in.wheel_right=10; r.start();
      for(int i=0;i<100;++i) r.tick();
      assert(r.lower.phase()==LowerPhase::Stopping && !r.lower.overridesArms());
      assert(!r.lower.request(r.now,r.in,1.77f,-1.77f));
      r.in.wheel_left=r.in.wheel_right=0;
      for(int i=0;i<24;++i) r.tick();
      assert(!r.lower.committed()); r.tick(); assert(r.lower.phase()==LowerPhase::Reaching); }
    { // Deliberate departure precedes contact. The v1 contact-first deadlock
      // must never return: quiet measured preparation is enough to commit.
      Rig r; r.commit(); assert(r.in.torque_left==0 && r.in.torque_right==0); }
    { // The recorded 4.46-degree backwards target must never reach the PD
      // while arms deploy. Outside this phase ordinary control is unchanged.
      Rig r; assert(r.lower.preparationSetpoint(92)==92); r.prepare();
      assert(r.lower.preparationSetpoint(92.46f)==88);
      assert(r.lower.preparationSetpoint(87)==87);
      r.in.rate=-8; r.in.velocity_left=-4; r.in.velocity_right=4;
      for(int i=0;i<30 && !r.lower.committed();++i) r.tick();
      assert(r.lower.committed()); // no upright/stationary wait during a forward fall
      assert(r.lower.preparationSetpoint(92)==92); }
    { // Pause a backwards disturbance, continue an expected forward departure.
      Rig r; r.prepare(); r.tick(); const float held=r.lower.left();
      r.in.rate=5; r.tick(); assert(r.lower.left()==held);
      r.in.rate=-8; r.tick(); assert(r.lower.left()<held); }
    { // No multi-degree backward excursion is accepted in preparation.
      Rig r; r.prepare(); r.in.tilt+=1.6f; r.tick();
      assert(r.lower.phase()==LowerPhase::Fault); }
    { // The catch first parks short of the recorded impact pose. A slow
      // extended search is permitted only after six measured forward degrees.
      Rig r; r.fall(); assert(std::fabs(r.lower.left())<=1.851f);
      r.in.tilt=81; r.tick(); assert(r.lower.armSpeed()==.5f);
      assert(r.lower.left() < -1.85f); }
    { // Arm acceleration torque alone cannot qualify contact without observed
      // forward departure and body deceleration.
      Rig r; r.fall(); r.in.torque_left=r.in.torque_right=.8f;
      r.in.rate=-9; r.tick(); assert(r.lower.armSpeed()==2.f && !r.lower.supported()); }
    { Rig r; r.fall(); const float command=r.lower.wheelCommand();
      for(int i=0;i<10;++i) r.tick(); assert(r.lower.wheelCommand()==command); }
    { // Forward fall is measured, never assumed after a timer or wheel request.
      Rig r; r.commit(); r.in.torque_left=r.in.torque_right=1;
      for(int i=0;i<100 && r.lower.active();++i) r.tick();
      assert(r.lower.phase()==LowerPhase::Fault && !r.lower.supported());
      assert(std::strcmp(r.lower.reason(),"lower_no_forward_fall")==0); }
    { // No catch: bounded launch then timeout, with arms retained (not retracted).
      Rig r; r.fall(); float left=r.lower.left();
      for(int i=0;i<150 && r.lower.active();++i) r.tick();
      assert(r.lower.phase()==LowerPhase::Fault && !r.lower.supported());
      assert(std::strcmp(r.lower.reason(),"lower_missed_catch")==0);
      assert(r.lower.left()<=left && r.lower.wheelCommand()==0); }
    { Rig r; r.fall(); r.in.torque_left=.8f; r.in.torque_right=0; r.in.rate=-1;
      for(int i=0;i<20;++i) r.tick(); assert(!r.lower.supported()); }
    { // Load without slowing the falling body is not a catch.
      Rig r; r.fall(); r.in.torque_left=r.in.torque_right=.8f; r.in.rate=-30;
      for(int i=0;i<20;++i) r.tick(); assert(!r.lower.supported()); }
    { // Hold an arm at the first impact. Do not require it to stall with a
      // growing target error or keep pushing it through the floor.
      Rig r; r.fall(); r.in.torque_left=.8f; r.in.velocity_left=.4f; r.in.rate=-6; r.tick();
      float left=r.lower.left(),right=r.lower.right(); r.in.torque_left=.3f;
      for(int i=0;i<10;++i) r.tick();
      assert(r.lower.left()>=left && r.lower.left()<=left+.061f);
      assert(r.lower.right()<=right && r.lower.right()>=right-.061f);
      assert(!r.lower.supported()); } // One loaded arm cannot qualify support.
    { // Regression: measured first v2 impact. Right load was just under 0.4Nm;
      // it must stop advancing with the left rather than pushing another frame.
      Rig r; r.fall(); r.in.tilt=82.511f; r.in.rate=-38.43f; r.tick(false);
      r.in.arm_left=-1.922f; r.in.arm_right=1.912f;
      r.in.velocity_left=-1.31f; r.in.velocity_right=1.349f;
      r.in.torque_left=-.525f; r.in.torque_right=.385f; r.in.rate=-28.789f;
      r.tick(false);
      assert(r.lower.left()>=r.in.arm_left && r.lower.left()<=r.in.arm_left+.061f);
      assert(r.lower.right()<=r.in.arm_right && r.lower.right()>=r.in.arm_right-.061f);
      assert(!r.lower.supported()); }
    { Rig r; r.commit(); r.in.tilt+=3; r.tick();
      assert(r.lower.phase()==LowerPhase::Fault && !r.lower.supported());
      assert(std::strcmp(r.lower.reason(),"lower_wrong_direction")==0); }
    { // Recorded v3 impact and next frame. Loaded rebound must continue the
      // return instead of freezing upright. It is NOT completed or supported
      // until measured two-arm return and calm body support qualify.
      Rig r; r.fall(); r.in.tilt=82; r.in.rate=-35; r.tick(false);
      r.in.arm_left=-1.902f; r.in.arm_right=1.893f;
      r.in.velocity_left=-1.272f; r.in.velocity_right=1.328f;
      r.in.torque_left=-.454f; r.in.torque_right=.189f; r.in.rate=-11.405f;
      r.tick(false); const float first=r.lower.left();
      assert(first>r.in.arm_left && r.lower.right()<r.in.arm_right);
      r.in.arm_left=-1.927f; r.in.arm_right=1.918f;
      r.in.velocity_left=-1.232f; r.in.velocity_right=1.295f;
      r.in.torque_left=-.918f; r.in.torque_right=.921f;
      r.in.tilt=82.558f; r.in.rate=45.776f; r.tick(false);
      assert(r.lower.active() && r.lower.left()>first && !r.lower.supported());
      assert(r.lower.armSpeed()==.5f);
      r.in.rate=0; r.in.velocity_left=.25f; r.in.velocity_right=-.25f;
      for(int i=0;i<20 && r.lower.active();++i) r.tick();
      assert(r.lower.supported() && r.lower.phase()==LowerPhase::Descending);
      assert(r.lower.left()>first+.06f); // full return, not v3 capped retreat
      assert(r.lower.phase()!=LowerPhase::Complete && r.in.tilt>80); }
    { // Rebound without load, persistent rebound, and backwards travel still
      // fault. Contact is no excuse to waive the global 65 deg/s limit.
      for(int fault=0;fault<4;++fault) {
        Rig r; r.fall(); r.in.torque_left=.8f; r.in.torque_right=-.8f;
        r.in.rate=0; r.tick(false);
        r.in.rate=45;
        if(fault==0) r.in.torque_left=r.in.torque_right=0;
        if(fault==1) r.now+=301;
        if(fault==2) r.in.tilt=91;
        if(fault==3) r.in.rate=66;
        for(int i=0;i<4 && r.lower.active();++i) r.tick(false);
        assert(r.lower.phase()==LowerPhase::Fault);
      } }
    { Rig r; r.prepare(); r.in.wheel_left=-6.2f; r.tick();
      assert(r.lower.phase()==LowerPhase::Fault && !r.lower.committed());
      assert(std::strcmp(r.lower.reason(),"lower_prepare_disturbed")==0); }
    { Rig r; r.prepare();
      for(int i=0;i<802 && r.lower.active();++i) { r.tick(false); assert(std::fabs(r.lower.left()-r.in.arm_left)<=.24001f); }
      assert(r.lower.phase()==LowerPhase::Fault && !r.lower.committed()); }
    { Rig r; r.in.wheel_left=20; r.start();
      for(int i=0;i<702;++i) r.tick();
      assert(r.lower.phase()==LowerPhase::Complete && !r.lower.committed());
      assert(std::strcmp(r.lower.reason(),"lower_stop_timeout")==0); }
    { Rig r; assert(r.lower.request(r.now,r.in,-1.77f,1.77f));
      for(int i=0;i<100;++i) r.tick(); assert(r.lower.left()>0 && r.lower.right()<0); }
    for(int fault=0;fault<4;++fault) {
      Rig r; r.catchFall();
      if(fault==0) r.in.healthy=false;
      if(fault==1) r.in.rate=-70;
      if(fault==2) r.in.wheel_left=6.1f;
      if(fault==3) r.in.arm_left=nan;
      r.tick(false); assert(r.lower.phase()==LowerPhase::Fault && r.lower.wheelCommand()==0); }
    { Rig r; r.catchFall(); const float held=r.lower.left(); r.in.rate=-20; r.tick(false);
      assert(r.lower.left()==held); }
    { Rig r; r.catchFall(); r.in.rate=-30; r.in.torque_left=r.in.torque_right=0;
      for(int i=0;i<5;++i) r.tick(false);
      assert(r.lower.phase()==LowerPhase::Fault);
      assert(std::strcmp(r.lower.reason(),"lower_support_lost")==0); }
    { Rig r; r.catchFall();
      for(int i=0;i<151;++i) r.tick(); assert(r.lower.phase()==LowerPhase::Fault);
      assert(std::strcmp(r.lower.reason(),"lower_descent_timeout")==0); }
    { Rig r; r.catchFall();
      for(int i=0;i<600 && r.lower.phase()!=LowerPhase::Retracting;++i) {
        r.in.tilt=std::max(0.f,r.in.tilt-.2f); r.in.rate=r.in.tilt>0?-10:0; r.tick();
      }
      assert(r.lower.phase()==LowerPhase::Retracting);
      for(int i=0;i<300 && r.lower.active();++i) r.tick();
      assert(r.lower.phase()==LowerPhase::Complete && r.lower.supported());
      assert(std::strcmp(r.lower.reason(),"lower_complete")==0); }
    { // Command/feedback sign disagreement is tolerated briefly, then aborts.
      Rig r; r.commit(); r.in.rate=0;
      for(int i=0;i<15;++i) r.tick();
      assert(r.lower.wheelCommand()<-.3f); r.in.wheel_left=.8f;
      for(int i=0;i<4;++i) { r.tick(false); assert(r.lower.active()); }
      r.tick(false); assert(std::strcmp(r.lower.reason(),"lower_wheel_direction")==0);
    }
    // The actual fast sender uses this stricter fall-owner freshness/command
    // gate. It does not affect ordinary balance or pilot driving.
    for(float command:{-6.f,-2.f,-.4f,0.f,.65f,2.f,6.f}) {
      auto fresh=lowerWheelCommand(command,100); assert(!fresh.fault && fresh.value==command);
      auto stale=lowerWheelCommand(command,101); assert(stale.fault && stale.value==0);
    }
    for(float bad:{nan,6.1f,-6.1f,std::numeric_limits<float>::infinity()}) {
      auto out=lowerWheelCommand(bad,0); assert(out.fault && out.value==0);
    }
    std::cout << "Lowering checks passed: prepare, deliberate forward departure, measured catch, independent arm hold, missed/false catch, faults, deadlines/rollover, landing and fast sender gate\n";
}
