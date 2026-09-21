#include "balance_lower.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
using namespace balance_math;

struct Rig {
    BalanceLower lower;
    LowerInput in{88,0,0,0,0,0,0,0,0,0,0,true};
    bool fast_selected = false;
    uint32_t now = UINT32_MAX-1000; // Run confirmation/deadlines through rollover.
    void tick(bool follow=true) {
        now+=20;
        lower.step(now,.02f,in);
        if (follow && lower.overridesArms()) { in.arm_left=lower.left(); in.arm_right=lower.right(); }
    }
    void start() { assert(lower.request(now,in,1.768f,-1.767f,fast_selected)); }
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
        for(int i=0;i<3;++i) tick();
        assert(!lower.supported());
        tick();
        assert(lower.phase()==LowerPhase::Descending && lower.supported());
        assert(lower.armSpeed()==.3f);
        assert(lower.left()>=caught_left && lower.left()<=caught_left+.061f);
        assert(lower.right()<=caught_right && lower.right()>=caught_right-.061f);
    }
};

int main() {
    { // CH6 mode is latched only on accepted request, independently per maneuver.
      Rig r; r.fast_selected=true; r.start(); assert(r.lower.fast());
      assert(!r.lower.request(r.now,r.in,1.768f,-1.767f,false));
      assert(r.lower.fast()); r.lower.reset(); assert(!r.lower.fast());
      r.in.healthy=false;
      assert(!r.lower.request(r.now,r.in,1.768f,-1.767f,true));
      assert(!r.lower.fast()); r.in.healthy=true; r.fast_selected=false;
      r.start(); assert(!r.lower.fast());
    }
    { // Fast selection leaves preparation/catch untouched; acceleration starts
      // only after confirmed support and normal speed returns near the floor.
      Rig normal, fast; fast.fast_selected=true;
      normal.catchFall(); fast.catchFall();
      assert(normal.now==fast.now && normal.lower.left()==fast.lower.left());
      assert(normal.lower.right()==fast.lower.right());
      assert(normal.lower.wheelCommand()==fast.lower.wheelCommand());
      assert(fast.lower.armSpeed()==normal.lower.armSpeed());
      for(int i=0;i<30;++i) { normal.in.tilt-=.1f;fast.in.tilt-=.1f;normal.tick();fast.tick(); }
      assert(std::fabs(fast.lower.armSpeed()-.75f)<.0001f);
      assert(fast.lower.left()>normal.lower.left()+.10f);
      normal.in.rate=fast.in.rate=-15;
      float a=normal.lower.left(),b=fast.lower.left(); normal.tick();fast.tick();
      assert(normal.lower.left()==a && std::fabs(fast.lower.left()-b-.012f)<.0001f);
      for(float rate:{-20.01f,4.01f}) {
        fast.in.rate=rate;b=fast.lower.left();fast.tick();assert(fast.lower.left()==b);
      }
      fast.in.tilt=25;fast.in.rate=-8;b=fast.lower.left();fast.tick();
      assert(std::fabs(fast.lower.armSpeed()-.525f)<.0001f);
      assert(std::fabs(fast.lower.left()-b-.0084f)<.0001f);
      fast.in.tilt=15;b=fast.lower.left();fast.tick();
      assert(fast.lower.armSpeed()==.30f);
      assert(std::fabs(fast.lower.left()-b-.0048f)<.0001f);
      fast.in.rate=-12.01f;b=fast.lower.left();fast.tick();assert(fast.lower.left()==b);
      fast.in.tilt=0;fast.in.rate=0;fast.in.wheel_left=fast.in.wheel_right=0;
      fast.tick(); assert(fast.lower.phase()==LowerPhase::GroundHold);
      for(int i=0;i<29;++i)fast.tick();
      assert(fast.lower.phase()==LowerPhase::GroundHold);fast.tick();
      assert(fast.lower.phase()==LowerPhase::Retracting && fast.lower.armSpeed()==.30f);
    }
    { // Fast return does not waive feedback, support-loss or global rate guards.
      for(int fault=0;fault<3;++fault) {
        Rig r;r.fast_selected=true;r.catchFall();
        if(fault==0)r.in.healthy=false;
        if(fault==1){r.in.rate=-26;r.in.torque_left=0;}
        if(fault==2)r.in.rate=65.01f;
        for(int i=0;i<5 && r.lower.active();++i)r.tick();
        assert(r.lower.phase()==LowerPhase::Fault);
      }
    }
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
    { // Physical v5 stopped before contact: the final preparation sample was
      // falling forward with one wheel already over 6 rad/s. Hand off at the
      // earlier measured moving-arm frame, without widening that wheel limit.
      Rig r; r.in.tilt=87.885f; r.prepare();
      r.in.tilt=85.721f; r.in.rate=-19.553f; r.in.error=2.163f;
      r.in.arm_left=-1.671f; r.in.arm_right=1.639f;
      r.in.velocity_left=-3.096f; r.in.velocity_right=3.163f;
      r.in.wheel_left=1.695f; r.in.wheel_right=4.490f;
      r.tick(false);
      assert(r.lower.committed() && !r.lower.supported());
      assert(std::fabs(r.lower.wheelCommand()-3.0925f)<.001f);
      // A subsequent qualified two-arm impact must enter continuous return,
      // not finish or hold merely because the arms are supporting the body.
      r.in.wheel_left=r.in.wheel_right=r.lower.wheelCommand();
      r.in.arm_left=-1.85f; r.in.arm_right=1.85f;
      r.in.tilt=82; r.in.rate=-30;
      for(int i=0;i<6;++i) r.tick(false);
      r.in.tilt=80; r.in.rate=-6;
      r.in.torque_left=.6f; r.in.torque_right=-.6f;
      r.in.velocity_left=.3f; r.in.velocity_right=-.3f;
      for(int i=0;i<5;++i) r.tick();
      assert(r.lower.supported() && r.lower.phase()==LowerPhase::Descending);
      const float first=r.lower.left();
      for(int i=0;i<40;++i) { r.in.tilt-=.1f; r.tick(); }
      assert(r.lower.active() && r.lower.left()>first+.1f);
      assert(r.lower.right()<-first-.1f);
    }
    { // The early route needs two moving, sufficiently deployed arms plus
      // measured forward departure. A jam, one late arm, upright stillness or
      // a backward disturbance cannot unlock it.
      for(int missing=0;missing<7;++missing) {
        Rig r; r.prepare(); r.in.tilt=86; r.in.rate=-10;
        r.in.arm_left=-1.65f; r.in.arm_right=1.65f;
        r.in.velocity_left=-1; r.in.velocity_right=1;
        if(missing==0) r.in.arm_right=1.5f;
        if(missing==1) r.in.velocity_left=0;
        if(missing==2) r.in.velocity_right=0;
        if(missing==3) r.in.tilt=88;
        if(missing==4) r.in.rate=3;
        if(missing==5) r.in.torque_left=.8f;
        if(missing==6) r.in.torque_right=-.8f;
        r.tick(false); assert(!r.lower.committed());
      }
    }
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
    { // Recorded v6 contact: both arms loaded at 30.030 s, then alternately
      // unload as the targets reverse. Instantaneous two-arm/4deg/s dwell
      // never passed, leaving wheels coasting until a second rebound fault.
      Rig r; r.in.tilt=87.306f; r.prepare();
      r.in.tilt=86.19f; r.in.rate=-8.368f;
      r.in.arm_left=-1.640f; r.in.arm_right=1.614f;
      r.in.velocity_left=-3.323f; r.in.velocity_right=3.216f;
      r.in.wheel_left=3.123f; r.in.wheel_right=3.153f;
      r.tick(false); assert(r.lower.committed());
      r.in.arm_left=-1.85f; r.in.arm_right=1.85f;
      r.in.tilt=82.53f; r.in.rate=-38.775f;
      r.in.velocity_left=-.211f; r.in.velocity_right=.118f;
      for(int i=0;i<8;++i) r.tick(false);
      const float coast=r.lower.wheelCommand();
      struct Contact { float tilt,rate,lv,rv,lt,rt; };
      const Contact recorded[]={
        {81.896f,-19.615f,.081f,.081f,-.489f,.453f},
        {81.707f,1.212f,-.052f,.042f,-.409f,.377f},
        {81.676f,6.867f,.023f,-.235f,-.175f,.010f},
        {81.797f,8.649f,.292f,-.348f,.110f,-.207f},
        {82.018f,8.964f,.250f,-.408f,.051f,-.228f}
      };
      float previous=coast;
      for(const auto& x:recorded) {
        r.in.tilt=x.tilt;r.in.rate=x.rate;
        r.in.velocity_left=x.lv;r.in.velocity_right=x.rv;
        r.in.torque_left=x.lt;r.in.torque_right=x.rt;
        r.tick(false);
        assert(r.lower.active());
        assert(r.lower.wheelCommand()<previous);
        assert(previous-r.lower.wheelCommand()<=.06001f);
        previous=r.lower.wheelCommand();
      }
      assert(r.lower.supported() && r.lower.phase()==LowerPhase::Descending);
      r.in.rate=-2;const float first=r.lower.left();
      for(int i=0;i<40;++i) { r.in.tilt-=.1f;r.tick(); }
      assert(r.lower.left()>first+.1f && r.lower.phase()!=LowerPhase::Complete);
    }
    { // V8 physical catch: reversal inertia delays qualification, then a
      // small loaded rebound reaches 17deg/s and resets the old 12deg/s dwell.
      Rig r; r.fall(); r.in.tilt=83.572f; r.in.rate=-39.388f;
      r.in.arm_left=-1.827f; r.in.arm_right=1.817f;
      r.tick(false);
      struct Contact { float tilt,rate,left,right,lv,rv,lt,rt; };
      const Contact samples[]={
        {82.387f,-8.689f,-1.848f,1.839f,-.289f,.216f,-.453f,.421f},
        {82.231f,7.501f,-1.852f,1.843f,-.170f,.158f,-.527f,.406f},
        {82.390f,8.517f,-1.851f,1.837f,.006f,-.213f,-.179f,-.102f},
        {82.297f,4.400f,-1.846f,1.830f,.247f,-.253f,.095f,-.371f},
        {82.786f,16.999f,-1.838f,1.822f,.295f,-.499f,.290f,-.268f},
        {83.129f,13.180f,-1.829f,1.812f,.422f,-.425f,.146f,-.249f}
      };
      for(unsigned i=0;i<sizeof(samples)/sizeof(samples[0]);++i) {
        const auto& x=samples[i];r.in.tilt=x.tilt;r.in.rate=x.rate;
        r.in.arm_left=x.left;r.in.arm_right=x.right;
        r.in.velocity_left=x.lv;r.in.velocity_right=x.rv;
        r.in.torque_left=x.lt;r.in.torque_right=x.rt;r.tick(false);
        assert(r.lower.active());
        if(i<5) assert(!r.lower.supported());
      }
      assert(r.lower.supported() && r.lower.phase()==LowerPhase::Descending);
      assert(r.lower.armSpeed()==.30f); // Exit the faster impact-softening path.
    }
    { // The wider confirmation window is only for bounded, recently loaded
      // rocking. It cannot turn missing support or a large/high/late rebound
      // into a catch, nor waive the measured arm-return velocity requirements.
      for(int missing=0;missing<7;++missing) {
        Rig r;r.fall();r.in.tilt=82;r.in.rate=-6;
        r.in.velocity_left=.2f;r.in.velocity_right=-.2f;
        r.in.torque_left=.6f;r.in.torque_right=-.6f;r.tick();
        r.in.rate=17;
        if(missing==1) r.in.torque_left=0;
        if(missing==2) r.in.rate=20.01f;
        if(missing==3) r.in.tilt=83.51f;
        if(missing==4) r.now+=301;
        if(missing==5) r.in.velocity_left=.71f;
        if(missing==6) r.in.velocity_right=.11f;
        for(int i=0;i<4 && r.lower.active();++i) r.tick();
        assert(r.lower.supported()==(missing==0));
      }
    }
    { // A single two-arm impact followed by complete unloading cannot
      // qualify an 80ms support dwell using the 60ms observation grace.
      Rig r;r.fall();r.in.rate=0;r.in.velocity_left=r.in.velocity_right=0;
      r.in.torque_left=r.in.torque_right=.8f;r.tick();
      r.in.torque_left=r.in.torque_right=0;
      for(int i=0;i<10;++i) { r.tick();assert(!r.lower.supported()); }
    }
    { // First contact may be one arm much earlier. The real-time dwell
      // starts from BOTH contacts, not the old one-arm impact timestamp.
      Rig r;r.fall();r.in.rate=0;r.in.velocity_left=r.in.velocity_right=0;
      r.in.torque_left=.8f;r.in.torque_right=0;
      for(int i=0;i<10;++i) r.tick();
      r.in.torque_right=.8f;r.tick();
      r.in.torque_left=r.in.torque_right=0;
      for(int i=0;i<10;++i) { r.tick();assert(!r.lower.supported()); }
    }
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
    { // Successful v7 descent spent 11.37s tracking its slow targets with
      // <=0.021rad error. V8 advances supported targets 50% faster, while
      // preserving the motor cap and rate pause through a recorded disturbance.
      Rig r; r.catchFall(); r.in.rate=-6.7f;
      const float start=r.lower.left();
      for(int i=0;i<50;++i) { r.in.tilt-=.14f; r.tick(); }
      assert(r.lower.left()>start+.235f && r.lower.left()<start+.245f);
      assert(r.lower.armSpeed()==.30f);
      const float held=r.lower.left();
      for(float rate:{-22.752f,-12.01f,7.684f}) {
        r.in.rate=rate; r.tick(false);
        assert(r.lower.active() && r.lower.left()==held);
      }
      r.in.rate=-6.7f; r.tick(); assert(r.lower.left()>held);
    }
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
