// Native simulation interface: execute the exact firmware maneuver policy.
#include "balance_lower.h"
using namespace balance_math;
static LowerInput input(const float* v, bool healthy) {
    return LowerInput{v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],healthy};
}
extern "C" {
void* lower_new() { return new BalanceLower; }
void lower_delete(void* p) { delete static_cast<BalanceLower*>(p); }
bool lower_request(void* p, uint32_t now, const float* values, bool healthy, float left, float right) {
    return static_cast<BalanceLower*>(p)->request(now,input(values,healthy),left,right);
}
void lower_step(void* p, uint32_t now, float dt, const float* values, bool healthy, float* result) {
    auto& lower = *static_cast<BalanceLower*>(p);
    lower.step(now,dt,input(values,healthy));
    result[0] = lower.left(); result[1] = lower.right();
    result[2] = static_cast<float>(lower.phase()); result[3] = lower.supported();
    result[4] = lower.overridesArms();
    result[5] = lower.wheelCommand();
    result[6] = lower.committed();
}
const char* lower_reason(void* p) { return static_cast<BalanceLower*>(p)->reason(); }
float lower_arm_speed(void* p) { return static_cast<BalanceLower*>(p)->armSpeed(); }
}

extern "C" float lower_balance_setpoint(void* p, float ordinary) {
#ifdef BALANCE_LOWER_FORWARD_PREPARE_V5
    return static_cast<balance_math::BalanceLower*>(p)->preparationSetpoint(ordinary);
#else
    (void)p;
    return ordinary;
#endif
}
