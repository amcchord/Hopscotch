#include "balance_tip_up.h"
#include "balance_math.h"

extern "C" {
void* recovery_create() { return new balance_math::RecoilUnwind; }
void recovery_destroy(void* p) { delete static_cast<balance_math::RecoilUnwind*>(p); }
float recovery_step(void* p, bool candidate, bool boosting, bool ramp_complete,
                    float velocity, float integral, float offset, float normal_ki, float dt) {
    const balance_math::RecoilConfig c={BALANCE_RECOIL_ENTER_SPEED, BALANCE_RECOIL_EXIT_SPEED,
        BALANCE_RECOIL_CONFIRM_MS, BALANCE_RECOIL_BLEND_MS, BALANCE_RECOIL_MULTIPLIER};
    const float multiplier=static_cast<balance_math::RecoilUnwind*>(p)->update(
        ramp_complete,velocity,integral,dt,c);
    // Baseline is fast v2's unchanged 1.0 boost; candidate uses fast v3.
    return balance_math::recoveryIntegral(integral,velocity,
        balance_math::tipRecoveryKi(candidate,boosting,normal_ki),dt,offset,
        BALANCE_START_RECOVERY_LIMIT_DEG,BALANCE_START_RECOVERY_RATE_DPS,true,multiplier).value;
}
}
