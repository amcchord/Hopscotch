#pragma once
#include <atomic>
#include <cstdint>

struct MaintenanceConditions {
    bool balance, saving, drive, arms, arming, calibration, test, simulation, download, motor_enabled;
    bool allowed() const {
        return !(balance || saving || drive || arms || arming || calibration || test || simulation || download || motor_enabled);
    }
};

// The control task alone grants access. Networking cannot infer safety from
// a stale telemetry sample. Granted stays latched until the operation ends.
class MaintenanceGate {
public:
    enum State : uint8_t { Idle, Requested, Granted, Denied, OtaRequested, OtaGranted };
    bool request(bool ota = false) {
        State s = Idle;
        return _state.compare_exchange_strong(s, ota ? OtaRequested : Requested);
    }
    // Preparation runs on the control owner before permission becomes visible
    // to networking. CAS retains the original request kind across cancellation.
    template <typename Prepare>
    void service(bool safe, Prepare prepare) {
        State s = state();
        if (s != Requested && s != OtaRequested) return;
        const State grantedState = s == OtaRequested ? OtaGranted : Granted;
        if (safe) prepare(s == OtaRequested);
        _state.compare_exchange_strong(s, safe ? grantedState : Denied);
    }
    void service(bool safe) { service(safe, [](bool) {}); }
    State state() const { return _state.load(); }
    bool pending() const { const auto s = state(); return s == Requested || s == OtaRequested; }
    bool busy() const { return state() != Idle; }
    bool granted() const { const auto s = state(); return s == Granted || s == OtaGranted; }
    bool ota() const { const auto s = state(); return s == OtaRequested || s == OtaGranted; }
    void release() { _state.store(Idle); }
private:
    std::atomic<State> _state{Idle};
};

// Owned by one task. Reset only on that task; exported via a value snapshot.
struct LoopTiming {
    uint32_t ticks = 0, max_us = 0, over_7500_us = 0, over_10000_us = 0;
    void record(uint32_t us) {
        ++ticks;
        if (us > max_us) max_us = us;
        if (us > 7500) ++over_7500_us;
        if (us > 10000) ++over_10000_us;
    }
};
