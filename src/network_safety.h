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
    enum State : uint8_t { Idle, Requested, Granted, Denied };
    bool request() { State s = Idle; return _state.compare_exchange_strong(s, Requested); }
    void service(bool safe) {
        State s = Requested;
        _state.compare_exchange_strong(s, safe ? Granted : Denied);
    }
    State state() const { return _state.load(); }
    bool busy() const { return state() != Idle; }
    bool granted() const { return state() == Granted; }
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
