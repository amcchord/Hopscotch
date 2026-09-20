#include "network_safety.h"
#include <cassert>
#include <thread>
#include <iostream>
int main() {
    bool MaintenanceConditions::* const blockers[] = {
        &MaintenanceConditions::balance, &MaintenanceConditions::saving,
        &MaintenanceConditions::drive, &MaintenanceConditions::arms, &MaintenanceConditions::arming,
        &MaintenanceConditions::calibration, &MaintenanceConditions::test,
        &MaintenanceConditions::simulation, &MaintenanceConditions::download, &MaintenanceConditions::motor_enabled
    };
    for (unsigned mask=0; mask < (1u<<10); ++mask) {
        MaintenanceConditions c{};
        for (unsigned i=0;i<10;++i) c.*blockers[i] = (mask & (1u<<i)) != 0;
        assert(c.allowed() == (mask == 0));
    }
    MaintenanceGate gate;
    assert(!gate.busy() && !gate.granted());
    assert(gate.request());
    assert(gate.busy() && !gate.granted());
    assert(!gate.request());
    gate.service(false);
    assert(gate.state() == MaintenanceGate::Denied);
    gate.service(true); // a denial cannot later turn into permission
    assert(!gate.granted());
    gate.release();
    assert(gate.request()); gate.service(true);
    assert(gate.granted());
    gate.service(false); // permission stays latched; arming must stay inhibited
    assert(gate.granted() && !gate.request()); gate.release();
    // Stress request cancellation versus grant. No permission can survive the
    // final release, and all new operations need another explicit request.
    for (int i=0;i<1000;++i) {
        assert(gate.request());
        std::thread owner([&]{gate.service(true);});
        gate.release(); owner.join(); assert(!gate.busy());
    }
    LoopTiming t;
    for (auto dt : {5000u, 5100u, 7500u, 7501u, 10001u}) t.record(dt);
    assert(t.ticks == 5 && t.max_us == 10001 && t.over_7500_us == 2 && t.over_10000_us == 1);
    std::cout << "network safety and timing tests passed\n";
}
