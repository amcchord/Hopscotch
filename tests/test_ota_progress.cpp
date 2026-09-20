#include "ota_progress.h"
#include <cassert>
#include <iostream>
#include <type_traits>

int main() {
    static_assert(std::is_trivially_copyable<OtaProgress>::value, "FreeRTOS value snapshot");
    OtaProgress p;
    assert(!p.visible(0) && !p.active() && p.percent() == 0);
    assert(p.elapsedMs(123) == 0 && p.bytesPerSecond(123) == 0);
    p = {OtaPhase::Preparing, 0, 1200000, 100, 100};
    assert(p.active() && p.visible(100) && p.percent() == 0 && p.bytesPerSecond(100) == 0);
    p = {OtaPhase::Receiving, 600000, 1200000, 100, 10100};
    assert(p.percent() == 50 && p.bytesPerSecond(10100) == 60000);
    assert(p.elapsedMs(20100) == 20000 && p.bytesPerSecond(20100) == 30000);
    p.phase = OtaPhase::Verifying; p.received = p.total;
    assert(p.percent() == 99); // Never claim completion before ESP/hash validation.
    p.phase = OtaPhase::Rebooting;
    assert(p.percent() == 100 && p.active() && p.visible(99999));
    assert(p.elapsedMs(99999) == 10000); // Freeze metrics on completion.
    p.phase = OtaPhase::Failed;
    assert(!p.active() && p.percent() == 99 && p.visible(15099) && !p.visible(15100));
    assert(p.elapsedMs(99999) == 10000);
    p = {OtaPhase::Receiving, 1024, 2048, UINT32_MAX - 999, 0};
    assert(p.elapsedMs(0) == 1000 && p.bytesPerSecond(0) == 1024 && p.percent() == 50);
    p.phase = OtaPhase::Failed; p.updated_ms = UINT32_MAX - 999;
    assert(p.visible(0) && !p.visible(4000));
    p = {OtaPhase::Receiving, UINT32_MAX, UINT32_MAX, 0, 1000};
    assert(p.percent() == 99); // Widen before multiplying byte counts.
    std::cout << "OTA progress lifecycle/metrics tests passed\n";
}
