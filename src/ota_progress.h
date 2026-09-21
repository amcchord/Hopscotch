#pragma once
#include <cstdint>

enum class OtaPhase : uint8_t { Idle, Preparing, Receiving, Verifying, Rebooting, Failed };

// A bounded value snapshot, copied from networking to the display without
// taking the HTTP/flash mutex. Bytes count successful Update.write calls.
struct OtaProgress {
    OtaPhase phase = OtaPhase::Idle;
    uint32_t received = 0, total = 0;
    uint32_t started_ms = 0, updated_ms = 0;

    bool active() const { return phase != OtaPhase::Idle && phase != OtaPhase::Failed; }
    bool visible(uint32_t now) const {
        return active() || (phase == OtaPhase::Failed && uint32_t(now - updated_ms) < 5000);
    }
    unsigned percent() const {
        if (phase == OtaPhase::Rebooting) return 100;
        if (!total) return 0;
        const uint64_t value = uint64_t(received) * 100 / total;
        return value < 100 ? static_cast<unsigned>(value) : 99;
    }
    uint32_t elapsedMs(uint32_t now) const {
        if (phase == OtaPhase::Idle) return 0;
        const auto end = active() && phase != OtaPhase::Rebooting ? now : updated_ms;
        return end - started_ms;
    }
    uint32_t bytesPerSecond(uint32_t now) const {
        const auto elapsed = elapsedMs(now);
        return elapsed ? uint64_t(received) * 1000 / elapsed : 0;
    }
};
