#include "balance_math.h"
#include "config.h"
#include "telemetry_transport.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <random>

int main() {
    assert(balance_math::fresh(100000, 50000, BALANCE_IMU_STALE_US));
    assert(!balance_math::fresh(100001, 50000, BALANCE_IMU_STALE_US));
    assert(balance_math::fresh(10, UINT32_MAX - 9, 20));
    assert(!balance_math::fresh(11, UINT32_MAX - 9, 20));
    assert(balance_math::finiteImu(0,0,1,0,0,0));
    assert(!balance_math::finiteImu(0,0,0,0,0,0));
    assert(!balance_math::finiteImu(0,0,1,std::numeric_limits<float>::quiet_NaN(),0,0));
    assert(!balance_math::finiteImu(0,0,1,0,std::numeric_limits<float>::infinity(),0));
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> command(-60,60), yaw(-4,4);
    for (int i=0; i<100000; ++i) {
        const float limit = i % 2 ? BALANCE_MAX_DRIVE_SPEED : BALANCE_DEADMAN_SOFT_CMD_MAX;
        const float common = balance_math::clamp(command(rng), -limit, limit);
        const auto wheels = balance_math::mix(common, yaw(rng), limit);
        assert(std::fabs(wheels.left) <= limit + 1e-5f);
        assert(std::fabs(wheels.right) <= limit + 1e-5f);
        assert(std::fabs((wheels.left + wheels.right)*0.5f - common) < 1e-5f);
    }
    const auto saturated = balance_math::mix(30, 1.5, 30);
    assert(saturated.left == 30 && saturated.right == 30 && saturated.yaw == 0);
    {
    const std::string payload = "hello";
    TelemetryTransport transport;
    assert(Serial.tx_timeout_ms == 50);
    Serial.chunk = 2;  // forced partial writes through the production writer
    assert(transport.write(reinterpret_cast<const uint8_t*>(payload.data()), payload.size()) == payload.size());
    assert(Serial.bytes == payload);
    assert(transport.checksum() == 0x4f9f2cabu);
    Serial.blocked = true;
    assert(transport.write(uint8_t('x')) == 0);
    assert(transport.failed());
    assert(fake_ms <= 2001);
    Serial.blocked = false;
    assert(transport.write(uint8_t('x')) == 0); // failed transfers never get a success trailer
    TelemetryTransport disconnected;
    Serial.connected = false;
    const auto before = fake_ms;
    assert(disconnected.write(uint8_t('x')) == 0);
    assert(fake_ms == before && disconnected.failed());
    }
    assert(Serial.tx_timeout_ms == 1);
    Serial.connected = true;
    Serial.bytes.clear();
    Serial.writable = 16;
    Serial.chunk = 7;
    Serial.resume_at_ms = fake_ms + 20; // connected host temporarily not draining
    {
        TelemetryTransport large;
        const std::string header(1024, 'h'); // larger than the real 256-byte TX ring
        assert(large.write(reinterpret_cast<const uint8_t*>(header.data()), header.size()) == header.size());
        assert(Serial.bytes == header && !large.failed());
        assert(Serial.overflow_writes == 0 && Serial.connected);
    }
    assert(Serial.tx_timeout_ms == 1);
    std::cout << "Native checks passed: 100000 wheel mixes, sensor validity/rollover, USB partial writes/stall/disconnect\n";
}
