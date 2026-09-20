#include "crsf.h"
#include "config.h"
#include <array>
#include <cassert>
#include <iostream>
#include <vector>

static uint8_t crc(const std::vector<uint8_t>& bytes) {
    uint8_t result = 0;
    for (uint8_t b : bytes) {
        result ^= b;
        for (int i = 0; i < 8; ++i)
            result = (result & 0x80) ? (result << 1) ^ 0xD5 : result << 1;
    }
    return result;
}
static std::vector<uint8_t> frame(uint8_t type, std::vector<uint8_t> payload) {
    payload.insert(payload.begin(), type);
    payload.push_back(crc(payload));
    payload.insert(payload.begin(), uint8_t(payload.size()));
    payload.insert(payload.begin(), CRSF_SYNC_BYTE);
    return payload;
}
static std::vector<uint8_t> channels(const std::array<uint16_t, 16>& values, bool extended = false) {
    std::vector<uint8_t> bytes(extended ? 24 : 22, 0);
    for (size_t ch = 0; ch < values.size(); ++ch)
        for (size_t bit = 0; bit < 11; ++bit)
            if (values[ch] & (1 << bit)) bytes[(ch * 11 + bit) / 8] |= 1 << ((ch * 11 + bit) % 8);
    return frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, bytes);
}
int main() {
    CrsfReceiver receiver;
    HardwareSerial uart;
    receiver.update(); // Before begin is harmless.
    receiver.begin(uart, 1, 2, 420000);
    assert(uart.buffer_size == 2048);
    assert(!receiver.isLinkUp());
    std::array<uint16_t, 16> values{172,992,1811,0,2047,174,1792,191,191,191,1792,191,1,2,3,4};
    auto valid = channels(values);
    fake_ms = 100;
    uart.feed({valid.begin(), valid.begin() + 13});
    receiver.update();
    assert(!receiver.isLinkUp());
    uart.feed({valid.begin() + 13, valid.end()});
    receiver.update();
    assert(receiver.isLinkUp());
    for (int i = 0; i < 16; ++i) assert(receiver.getChannel(i) == values[i]);
    assert(receiver.getChannelNormalized(10) > 0.9f); // CH11 trigger.
    assert(uart.single_calls == 0 && uart.available_calls == 0);

    fake_ms = 400;
    auto corrupt = channels(values);
    corrupt.back() ^= 1;
    uart.feed(corrupt);
    uart.feed(frame(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, {0,0})); // Valid CRC, insufficient data.
    uart.feed({0xC8, 0xFF, 0x55, 0xC8, 0x00, 0x55}); // Invalid lengths + noise.
    receiver.update();
    assert(receiver.timeSinceLastFrame() == 300);
    for (int i = 0; i < 16; ++i) assert(receiver.getChannel(i) == values[i]);
    fake_ms = 600;
    assert(!receiver.isLinkUp());
    uart.feed(frame(CRSF_FRAMETYPE_LINK_STATISTICS, {60,60,99}));
    receiver.update();
    assert(receiver.getLinkQuality() == 0);
    uart.feed(frame(CRSF_FRAMETYPE_LINK_STATISTICS, {60,60,99,0,0,0,0,0,0,0,42}));
    receiver.update();
    assert(receiver.getLinkQuality() == 99 && receiver.getRssi() == -60);
    assert(!receiver.isLinkUp()); // Stats do not refresh control freshness.

    values[10] = 191;
    uart.feed(channels(values, true));
    receiver.update();
    assert(receiver.isLinkUp() && receiver.getChannel(10) == 191);
    uart.max_chunk = 1; // UART may return less than requested.
    values[9] = 1792;
    uart.feed(channels(values));
    receiver.update();
    assert(receiver.getChannel(9) == 1792);

    // A continuous stream cannot defeat either the byte or time budget.
    uart.max_chunk = 128;
    uart.storm = true;
    auto before = receiver.receivedBytes();
    auto yields = receiver.budgetYields();
    auto start = fake_us;
    receiver.update();
    assert(receiver.receivedBytes() - before == 1024);
    assert(fake_us - start < 2000 && receiver.budgetYields() == yields + 1);
    uart.read_cost_us = 600;
    before = receiver.receivedBytes();
    start = fake_us = UINT32_MAX - 1000; // Wrap-safe elapsed time.
    receiver.update();
    assert(uint32_t(fake_us - start) == 2400);
    assert(receiver.receivedBytes() - before == 512);

    // Yield between blocks in the middle of a frame; preserve the fragment.
    uart.storm = false;
    uart.max_chunk = 7;
    uart.read_cost_us = 1100;
    values[10] = 1792;
    uart.feed(channels(values));
    receiver.update(); // 14 bytes, time budget exhausted.
    assert(receiver.getChannel(10) == 191);
    receiver.update();
    assert(receiver.getChannel(10) == 1792 && uart.rx.empty());
    const auto calls = uart.bulk_calls;
    receiver.update();
    assert(uart.bulk_calls == calls + 1); // Empty read exits immediately.
    assert(uart.single_calls == 0 && uart.available_calls == 0);
    assert(receiver.maxUpdateUs() == 2400);

    // OTA shuts down RX interrupts/TX and discards both buffered frames and a
    // partial parser frame. A live transmitter cannot create work while paused.
    uart.max_chunk = 128;
    uart.read_cost_us = 40;
    uart.feed({valid.begin(), valid.begin() + 13});
    receiver.update();
    uart.feed({valid.begin() + 13, valid.end()});
    receiver.suspend();
    receiver.suspend(); // Repeated maintenance ticks do not tear down twice.
    assert(receiver.suspended() && !uart.begun && uart.end_calls == 1);
    assert(!receiver.isLinkUp() && receiver.timeSinceLastFrame() == UINT32_MAX);
    assert(receiver.getLinkQuality() == 0 && receiver.getRssi() == 0);
    for (int i = 0; i < 16; ++i) assert(receiver.getChannel(i) == CRSF_CHANNEL_MID);
    const auto paused_calls = uart.bulk_calls;
    const auto paused_bytes = receiver.receivedBytes();
    const auto paused_tx = uart.tx.size();
    uart.storm = true;
    uart.feed(valid);
    receiver.update();
    receiver.sendFlightMode("OTA");
    receiver.sendBatteryTelemetry(24, 0);
    receiver.sendAttitudeTelemetry(0, 0, 0);
    const uint8_t payload[6] = {};
    assert(!receiver.sendRobotTelemetry(payload, sizeof(payload)));
    assert(uart.bulk_calls == paused_calls && receiver.receivedBytes() == paused_bytes);
    assert(uart.tx.size() == paused_tx);
    uart.storm = false;

    // Failed OTA resumes only an empty UART. Cached LOW switches cannot clear
    // the control task's rearm latch; new RC channel frames are required.
    receiver.resume(); receiver.resume();
    assert(!receiver.suspended() && uart.begun && uart.begin_calls == 2);
    receiver.update();
    assert(!receiver.isLinkUp() && uart.rx.empty());
    uart.feed(frame(CRSF_FRAMETYPE_LINK_STATISTICS, {60,60,99,0,0,0,0,0,0,0}));
    receiver.update();
    assert(!receiver.isLinkUp());
    fake_ms = 900;
    uart.feed(channels(values));
    receiver.update();
    assert(receiver.isLinkUp() && receiver.getChannel(10) == 1792);
    receiver.sendFlightMode("DISARM");
    assert(uart.tx.size() > paused_tx);
    std::cout << "CRSF production parser/timing tests passed\n";
}
