#pragma once
#include "Arduino.h"
#include <deque>
#include <vector>
inline constexpr int SERIAL_8N1 = 0;
struct HardwareSerial {
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    size_t buffer_size = 0;
    size_t bulk_calls = 0, single_calls = 0, available_calls = 0;
    size_t max_chunk = 128;
    uint32_t read_cost_us = 40;
    bool storm = false;
    bool begun = false;
    size_t begin_calls = 0, end_calls = 0;
    int tx_space = 128;
    int availableForWrite() { return tx_space; }
    size_t setRxBufferSize(size_t n) { buffer_size = begun ? 0 : n; return buffer_size; }
    void begin(uint32_t, int, int, int) { begun = true; ++begin_calls; rx.clear(); }
    void end() { begun = false; ++end_calls; rx.clear(); }
    int available() { ++available_calls; return storm ? 2048 : rx.size(); }
    int read() {
        ++single_calls;
        fake_us += 1200;
        if (rx.empty()) return storm ? 0x55 : -1;
        const auto b = rx.front(); rx.pop_front(); return b;
    }
    size_t read(uint8_t* out, size_t n) {
        ++bulk_calls;
        fake_us += read_cost_us;
        n = std::min(n, max_chunk);
        if (!storm) n = std::min(n, rx.size());
        for (size_t i = 0; i < n; ++i) {
            out[i] = rx.empty() ? 0x55 : rx.front();
            if (!rx.empty()) rx.pop_front();
        }
        return n;
    }
    size_t write(const uint8_t* data, size_t n) { tx.insert(tx.end(), data, data + n); return n; }
    void feed(const std::vector<uint8_t>& bytes) { if (begun) rx.insert(rx.end(), bytes.begin(), bytes.end()); }
};
