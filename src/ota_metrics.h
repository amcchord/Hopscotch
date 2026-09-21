#pragma once
#include <cstdint>

// Network-task diagnostics, serialized under the existing HTTP mutex. Keep
// cumulative flash-write time separate from time waiting between callbacks.
struct OtaMetrics {
    uint32_t write_calls = 0, max_write_us = 0, verify_us = 0;
    uint32_t last_callback_end_ms = 0, max_receive_gap_ms = 0;
    uint64_t write_us = 0;
    void start(uint32_t now) { *this = {}; last_callback_end_ms = now; }
    void received(uint32_t now) {
        const uint32_t gap = now - last_callback_end_ms;
        if (gap > max_receive_gap_ms) max_receive_gap_ms = gap;
    }
    void wrote(uint32_t elapsed_us) {
        ++write_calls;
        write_us += elapsed_us;
        if (elapsed_us > max_write_us) max_write_us = elapsed_us;
    }
};
