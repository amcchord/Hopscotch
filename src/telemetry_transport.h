#pragma once
#include <Arduino.h>

// Used only for an explicitly requested, disarmed log download. HWCDC 2.0.16
// marks the connection lost when a large write cannot drain within its timeout.
// Pace writes to available space and allow host scheduling during this idle
// transfer. Restore the 1ms live-control timeout on every exit, including errors.
class TelemetryTransport : public Print {
public:
    TelemetryTransport() { Serial.setTxTimeoutMs(50); }
    ~TelemetryTransport() { Serial.setTxTimeoutMs(1); }
    TelemetryTransport(const TelemetryTransport&) = delete;
    TelemetryTransport& operator=(const TelemetryTransport&) = delete;
    using Print::write;
    size_t write(uint8_t byte) override { return write(&byte, 1); }
    size_t write(const uint8_t* data, size_t size) override {
        if (_failed) return 0;
        size_t sent = 0;
        uint32_t progress_ms = millis();
        while (sent < size) {
            // HWCDC's disconnected path can report bytes accepted while
            // discarding queued data. Never deliberately enter that path.
            if (!Serial) { _failed = true; break; }
            const int available = Serial.availableForWrite();
            size_t chunk = size - sent;
            if (chunk > 64) chunk = 64;
            if (available <= 0) chunk = 0;
            else if (chunk > static_cast<size_t>(available)) chunk = available;
            size_t n = chunk ? Serial.write(data + sent, chunk) : 0;
            for (size_t i = 0; i < n; ++i) {
                _checksum ^= data[sent + i];
                _checksum *= 16777619u;
            }
            sent += n;
            if (n) progress_ms = millis();
            else if (!Serial || millis() - progress_ms > 2000) {
                _failed = true;
                break;
            } else delay(1);
        }
        return sent;
    }
    void resetChecksum() { _checksum = 2166136261u; }
    uint32_t checksum() const { return _checksum; }
    bool failed() const { return _failed; }
private:
    uint32_t _checksum = 2166136261u;
    bool _failed = false;
};
