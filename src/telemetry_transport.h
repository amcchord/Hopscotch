#pragma once
#include <Arduino.h>

// Used only for an explicitly requested, disarmed log download. Live control
// keeps Serial's 1ms timeout (zero underflows in the pinned HWCDC library).
// Partial USB writes are retried with a bounded
// timeout here, and the host verifies the exact bytes of the CSV payload.
class TelemetryTransport : public Print {
public:
    using Print::write;
    size_t write(uint8_t byte) override { return write(&byte, 1); }
    size_t write(const uint8_t* data, size_t size) override {
        if (_failed) return 0;
        size_t sent = 0;
        uint32_t progress_ms = millis();
        while (sent < size) {
            size_t n = Serial.write(data + sent, size - sent);
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
