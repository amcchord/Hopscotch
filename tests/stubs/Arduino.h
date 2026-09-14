#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
class Print {
public:
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t* bytes, size_t n) {
        size_t done = 0;
        for (; done < n; ++done) if (!write(bytes[done])) break;
        return done;
    }
};
inline uint32_t fake_ms = 0;
inline uint32_t millis() { return fake_ms; }
inline void delay(uint32_t n) { fake_ms += n; }
struct FakeSerial {
    bool connected = true;
    bool blocked = false;
    size_t chunk = 7;
    std::string bytes;
    explicit operator bool() const { return connected; }
    size_t write(const uint8_t* p, size_t n) {
        if (blocked || !connected) return 0;
        n = std::min(chunk, n);
        bytes.append(reinterpret_cast<const char*>(p), n);
        return n;
    }
};
inline FakeSerial Serial;
