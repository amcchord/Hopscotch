#pragma once

#include <cstdint>
#include <HardwareSerial.h>

static constexpr int CRSF_MAX_CHANNELS = 16;
static constexpr int CRSF_MAX_PACKET_SIZE = 64;

// CRSF frame types
static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS    = 0x14;
static constexpr uint8_t CRSF_FRAMETYPE_FLIGHT_MODE        = 0x21;

// Sync/address bytes
static constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

class CrsfReceiver {
public:
    // Initialize UART on given pins
    void begin(HardwareSerial& serial, int rx_pin, int tx_pin, uint32_t baudrate);

    // Call every loop iteration to process incoming bytes
    void update();

    // Channel data (raw 11-bit: 172..1811)
    uint16_t getChannel(int ch) const;

    // Normalized channel value: -1.0 to +1.0
    float getChannelNormalized(int ch) const;

    // Send flight mode telemetry string back to transmitter
    void sendFlightMode(const char* mode);

    // Is the receiver link active?
    bool isLinkUp() const;

    // Time since last valid frame (ms)
    uint32_t timeSinceLastFrame() const;

    // Link quality (0-100, from link statistics)
    uint8_t getLinkQuality() const { return _link_quality; }
    int8_t  getRssi() const { return _rssi; }

private:
    HardwareSerial* _serial = nullptr;
    uint16_t _channels[CRSF_MAX_CHANNELS] = {0};
    uint32_t _last_frame_time = 0;
    uint8_t  _buf[CRSF_MAX_PACKET_SIZE];
    int      _buf_pos = 0;
    uint8_t  _link_quality = 0;
    int8_t   _rssi = 0;

    void parseFrame(const uint8_t* frame, int len);
    void decodeRcChannels(const uint8_t* payload);
    void decodeLinkStats(const uint8_t* payload);
    static uint8_t crc8(const uint8_t* data, int len);
};
