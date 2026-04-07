#include "crsf.h"
#include <Arduino.h>
#include "config.h"

// CRC8 with poly 0xD5 (CRSF/DVB-S2 standard)
static const uint8_t crc8_lut[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9,
};

uint8_t CrsfReceiver::crc8(const uint8_t* data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crc8_lut[crc ^ data[i]];
    }
    return crc;
}

void CrsfReceiver::begin(HardwareSerial& serial, int rx_pin, int tx_pin, uint32_t baudrate) {
    _serial = &serial;
    _serial->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);

    // Initialize channels to center
    for (int i = 0; i < CRSF_MAX_CHANNELS; i++) {
        _channels[i] = CRSF_CHANNEL_MID;
    }

    _buf_pos = 0;
    _last_frame_time = 0;
    Serial.printf("[CRSF] Initialized on RX=%d TX=%d @ %lu baud\n", rx_pin, tx_pin, baudrate);
}

void CrsfReceiver::update() {
    if (!_serial) return;

    while (_serial->available()) {
        uint8_t b = _serial->read();

        if (_buf_pos == 0) {
            // Looking for sync byte
            if (b == CRSF_SYNC_BYTE || b == 0xEE || b == 0xEA || b == 0xEC) {
                _buf[0] = b;
                _buf_pos = 1;
            }
            continue;
        }

        _buf[_buf_pos++] = b;

        if (_buf_pos == 2) {
            // _buf[1] = frame length (includes type + payload + crc, NOT sync and length)
            if (_buf[1] < 2 || _buf[1] > CRSF_MAX_PACKET_SIZE - 2) {
                _buf_pos = 0;
            }
            continue;
        }

        int frame_total = _buf[1] + 2;  // sync + len + (type + payload + crc)

        if (_buf_pos >= frame_total) {
            // Full frame received
            // CRC covers from type byte to end of payload (excludes sync, length, and crc itself)
            uint8_t calc_crc = crc8(&_buf[2], _buf[1] - 1);
            uint8_t recv_crc = _buf[frame_total - 1];

            if (calc_crc == recv_crc) {
                parseFrame(_buf, frame_total);
            }
            _buf_pos = 0;
        }

        if (_buf_pos >= CRSF_MAX_PACKET_SIZE) {
            _buf_pos = 0;
        }
    }
}

void CrsfReceiver::parseFrame(const uint8_t* frame, int len) {
    uint8_t type = frame[2];

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        decodeRcChannels(&frame[3]);
        _last_frame_time = millis();
    } else if (type == CRSF_FRAMETYPE_LINK_STATISTICS) {
        decodeLinkStats(&frame[3]);
    }
}

void CrsfReceiver::decodeRcChannels(const uint8_t* payload) {
    // 16 channels x 11 bits = 176 bits = 22 bytes, packed LSB-first
    _channels[0]  = ((uint16_t)payload[0]       | (uint16_t)payload[1]  << 8) & 0x07FF;
    _channels[1]  = ((uint16_t)payload[1]  >> 3  | (uint16_t)payload[2]  << 5) & 0x07FF;
    _channels[2]  = ((uint16_t)payload[2]  >> 6  | (uint16_t)payload[3]  << 2 | (uint16_t)payload[4]  << 10) & 0x07FF;
    _channels[3]  = ((uint16_t)payload[4]  >> 1  | (uint16_t)payload[5]  << 7) & 0x07FF;
    _channels[4]  = ((uint16_t)payload[5]  >> 4  | (uint16_t)payload[6]  << 4) & 0x07FF;
    _channels[5]  = ((uint16_t)payload[6]  >> 7  | (uint16_t)payload[7]  << 1 | (uint16_t)payload[8]  << 9) & 0x07FF;
    _channels[6]  = ((uint16_t)payload[8]  >> 2  | (uint16_t)payload[9]  << 6) & 0x07FF;
    _channels[7]  = ((uint16_t)payload[9]  >> 5  | (uint16_t)payload[10] << 3) & 0x07FF;
    _channels[8]  = ((uint16_t)payload[11]       | (uint16_t)payload[12] << 8) & 0x07FF;
    _channels[9]  = ((uint16_t)payload[12] >> 3  | (uint16_t)payload[13] << 5) & 0x07FF;
    _channels[10] = ((uint16_t)payload[13] >> 6  | (uint16_t)payload[14] << 2 | (uint16_t)payload[15] << 10) & 0x07FF;
    _channels[11] = ((uint16_t)payload[15] >> 1  | (uint16_t)payload[16] << 7) & 0x07FF;
    _channels[12] = ((uint16_t)payload[16] >> 4  | (uint16_t)payload[17] << 4) & 0x07FF;
    _channels[13] = ((uint16_t)payload[17] >> 7  | (uint16_t)payload[18] << 1 | (uint16_t)payload[19] << 9) & 0x07FF;
    _channels[14] = ((uint16_t)payload[19] >> 2  | (uint16_t)payload[20] << 6) & 0x07FF;
    _channels[15] = ((uint16_t)payload[20] >> 5  | (uint16_t)payload[21] << 3) & 0x07FF;
}

void CrsfReceiver::decodeLinkStats(const uint8_t* payload) {
    // payload[0] = uplink RSSI ant 1 (dBm, negate)
    // payload[1] = uplink RSSI ant 2
    // payload[2] = uplink link quality (0-100)
    // payload[3] = uplink SNR
    _rssi = -(int8_t)payload[0];
    _link_quality = payload[2];
}

uint16_t CrsfReceiver::getChannel(int ch) const {
    if (ch < 0 || ch >= CRSF_MAX_CHANNELS) return CRSF_CHANNEL_MID;
    return _channels[ch];
}

float CrsfReceiver::getChannelNormalized(int ch) const {
    uint16_t raw = getChannel(ch);
    float mid = static_cast<float>(CRSF_CHANNEL_MID);
    float half_range = static_cast<float>(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID);
    float val = (static_cast<float>(raw) - mid) / half_range;
    if (val < -1.0f) val = -1.0f;
    if (val > 1.0f) val = 1.0f;
    return val;
}

bool CrsfReceiver::isLinkUp() const {
    if (_last_frame_time == 0) return false;
    return (millis() - _last_frame_time) < CRSF_TIMEOUT_MS;
}

uint32_t CrsfReceiver::timeSinceLastFrame() const {
    if (_last_frame_time == 0) return UINT32_MAX;
    return millis() - _last_frame_time;
}
