#include "radio_status.h"
#include "crsf.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>

static uint8_t crc(const std::vector<uint8_t>& f) {
    uint8_t result = 0;
    for (size_t i = 2; i < f.size() - 1; ++i) {
        result ^= f[i];
        for (int bit = 0; bit < 8; ++bit)
            result = (result & 128) ? (result << 1) ^ 0xD5 : result << 1;
    }
    return result;
}
static void fixture(std::ofstream& out, const char* name, const uint8_t* p, size_t n) {
    out << name << "={";
    for (size_t i=0; i<n; ++i) out << unsigned(p[i]) << ',';
    out << "},\n";
}
int main() {
    using namespace radio_status;
    assert(angle(-12.345f) == -1235 && angle(12.345f) == 1235);
    assert(angle(std::numeric_limits<float>::quiet_NaN()) == INT16_MIN);
    assert(angle(-1000) == -32767 && angle(1000) == 32767);
    assert(unsignedScaled(-3,10) == 0 && unsignedScaled(25.2f,10) == 252);
    assert(unsignedScaled(1e9f,10) == 65535);
    assert(unsignedScaled(std::numeric_limits<float>::infinity(),10) == 0);
    Snapshot s;
    s.sequence=0xABCD; s.flags=DriveArmed|RcLinked|ImuFresh|VoltageFresh|CurrentFresh;
    s.online=63; s.enabled=15; s.max_temp=43; s.tilt=-1235; s.error=75;
    s.voltage=252; s.motor_current=123; s.mode=1; s.label="WHEELS READY";
    uint8_t payload[STATUS_SIZE]; encode(s,payload);
    assert(payload[0]==0xEA && payload[1]==0xC8 && payload[2]=='H' && payload[3]=='S');
    assert(payload[6]==0xAB && payload[7]==0xCD && payload[10]==1);
    assert(payload[18]==0xFB && payload[19]==0x2D && payload[20]==0 && payload[21]==75);
    assert(payload[22]==0 && payload[23]==252 && payload[32]=='W' && payload[47]==0);
    CrsfReceiver rx; HardwareSerial uart;
    assert(!rx.sendRobotTelemetry(payload,sizeof(payload)));
    rx.begin(uart,1,2,420000);
    assert(rx.sendRobotTelemetry(payload,sizeof(payload)));
    assert(uart.tx.size()==52 && uart.tx[1]==50 && uart.tx[2]==0x7E);
    assert(uart.tx.back()==crc(uart.tx));
    uart.tx.clear(); uart.tx_space=51;
    assert(!rx.sendRobotTelemetry(payload,sizeof(payload)) && uart.tx.empty());
    assert(rx.telemetryDrops()==1);
    uart.tx_space=0;
    rx.sendFlightMode("BALANCE"); rx.sendBatteryTelemetry(25.2f,1); rx.sendAttitudeTelemetry(0,0,0);
    assert(uart.tx.empty() && rx.telemetryDrops()==4);
    uart.tx_space=128;
    assert(!rx.sendRobotTelemetry(nullptr,4) && !rx.sendRobotTelemetry(payload,61));
    std::ofstream fixtures("output/radio-fixtures.lua"); assert(fixtures);
    fixtures << "return {\n"; fixture(fixtures,"drive",payload,sizeof(payload));
    s.flags |= ArmsArmed; s.enabled=63; s.mode=2; s.phase=4; s.motion=256;
    s.tilt=8825; s.error=-45; s.label="BALANCE"; ++s.sequence; encode(s,payload);
    fixture(fixtures,"balance",payload,sizeof(payload));
    s.faults=2; s.inner_fault=1024; ++s.sequence; encode(s,payload);
    fixture(fixtures,"fault",payload,sizeof(payload));
    uint8_t detail[DETAIL_SIZE]; encodeDetail(15,0x12345678,"tilt out of range (fallen)",detail);
    assert(detail[8]==0x12 && detail[11]==0x78 && detail[59]==0);
    fixture(fixtures,"detail",detail,sizeof(detail)); fixtures << "}\n";
    assert(rx.sendRobotTelemetry(detail,sizeof(detail)));
    assert(uart.tx.size()==64 && uart.tx.back()==crc(uart.tx));
    std::cout << "Radio encoding, full-frame TX backpressure, CRC and C++/Lua fixtures passed\n";
}
