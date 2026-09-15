#include "motor_manager.h"
#include <cassert>
#include <cmath>
#include <iostream>

static twai_message_t packet(uint8_t id, uint8_t type=2, uint8_t length=8) {
    twai_message_t m{};m.extd=true;m.identifier=(uint32_t(type)<<24)|(uint32_t(id)<<8)|0xFD;
    m.data_length_code=length;return m;
}
static void field(twai_message_t& m, int i, uint16_t value) {m.data[i]=value>>8;m.data[i+1]=value;}
int main() {
    Robstride can;MotorManager motors;
    assert(can.begin(5,6));motors.begin(&can);
    assert(fake_can_rx_capacity==64);
    auto wheel=packet(20);field(wheel,0,32768);field(wheel,2,65535);field(wheel,4,65535);field(wheel,6,250);
    auto arm=wheel;arm.identifier=(2u<<24)|(1u<<8)|0xFD;
    fake_can_rx.push_back(wheel);fake_can_rx.push_back(arm);fake_ms=100;
    motors.processFeedback();
    assert(std::fabs(motors.getMotor(MotorRole::BackRight).velocity-50)<.001);
    assert(std::fabs(motors.getMotor(MotorRole::BackRight).torque-5.5)<.001);
    assert(std::fabs(motors.getMotor(MotorRole::ArmLeft).velocity-33)<.001);
    assert(std::fabs(motors.getMotor(MotorRole::ArmLeft).torque-14)<.001);
    const auto position=motors.getMotor(MotorRole::BackRight).position;
    // Acknowledgements, short frames and unknown traffic are not motion.
    fake_can_rx.push_back(packet(20,18));fake_can_rx.push_back(packet(20,2,3));
    fake_can_rx.push_back(packet(20,31));fake_can_rx.push_back(twai_message_t{});
    auto fault=packet(20,21);fault.data[0]=4;fake_can_rx.push_back(fault);
    fake_can_rx.push_back(packet(20,18));fake_ms=200;motors.processFeedback();
    const auto& m=motors.getMotor(MotorRole::BackRight);
    assert(m.position==position && m.velocity==50 && m.last_feedback_ms==100);
    assert(m.has_fault && m.errors==4 && fake_can_rx.empty());
    fake_can_rx.push_back(wheel);motors.processFeedback();assert(!m.has_fault && m.last_feedback_ms==200);
    // Model mapping follows the configured role, not fixed CAN ID numbers.
    motors.setMotorId(MotorRole::BackLeft,77);
    wheel.identifier=(2u<<24)|(77u<<8)|0xFD;fake_can_rx.push_back(wheel);motors.processFeedback();
    assert(motors.getMotor(MotorRole::BackLeft).velocity==-50);
    // High-rate queue: one call drains 64 frames, not the old 16-frame limit.
    for(int i=0;i<100;++i)fake_can_rx.push_back(wheel);
    motors.processFeedback();assert(fake_can_rx.size()==36);motors.processFeedback();assert(fake_can_rx.empty());
    // Time budget is independent of traffic count and survives clock rollover.
    fake_can_read_cost_us=500;fake_us=UINT32_MAX-700;
    for(int i=0;i<20;++i)fake_can_rx.push_back(wheel);
    motors.processFeedback();assert(fake_can_rx.size()==17);fake_can_rx.clear();fake_can_read_cost_us=10;
    // Correct model ranges also apply to MIT encoding (not used for balance).
    assert(can.sendMITControl(77,0xFD,0,50,0,0,5.5));
    const auto sent=fake_can_tx.back();assert(sent.data[2]==255 && sent.data[3]==255);
    assert(((sent.identifier>>8)&65535)==65535);
    fake_can_tx_fail=true;assert(!can.sendPositionCommand(77,0xFD,0,1));
    std::cout<<"Motor feedback tests passed: RS05/RS00, remapping, malformed/ack/fault freshness, receive budgets\n";
}
