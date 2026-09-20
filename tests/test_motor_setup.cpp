// Exercise the production MotorManager with a simulated RS05 transport.
// No hardware is accessed. ACC_RAD behaves like the attached motors: writes
// work, but reads return zero (or no reply), never the configured value.
#include "motor_manager.h"
#include <Arduino.h>
#include <cassert>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <utility>

namespace {
std::deque<RobstrideFeedback> replies;
std::map<std::pair<uint8_t, uint16_t>, float> parameters;
int acceleration_reads = 0, acceleration_writes = 0, current_reads = 0;
int acceleration_tx_failures = 0, current_tx_failures = 0;
int current_bad_replies = 0, stop_count = 0;
int ping_count = 0;
uint8_t ping_id = 0;
bool ping_ok = true;
bool acceleration_reply = true, current_reply = true;
float bad_current = 5.0f;

void resetTransport() {
    replies.clear(); parameters.clear(); Serial.bytes.clear();
    acceleration_reads = acceleration_writes = current_reads = stop_count = 0;
    acceleration_tx_failures = current_tx_failures = current_bad_replies = 0;
    acceleration_reply = current_reply = true;
    bad_current = 5.0f;
}

void armSimulatedDrive(MotorManager& motors, Robstride& can) {
    resetTransport();
    motors.begin(&can);
    motors.requestArmDrive();
    for (int tick = 0; tick < 200 && motors.isArming(); ++tick) {
        motors.updateArming();
        motors.processFeedback();
        delay(20);
    }
    assert(motors.isDriveArmed());
    for (int i = 0; i < NUM_DRIVE_MOTORS; ++i) assert(motors.getMotor(i).enabled);
    resetTransport();
}
}

bool Robstride::stopMotor(uint8_t, uint8_t, bool) { ++stop_count; return true; }
bool Robstride::setRunMode(uint8_t, uint8_t, RobstrideRunMode) { return true; }
bool Robstride::enableMotor(uint8_t, uint8_t) { return true; }
bool Robstride::writeU32Param(uint8_t, uint8_t, uint16_t, uint32_t) { return true; }
bool Robstride::sendPositionCommand(uint8_t, uint8_t, float, float) { return true; }
bool Robstride::sendMotionPing(uint8_t id) { ++ping_count; ping_id=id; return ping_ok; }
bool Robstride::changeMotorCanId(uint8_t, uint8_t, uint8_t) { return true; }

bool Robstride::writeFloatParam(uint8_t id, uint8_t, uint16_t addr, float value) {
    if (addr == RobstrideParam::ACC_RAD) {
        ++acceleration_writes;
        if (acceleration_tx_failures > 0) { --acceleration_tx_failures; return false; }
    }
    if (addr == RobstrideParam::CURRENT_LIMIT && current_tx_failures > 0) {
        --current_tx_failures; return false;
    }
    parameters[{id, addr}] = value;
    return true;
}

bool Robstride::readParam(uint8_t id, uint8_t, uint16_t addr) {
    RobstrideFeedback reply{};
    reply.valid = true;
    reply.is_param_response = true;
    reply.motor_id = id;
    reply.param_addr = addr;
    reply.param_value = 1.25f;
    replies.push_back(reply);
    return true;
}

bool Robstride::receiveFeedback(RobstrideFeedback& reply, uint32_t) {
    if (replies.empty()) return false;
    reply = replies.front(); replies.pop_front(); return true;
}

bool Robstride::readParamSync(uint8_t id, uint8_t, uint16_t addr, float& value, uint32_t) {
    if (addr == RobstrideParam::ACC_RAD) {
        ++acceleration_reads;
        value = 0.0f;
        return acceleration_reply;
    }
    assert(addr == RobstrideParam::CURRENT_LIMIT);
    ++current_reads;
    if (!current_reply) return false;
    if (current_bad_replies > 0) { --current_bad_replies; value = bad_current; }
    else value = parameters.at({id, addr});
    return true;
}

int main() {
    Robstride can;
    MotorManager motors;
    // An unarmed robot must not enter Speed mode, even with a working bus.
    resetTransport(); motors.begin(&can);
    assert(!motors.setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed, 100, 10));
    assert(acceleration_writes == 0);

    for (bool acc_has_reply : {true, false}) {
        armSimulatedDrive(motors, can);
        acceleration_reply = acc_has_reply;
        for (MotorRole role : {MotorRole::BackLeft, MotorRole::BackRight}) {
            assert(motors.setDriveRunMode(role, RobstrideRunMode::Speed, 100, 10));
            const auto& motor = motors.getMotor(role);
            assert(motor.enabled && motor.run_mode == RobstrideRunMode::Speed);
            assert(parameters.at({motor.can_id, RobstrideParam::ACC_RAD}) == 100);
            assert(parameters.at({motor.can_id, RobstrideParam::TARGET_SPEED}) == 0);
        }
        assert(acceleration_reads == 0 && current_reads == 2);
    }

    // Lost transmissions retry; persistent failure stops and refuses the motor.
    armSimulatedDrive(motors, can);
    acceleration_tx_failures = 2;
    assert(motors.setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed, 100, 10));
    assert(acceleration_writes == 3 && acceleration_reads == 0);

    armSimulatedDrive(motors, can);
    acceleration_tx_failures = 100;
    assert(!motors.setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed, 100, 10));
    assert(acceleration_writes == 4 && current_reads == 0 && stop_count == 2);
    assert(!motors.getMotor(MotorRole::BackLeft).enabled);

    // Current-limit readback remains mandatory, including missing/nonfinite data.
    for (int failure = 0; failure < 4; ++failure) {
        armSimulatedDrive(motors, can);
        if (failure == 0) current_reply = false;
        if (failure == 1) current_bad_replies = 100;
        if (failure == 2) { current_bad_replies = 100; bad_current = std::numeric_limits<float>::quiet_NaN(); }
        if (failure == 3) current_tx_failures = 100;
        assert(!motors.setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed, 100, 10));
        assert(!motors.getMotor(MotorRole::BackLeft).enabled && stop_count == 2);
        assert(current_reads == (failure == 3 ? 0 : 4));
        if (failure == 0) assert(Serial.bytes.find("readback missing") != std::string::npos);
    }
    armSimulatedDrive(motors, can);
    current_bad_replies = 2;
    assert(motors.setDriveRunMode(MotorRole::BackLeft, RobstrideRunMode::Speed, 100, 10));
    assert(current_reads == 3);
    // Fast tip-up requests motion samples without manufacturing freshness or
    // changing a target/run mode. It works for mapped IDs and propagates TX failure.
    motors.setMotorId(MotorRole::ArmLeft, 17);
    const auto last_sample=motors.getMotor(MotorRole::ArmLeft).last_feedback_ms;
    const auto current_mode=motors.getMotor(MotorRole::ArmLeft).run_mode;
    const auto writes=parameters;
    ping_count=0;
    assert(motors.requestMotionFeedback(MotorRole::ArmLeft) && ping_id==17 && ping_count==1);
    assert(motors.getMotor(MotorRole::ArmLeft).last_feedback_ms==last_sample);
    assert(motors.getMotor(MotorRole::ArmLeft).run_mode==current_mode && parameters==writes);
    ping_ok=false;
    assert(!motors.requestMotionFeedback(MotorRole::ArmLeft));
    assert(!motors.requestMotionFeedback(static_cast<MotorRole>(NUM_MOTORS)));
    std::cout << "Motor setup checks passed: RS05 write-only acceleration, both rear motors, retry/failure paths, mandatory current readback\n";
}
