#include "drive_controller.h"
#include "ground_drive_gate.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>

namespace {
using Parameters = std::map<std::pair<uint8_t, uint16_t>, float>;

uint16_t address(const twai_message_t& frame) {
    return frame.data[0] | (uint16_t(frame.data[1]) << 8);
}

Parameters parameters() {
    Parameters result;
    for (const auto& frame : fake_can_tx) {
        // Ground drive must only send CSP position and speed-limit writes.
        assert((frame.identifier >> 24) == 0x12);
        const auto addr = address(frame);
        assert(addr == RobstrideParam::SPEED_LIMIT || addr == RobstrideParam::TARGET_POSITION);
        float value;
        std::memcpy(&value, frame.data + 4, sizeof(value));
        result[{uint8_t(frame.identifier), addr}] = value;
    }
    return result;
}

struct Rig {
    Robstride bus;
    MotorManager motors;
    DriveController drive;
    GroundDriveGate gate;

    Rig() {
        fake_can_tx.clear();
        fake_can_rx.clear();
        assert(bus.begin(5, 6));
        motors.begin(&bus);
        drive.begin(&motors);
    }

    void arm() {
        motors.requestArmDrive();
        size_t checked = fake_can_tx.size();
        for (int tick = 0; tick < 200 && motors.isArming(); ++tick) {
            motors.updateArming();
            for (; checked < fake_can_tx.size(); ++checked) {
                const auto& request = fake_can_tx[checked];
                if ((request.identifier >> 24) != 0x11) continue;
                assert(address(request) == RobstrideParam::MECH_POS);
                twai_message_t reply{};
                reply.extd = true;
                reply.identifier = (0x11u << 24) | ((request.identifier & 0xff) << 8) | CAN_HOST_ID;
                reply.data_length_code = 8;
                reply.data[0] = request.data[0];
                reply.data[1] = request.data[1];
                fake_can_rx.push_back(reply);
            }
            motors.processFeedback();
            delay(20);
        }
        assert(motors.isDriveArmed());
        assert(!motors.isArmArmed()); // Flat driving does not require armed arms.
        fake_can_tx.clear();
    }

    // Exercise the production gate, drive controller, motor manager, and CAN
    // encoder together. These are deadbanded RC values, as supplied by main.
    Parameters tick(float throttle, float steering, bool balance_active = false,
                    bool pending = false, bool link = true, bool balance_driving = false) {
        fake_can_tx.clear();
        const bool allowed = gate.update(balance_active, pending, link, throttle, steering);
        if (!link) drive.emergencyStop();
        else if (!balance_driving)
            drive.update(allowed ? throttle : 0, allowed ? steering : 0, .02f);
        return parameters();
    }

    void expectMotion(float throttle, float steering, const std::array<float, 4>& speeds) {
        const auto sent = tick(throttle, steering);
        assert(sent.size() == 8);
        for (int i = 0; i < 4; ++i) {
            const auto role = static_cast<MotorRole>(i);
            const auto& motor = motors.getMotor(role);
            const auto speed = speeds[i];
            assert(std::fabs(drive.getCommandedSpeed(role) - speed) < .001f);
            assert(std::fabs(sent.at({motor.can_id, RobstrideParam::SPEED_LIMIT}) - std::fabs(speed)) < .001f);
            const auto target = sent.at({motor.can_id, RobstrideParam::TARGET_POSITION});
            const auto physical_speed = motor.reversed ? -speed : speed;
            assert(target * physical_speed > 0); // Includes left-side reversal.
        }
    }

    void expectStopped(const Parameters& sent) {
        assert(sent.size() == 8);
        for (int i = 0; i < 4; ++i) {
            const auto& motor = motors.getMotor(i);
            assert(drive.getCommandedSpeed(static_cast<MotorRole>(i)) == 0);
            assert(sent.at({motor.can_id, RobstrideParam::TARGET_POSITION}) == 0);
            assert(sent.at({motor.can_id, RobstrideParam::SPEED_LIMIT}) <= .5f);
        }
    }
};
}

int main() {
    Rig rig;
    assert(rig.tick(1, 1).empty()); // Disarmed: no CAN motion frames.
    rig.arm();

    // Idle ownership is independent of the balance selector. CH1 steering and
    // CH2 throttle use the same existing arcade mix on all four ground wheels.
    rig.expectMotion(.5f, 0, {16.5f, 16.5f, 16.5f, 16.5f});
    rig.expectMotion(-.5f, 0, {-16.5f, -16.5f, -16.5f, -16.5f});
    rig.expectMotion(0, .5f, {-16.5f, -16.5f, 16.5f, 16.5f});
    rig.expectMotion(0, -.5f, {16.5f, 16.5f, -16.5f, -16.5f});
    rig.expectMotion(.8f, .4f, {13.2f, 13.2f, 33, 33});
    rig.expectStopped(rig.tick(0, 0));
    rig.expectStopped(rig.tick(0, 0));

    // Stop immediately when CH11 starts its 500ms single/double-tap window.
    rig.expectMotion(.5f, 0, {16.5f, 16.5f, 16.5f, 16.5f});
    rig.expectStopped(rig.tick(.5f, 0, false, true));
    // A canceled/refused start cannot immediately resume a held command.
    rig.expectStopped(rig.tick(.5f, 0));
    rig.expectStopped(rig.tick(0, 0));
    rig.expectMotion(.5f, 0, {16.5f, 16.5f, 16.5f, 16.5f});

    // Tip-up/balancing own the wheels exclusively, including centered sticks.
    assert(rig.tick(1, 1, true, false, true, true).empty());
    assert(rig.tick(0, 0, true, false, true, true).empty());
    rig.drive.emergencyStop(); // Production handoff resynchronizes CSP targets.
    // Arm return inhibits ground input even after CH7 is switched off.
    rig.expectStopped(rig.tick(.5f, .5f, true));
    rig.expectStopped(rig.tick(0, 0, true));
    // Neither a normal return nor a direct hard-abort to Idle leaks a held stick.
    rig.expectStopped(rig.tick(.5f, 0));
    rig.expectStopped(rig.tick(0, .5f));
    rig.expectStopped(rig.tick(0, 0));
    rig.expectMotion(.5f, 0, {16.5f, 16.5f, 16.5f, 16.5f});
    assert(rig.tick(.5f, 0, true, false, true, true).empty());
    rig.drive.emergencyStop();
    rig.expectStopped(rig.tick(.5f, 0));
    rig.expectStopped(rig.tick(0, 0));
    rig.expectMotion(0, -.5f, {16.5f, 16.5f, -16.5f, -16.5f});

    // Radio loss still freezes targets; restored held input waits for center.
    rig.expectStopped(rig.tick(1, 1, false, false, false));
    rig.expectStopped(rig.tick(1, 1));
    rig.expectStopped(rig.tick(0, 0));
    rig.expectMotion(-.5f, 0, {-16.5f, -16.5f, -16.5f, -16.5f});
    rig.motors.disarmDriveMotors();
    assert(rig.tick(1, 1).empty());

    std::cout << "Ground drive checks passed: four-wheel CSP mixing/reversal, disarm, neutral stop, pending start, balance ownership/return/abort, link loss and neutral handoff\n";
}
