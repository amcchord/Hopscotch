#pragma once

// Control-task ownership gate. Selecting balance on CH7 does not own the
// wheels: a pending start or an active balance state does. Inputs here have
// already passed through the RC deadband.
class GroundDriveGate {
public:
    bool update(bool balance_active, bool start_pending, bool input_valid,
                float throttle, float steering) {
        if (balance_active || start_pending || !input_valid) {
            _needs_neutral = true;
            return false;
        }

        // A held standing-drive command must not launch the ground controller
        // after a fall/abort/return. The same rule covers a canceled start or
        // restored radio link. Neutral must be observed after ownership ends.
        if (_needs_neutral) {
            if (throttle != 0.0f || steering != 0.0f) return false;
            _needs_neutral = false;
        }
        return true;
    }

private:
    bool _needs_neutral = false;
};
