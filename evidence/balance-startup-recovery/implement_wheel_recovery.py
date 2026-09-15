from pathlib import Path
p=Path('src/balance_controller.cpp');s=p.read_text()
def edit(old,new):
 global s
 assert old in s,old
 s=s.replace(old,new,1)
edit('    _vel_sp_integral    = 0.0f;', '''    _startup_detector.reset();
    _startup_recovery.reset();
    _hold_drift = 0.0f;
    _vel_sp_integral    = 0.0f;''')
edit('''            if (!_ramp_complete) {
                // The ramp waits''','''            if (!_ramp_complete && !_startup_recovery.triggered()) {
                // The ramp waits''')
edit('''            // Origin stays at ENGAGE: the position loop (active only from
            // now on, so no mid-ramp fighting -- lesson 13 concern) walks
            // the robot back to where it was stood up, undoing the
            // unavoidable standup travel.
            // Stored trim is already inside the base setpoint (delivered by
            // the ramp); the integrator learns only the residual, from zero.
            // _sp_offset and _filtered_wheel_vel are NOT reset: the velocity
            // damper has been running through the ramp and both are already
            // continuous (resets here caused the 154823 setpoint step).
            _vel_sp_integral = 0.0f;''','''            // A detected runaway has already started learning equilibrium.
            // Keep that correction through handoff; resetting it would remove
            // the catch just as the arms arrive. Ordinary starts retain the
            // previous zero-integral handoff and original position reference.
            if (!_startup_recovery.triggered()) _vel_sp_integral = 0.0f;''')
edit('''        // Velocity damper runs from ENGAGE (during the base ramp the PD
        // chases the rising setpoint and can surge away -- +10 rad drift in
        // run 164837). Position return and learning only after the ramp.''','''        // Velocity damping runs from ENGAGE. Confirm sustained outward motion
        // after the measured arms start returning, using fresh wheel feedback.
        // Recovery starts learning now, while normal learning is still gated.''')
edit('''        float target_vel = 0.0f;
        if (_ramp_complete) {''','''        const bool recovery_feedback_fresh =
            fb_age_l <= BALANCE_START_RECOVERY_FEEDBACK_MS
            && fb_age_r <= BALANCE_START_RECOVERY_FEEDBACK_MS;
        const balance_math::RunawayConfig recovery_detector_config = {
            BALANCE_START_RECOVERY_SPEED, BALANCE_START_RECOVERY_FORCE_SPEED,
            BALANCE_START_RECOVERY_ACCEL, BALANCE_START_RECOVERY_ACCEL_TAU,
            BALANCE_START_RECOVERY_CONFIRM_MS
        };
        if (!_startup_recovery.triggered()) {
            const bool eligible = _arms_returning && !_ramp_complete
                && computeArmFraction() <= BALANCE_START_RECOVERY_TIP_MAX
                && recovery_feedback_fresh;
            const float direction = _startup_detector.update(
                eligible, _filtered_wheel_vel, dt, recovery_detector_config);
            if (direction && _startup_recovery.start(now)) {
                Serial.printf("[Balance] Early roll recovery: v=%.2f, drift=%.2f, direction=%.0f\\n",
                              _filtered_wheel_vel, meas_drift, direction);
            }
        }
        const bool recovery_calm = _ramp_complete && recovery_feedback_fresh
            && fabsf(_filtered_wheel_vel) < BALANCE_START_RECOVERY_CALM_VEL
            && fabsf(rate) < BALANCE_START_RECOVERY_CALM_RATE
            && fabsf(_effective_setpoint - tilt) < BALANCE_START_RECOVERY_CALM_ERR;
        if (_startup_recovery.settle(recovery_calm, dt, BALANCE_START_RECOVERY_CALM_MS)) {
            // Hold where recovery settled; do not immediately request a trip
            // back through all the stand-up travel. Log odometry from ENGAGE
            // unchanged so this cannot hide displacement in the evidence.
            _hold_drift = meas_drift;
            _last_outer_diag |= BAL_DIAG_RECOVERY_SETTLED;
            Serial.printf("[Balance] Early roll recovery settled: hold drift=%.2f, integral=%.2f\\n",
                          _hold_drift, _vel_sp_integral);
        }
        const bool recovering = _startup_recovery.active();
        const bool recovery_boost = _startup_recovery.boosting(
            now, _ramp_complete, BALANCE_START_RECOVERY_BOOST_MS);
        if (recovering) _last_outer_diag |= BAL_DIAG_START_RECOVERY;
        if (recovery_boost) _last_outer_diag |= BAL_DIAG_RECOVERY_BOOST;

        float target_vel = 0.0f;
        if (recovering) {
            // Arrest wheel motion before requesting position correction.
        } else if (_ramp_complete) {''')
edit('''target_vel = clampf(-_drift_vel_kp * meas_drift,''','''target_vel = clampf(-_drift_vel_kp * (meas_drift - _hold_drift),''')
edit('''        if (_ramp_complete) {
            // Glide detection''','''        if (recovering) {
            // One integrator, bounded gain/time/angle/rate. The initial catch
            // is not suppressed by angle-error gating; normal P damping still
            // respects that gate. End the boost at 800 ms or ramp completion.
            const float ki = recovery_boost ? BALANCE_START_RECOVERY_KI : _vel_sp_ki;
            const auto next = balance_math::recoveryIntegral(
                _vel_sp_integral, vel_err, ki, dt, _sp_offset,
                BALANCE_START_RECOVERY_LIMIT_DEG, BALANCE_START_RECOVERY_RATE_DPS,
                recovery_feedback_fresh);
            _vel_sp_integral = next.value;
            if (next.limited) _last_outer_diag |= BAL_DIAG_RECOVERY_LIMIT;
        } else if (_ramp_complete) {
            // Glide detection''')
edit('''        float sp_offset_unclamped = (p_term * pos_gate * shed) + _vel_sp_integral;''','''        if (recovering) off_max = BALANCE_START_RECOVERY_LIMIT_DEG;
        float sp_offset_unclamped = (p_term * pos_gate * shed) + _vel_sp_integral;''')
edit('''            _last_outer_diag |= BAL_DIAG_SP_CLAMPED;
        }
        _last_sp_offset_target''','''            _last_outer_diag |= BAL_DIAG_SP_CLAMPED;
            if (recovering) _last_outer_diag |= BAL_DIAG_RECOVERY_LIMIT;
        }
        _last_sp_offset_target''')
edit('''                    | (_ramp_complete ? 0x40 : 0);''','''                    | (_ramp_complete ? 0x40 : 0)
                    | (_startup_recovery.triggered() ? 0x80 : 0);''')
edit('''    _log_config.control_loop_hz = CONTROL_LOOP_HZ;''','''    _log_config.control_loop_hz = CONTROL_LOOP_HZ;
    _log_config.reserved[0] = static_cast<uint8_t>(BALANCE_START_RECOVERY_SPEED * 10 + .5f);
    _log_config.reserved[1] = static_cast<uint8_t>(BALANCE_START_RECOVERY_FORCE_SPEED * 10 + .5f);
    _log_config.reserved[2] = BALANCE_START_RECOVERY_CONFIRM_MS / 10;''')
edit('''header.reserved = 15;  // IMU age, RS05 units, fast CAN RX, absolute capture trim''','''header.reserved = 31;  // prior extensions plus wheel-feedback startup recovery v1''')
edit('''    if (header.reserved & 8) {''','''    if (header.reserved & 16) {
        out.println("# startup_recovery=wheel_velocity_learning_v1");
        out.printf("# startup_recovery_speed=%.1f\\n", header.config.reserved[0] * .1f);
        out.printf("# startup_recovery_force_speed=%.1f\\n", header.config.reserved[1] * .1f);
        out.printf("# startup_recovery_confirm_ms=%u\\n", header.config.reserved[2] * 10);
        // Versioned constants: these describe v1, never the running config of
        // firmware that happens to download an older stored log.
        out.println("# startup_recovery_ki=1.0 boost_ms=800 integral_rate_dps=6 limit_deg=6");
        out.println("# startup_recovery_hold=settled_position calm_ms=400");
    }
    if (header.reserved & 8) {''')
p.write_text(s)
