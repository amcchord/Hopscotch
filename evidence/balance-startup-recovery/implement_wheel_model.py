from pathlib import Path
p=Path('scripts/balance_sim.py');s=p.read_text()
def edit(old,new):
 global s
 assert old in s,old
 s=s.replace(old,new,1)
edit('    base_sp_rate_max: float = 3.0','''    startup_recovery: bool = False  # historical variants leave the early catch off
    startup_speed: float = 1.0
    startup_force_speed: float = 4.0
    startup_accel: float = 2.0
    startup_accel_tau: float = .06
    startup_confirm_ms: float = 60
    startup_tip_max: float = .9
    recovery_ki: float = 1.0
    recovery_boost_ms: float = 800
    recovery_limit: float = 6.0
    recovery_rate: float = 6.0
    recovery_calm_vel: float = .7
    recovery_calm_rate: float = 4.0
    recovery_calm_err: float = 1.0
    recovery_calm_ms: float = 400
    base_sp_rate_max: float = 3.0''')
edit('        self.vel_sp_integral = 0.0','''        self.startup_triggered = False
        self.startup_active = False
        self.startup_ms = 0.
        self.recovery_calm_ms = 0.
        self.startup_accel = 0.
        self.startup_previous_vel = None
        self.startup_qualifying_ms = 0.
        self.startup_direction = 0.
        self.hold_drift = 0.
        self.vel_sp_integral = 0.0''')
edit('''            if not self.ramp_complete:
                vel_gate =''','''            if not self.ramp_complete and not self.startup_triggered:
                vel_gate =''')
edit('''                self.vel_sp_integral = 0.0

        # --- capture''','''                if not self.startup_triggered:
                    self.vel_sp_integral = 0.0

        # --- capture''')
edit('''        target_vel = 0.0
        if self.ramp_complete:''','''        # The plant model supplies fresh wheel samples; CAN loss and the
        # production 30 ms eligibility guard are covered separately in native tests.
        if c.startup_recovery and not self.startup_triggered:
            acceleration = (0. if self.startup_previous_vel is None else
                            (self.filtered_wheel_vel-self.startup_previous_vel)/dt)
            self.startup_previous_vel = self.filtered_wheel_vel
            self.startup_accel += clampf(dt/c.startup_accel_tau,0,1)*(acceleration-self.startup_accel)
            direction = 1. if self.filtered_wheel_vel>0 else -1.
            eligible = self.arms_returning and not self.ramp_complete and tip_frac<=c.startup_tip_max
            qualifies = eligible and abs(self.filtered_wheel_vel)>=c.startup_speed and (
                direction*self.startup_accel>=c.startup_accel or abs(self.filtered_wheel_vel)>=c.startup_force_speed)
            if not qualifies:
                self.startup_qualifying_ms = 0.
                self.startup_direction = 0.
            else:
                if direction != self.startup_direction:
                    self.startup_qualifying_ms = 0.
                    self.startup_direction = direction
                self.startup_qualifying_ms += dt*1000
                if self.startup_qualifying_ms+.001>=c.startup_confirm_ms:
                    self.startup_triggered = self.startup_active = True
                    self.startup_ms = now_ms
        if self.startup_active:
            calm = (self.ramp_complete and abs(self.filtered_wheel_vel)<c.recovery_calm_vel
                    and abs(rate)<c.recovery_calm_rate
                    and abs(self.effective_setpoint-tilt)<c.recovery_calm_err)
            self.recovery_calm_ms = self.recovery_calm_ms+dt*1000 if calm else 0.
            if self.recovery_calm_ms+.001>=c.recovery_calm_ms:
                self.startup_active = False
                self.hold_drift = meas_drift
        target_vel = 0.0
        if self.startup_active:
            pass  # stop first, then hold the settled position
        elif self.ramp_complete:''')
edit('''-c.drift_vel_kp * meas_drift,''','''-c.drift_vel_kp * (meas_drift-self.hold_drift),''')
edit('''        if self.ramp_complete:
            ki = c.vel_sp_ki''','''        if self.startup_active:
            ki = (c.recovery_ki if not self.ramp_complete and now_ms-self.startup_ms<c.recovery_boost_ms
                  else c.vel_sp_ki)
            change = clampf(ki*vel_err,-c.recovery_rate,c.recovery_rate)*dt
            if abs(self.sp_offset)<c.recovery_limit-.05 or change*self.sp_offset<0:
                self.vel_sp_integral = clampf(self.vel_sp_integral+change,-c.recovery_limit,c.recovery_limit)
        elif self.ramp_complete:
            ki = c.vel_sp_ki''')
edit('''        sp_offset_target = clampf(p_term * pos_gate * shed + self.vel_sp_integral,''','''        if self.startup_active:
            off_max = c.recovery_limit
        sp_offset_target = clampf(p_term * pos_gate * shed + self.vel_sp_integral,''')
edit("        'glide_cmd_max': 'BALANCE_GLIDE_CMD_MAX',", """        'glide_cmd_max': 'BALANCE_GLIDE_CMD_MAX',
        'startup_speed': 'BALANCE_START_RECOVERY_SPEED',
        'startup_force_speed': 'BALANCE_START_RECOVERY_FORCE_SPEED',
        'startup_accel': 'BALANCE_START_RECOVERY_ACCEL',
        'startup_accel_tau': 'BALANCE_START_RECOVERY_ACCEL_TAU',
        'startup_confirm_ms': 'BALANCE_START_RECOVERY_CONFIRM_MS',
        'startup_tip_max': 'BALANCE_START_RECOVERY_TIP_MAX',
        'recovery_ki': 'BALANCE_START_RECOVERY_KI',
        'recovery_boost_ms': 'BALANCE_START_RECOVERY_BOOST_MS',
        'recovery_limit': 'BALANCE_START_RECOVERY_LIMIT_DEG',
        'recovery_rate': 'BALANCE_START_RECOVERY_RATE_DPS',
        'recovery_calm_vel': 'BALANCE_START_RECOVERY_CALM_VEL',
        'recovery_calm_rate': 'BALANCE_START_RECOVERY_CALM_RATE',
        'recovery_calm_err': 'BALANCE_START_RECOVERY_CALM_ERR',
        'recovery_calm_ms': 'BALANCE_START_RECOVERY_CALM_MS',""")
edit('''                   absolute_capture_trim=True,''','''                   absolute_capture_trim=True, startup_recovery=True,''')
edit('''measured_arm_arrival=False, hard_stop_latches=False, absolute_capture_trim=False)''','''measured_arm_arrival=False, hard_stop_latches=False, absolute_capture_trim=False,
                       startup_recovery=False)''')
p.write_text(s)
