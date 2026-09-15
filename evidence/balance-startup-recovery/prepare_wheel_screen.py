from pathlib import Path
out=Path(__file__).parent
text=Path('scripts/balance_sim.py').read_text()
(out/'arm-screen-simulator.py').write_text(text)
def edit(old,new):
 global text
 assert old in text,old
 text=text.replace(old,new,1)
edit('    startup_recovery: bool = False','''    wheel_recovery: bool = False
    recovery_ki: float = 1.0
    recovery_limit: float = 6.0
    recovery_rate: float = 6.0
    startup_recovery: bool = False''')
edit('        self.startup_ms = 0.','''        self.recovery_calm_ms = 0.
        self.recovery_done = False
        self.startup_ms = 0.''')
edit('                self.vel_sp_integral = 0.0\n\n        # --- capture', '''                if not (c.wheel_recovery and self.startup_recovery):
                    self.vel_sp_integral = 0.0

        # --- capture''')
edit('        if c.startup_recovery and not self.startup_recovery:', '        if (c.startup_recovery or c.wheel_recovery) and not self.startup_recovery:')
edit('                    self.arm_stage, self.arm_sign, self.arm_calm_ms = 1, direction, 0.', '''                    if c.startup_recovery:
                        self.arm_stage, self.arm_sign, self.arm_calm_ms = 1, direction, 0.''')
edit('        target_vel = 0.0\n        if self.ramp_complete:', '''        wheel_active=c.wheel_recovery and self.startup_recovery and not self.recovery_done
        if wheel_active:
            calm=(self.ramp_complete and abs(self.filtered_wheel_vel)<.7 and abs(rate)<4.
                  and abs(self.effective_setpoint-tilt)<1.)
            self.recovery_calm_ms=self.recovery_calm_ms+dt*1000 if calm else 0.
            if self.recovery_calm_ms>=400:
                self.recovery_done=True
                wheel_active=False
        target_vel = 0.0
        if wheel_active:
            pass  # arrest motion before repaying accumulated position error
        elif self.ramp_complete:''')
edit('''        if self.ramp_complete:
            ki = c.vel_sp_ki''', '''        if wheel_active:
            change=clampf(c.recovery_ki*vel_err,-c.recovery_rate,c.recovery_rate)*dt
            if abs(self.sp_offset)<c.recovery_limit-.05 or change*self.sp_offset<0:
                self.vel_sp_integral=clampf(self.vel_sp_integral+change,-c.recovery_limit,c.recovery_limit)
        elif self.ramp_complete:
            ki = c.vel_sp_ki''')
edit('elif abs_err <= c.vel_sp_knee or not self.ramp_complete:', 'elif abs_err <= c.vel_sp_knee or not self.ramp_complete or wheel_active:')
edit('''        sp_offset_target = clampf(p_term * pos_gate * shed + self.vel_sp_integral,''', '''        if wheel_active:
            off_max=c.recovery_limit
        sp_offset_target = clampf(p_term * pos_gate * shed + self.vel_sp_integral,''')
edit('        if self.ramp_complete or self.startup_recovery:', '        if self.ramp_complete or (c.startup_recovery and self.startup_recovery):')
# Freeze source-derived config outside this copied module's REPO_ROOT.
text=text.replace("REPO_ROOT = Path(__file__).resolve().parent.parent", "REPO_ROOT = Path(__file__).resolve().parents[2]")
(out/'wheel-screen-simulator.py').write_text(text)
