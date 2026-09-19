"""Controller handoff invariants in the mirrored plant model, not hardware proof."""
import sys
import unittest
from dataclasses import replace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from balance_sim import OuterController, PlantParams, current_firmware_config, make_variant, simulate

class StartupModelTests(unittest.TestCase):
    def test_recoil_release_preserves_initial_catch(self):
        cfg=current_firmware_config()
        plant=PlantParams(feedback_velocity_scale=1.,fb_hold_ticks=1)
        old=simulate(replace(cfg,recoil_unwind=False),plant,duration_s=2.)
        new=simulate(cfg,plant,duration_s=2.)
        self.assertEqual(old.cmd,new.cmd)
        self.assertEqual(old.setpoint,new.setpoint)
        self.assertEqual(old.drift,new.drift)
        self.assertFalse(make_variant('july33').recoil_unwind)

    def test_recoil_release_requires_confirmation_and_resets_after_settle(self):
        c=self.at_forward()
        c.ramp_complete=True
        c.smoothed_base_sp=84.
        c.effective_setpoint=86.
        c.vel_sp_integral=c.sp_offset=2.
        c.filtered_wheel_vel=-1.
        for i in range(8):
            c.tick(1000+i*20,84,0,0,0,-1,0,0,.02)
            if i<2:self.assertEqual(c.recoil_blend,0.)
        self.assertAlmostEqual(c.recoil_blend,1.)
        c.startup_active=False
        before=c.vel_sp_integral
        c.tick(1160,84,0,0,0,-1,0,0,.02)
        self.assertEqual(c.recoil_blend,0.)
        self.assertLessEqual(abs(c.vel_sp_integral-before),c.cfg.vel_sp_ki*.02)

    def test_before_detection_matches_installed(self):
        cfg=current_firmware_config()
        plant=PlantParams(feedback_velocity_scale=1.,fb_hold_ticks=1)
        old=simulate(replace(cfg,startup_recovery=False),plant,duration_s=.4)
        new=simulate(cfg,plant,duration_s=.4)
        self.assertEqual(old.cmd,new.cmd)
        self.assertEqual(old.setpoint,new.setpoint)
        self.assertEqual(old.drift,new.drift)
        self.assertFalse(make_variant('july33').startup_recovery)

    def at_forward(self):
        cfg=current_firmware_config()
        c=OuterController(cfg,84.,0.,0.)
        c.arms_returning=c.arms_returned=True
        c.arm_l_target=c.arm_r_target=c.arm_l_goal=c.arm_r_goal=0.
        c.smoothed_base_sp=c.effective_setpoint=84.
        c.engage_capture_shift=c.run_curve_shift=0.
        c.startup_triggered=c.startup_active=True
        return c

    def test_ramp_completion_keeps_learned_correction(self):
        c=self.at_forward()
        c.vel_sp_integral=c.sp_offset=2.
        c.effective_setpoint=86.
        c.tick(1000,84,0,0,12,0,0,0,.02)
        self.assertTrue(c.ramp_complete and c.startup_active)
        self.assertEqual(c.vel_sp_integral,2.)
        self.assertEqual(c.last_target_vel,0.)
        self.assertEqual(c.sp_offset,2.)

    def test_hold_capture_preserves_travel_reference(self):
        c=self.at_forward()
        for i in range(19):
            c.tick(1000+i*20,84,0,0,12,0,0,0,.02)
            self.assertTrue(c.startup_active)
        c.tick(1380,84,0,0,12,0,0,0,.02)
        self.assertFalse(c.startup_active)
        self.assertEqual(c.hold_drift,12.)
        self.assertEqual(c.wheel_start_pos,0.)
        self.assertEqual(c.last_target_vel,0.)
        c.tick(1400,84,0,0,13,0,0,0,.02)
        self.assertAlmostEqual(c.last_target_vel,-c.cfg.drift_vel_kp)

if __name__=='__main__':unittest.main()
