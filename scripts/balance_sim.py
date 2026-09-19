#!/usr/bin/env python3
"""Approximate planar simulator for the Hopscotch Speed-mode balance stack.

Plant (fitted from Speed-mode telemetry by fit_balance_model.py --speed-only):
    roll_accel = A * (roll - eq(arms)) + B * wheel_accel        [deg/s^2]
    wheel velocity tracks the speed command through a first-order lag tau_m
    with the motor-side acceleration limit (100 rad/s^2).

Control (ported line-for-line from src/balance_controller.cpp):
    - 200 Hz inner PD on Core 0 (complementary filter, two-stage dead-man)
    - 50 Hz outer tick on Core 1 (capture, arm return, base-sp ramp,
      dual-slope velocity PI, position P, arm assist lifecycle)

The simulated "Core 1" can be stalled for arbitrary windows: the outer tick
freezes (setpoint, arm targets, safety) while the inner PD keeps running
against the stale setpoint, with fresh idealized IMU input (the candidate reads IMU in the fast task).

Scenarios:
    validate        reproduce three logged failures (sim trustworthiness)
    standup         one standup with a chosen controller variant
    standup-matrix  variants x engage angles x model uncertainty
    push            disturbance response for a chosen variant
    stall-campaign  stall duration x timing sweep, dead-man verification

Usage:
    .venv/bin/python scripts/balance_sim.py validate
    .venv/bin/python scripts/balance_sim.py standup --variant carrot+curve+earlypos --plot
    .venv/bin/python scripts/balance_sim.py standup-matrix
    .venv/bin/python scripts/balance_sim.py stall-campaign --variant carrot+curve+earlypos
"""
from __future__ import annotations

import argparse
import json
import math
import random
import re
from dataclasses import dataclass, field, replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

INNER_DT = 1.0 / 200.0
OUTER_EVERY = 4          # outer tick every 4 inner ticks (50 Hz)


def clampf(v: float, lo: float, hi: float) -> float:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def move_toward(current: float, target: float, rate: float, dt: float) -> float:
    step = rate * dt
    diff = target - current
    if abs(diff) <= step:
        return target
    if diff > 0.0:
        return current + step
    return current - step


# ---------------------------------------------------------------------------
# Configuration (mirrors src/config.h -- keep in sync)
# ---------------------------------------------------------------------------

@dataclass
class FirmwareConfig:
    # Setpoint schedule
    sp_curve: tuple = ((0.00, 84.0), (0.50, 82.5), (1.00, 81.1))
    setpoint_arms_fwd: float = 84.0
    setpoint_arms_center: float = 77.7
    setpoint_min: float = 70.0
    setpoint_max: float = 110.0
    capture_shift_max: float = 15.0
    absolute_capture_trim: bool = False  # old records bounded the relative shift
    startup_recovery: bool = False  # historical variants leave the early catch off
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
    recoil_unwind: bool = False  # historical variants retain the original release
    recoil_enter_speed: float = .35
    recoil_exit_speed: float = .15
    recoil_confirm_ms: float = 60
    recoil_blend_ms: float = 120
    recoil_multiplier: float = 2
    base_sp_rate_max: float = 3.0
    ramp_vel_slow: float = 2.0
    ramp_vel_gate_floor: float = 0.3

    # Capture / arm return
    capture_err_max: float = 1.0
    capture_rate_max: float = 4.0
    capture_cmd_max: float = 1.0
    capture_settle_ms: int = 400
    arm_hold_max_ms: int = 1000
    arm_tip_left: float = 2.71
    arm_tip_right: float = 1.96
    arm_return_speed: float = 1.5
    arm_center_left: float = 1.768
    arm_center_right: float = -1.767

    # Inner PD
    kp: float = 2.0
    kd: float = 0.08
    comp_alpha: float = 0.996
    gyro_lpf_alpha: float = 0.08
    max_drive_speed: float = 30.0

    # Dead-man
    deadman_soft_ms: int = 300
    deadman_soft_cmd_max: float = 20.0
    deadman_hard_ms: int = 1500

    # Outer cascade
    drift_vel_kp: float = 0.05
    drift_max_vel: float = 1.0
    vel_sp_kp: float = 2.2
    vel_sp_kp_low: float = 0.7
    vel_sp_knee: float = 0.8
    vel_sp_ki: float = 0.35
    sp_offset_max: float = 8.0
    outer_handoff_ms: float = 0.0
    sp_offset_rate: float = 12.0
    vel_filter_alpha: float = 0.35
    pos_gate_err: float = 8.0
    glide_vel_err: float = 0.8
    glide_ki_boost: float = 4.0
    glide_rate_max: float = 10.0
    glide_cmd_max: float = 3.0
    shed_vel_start: float = 8.0
    shed_vel_full: float = 14.0

    # Arm assist
    arm_assist_thresh: float = 1.4
    arm_assist_gain: float = 0.40
    arm_assist_bias: float = 0.0
    arm_assist_range_pos: float = 0.45
    arm_assist_range_neg: float = 0.30
    arm_assist_vel_tau: float = 0.02
    arm_assist_tau_in: float = 0.08
    arm_assist_tau_out: float = 0.65
    arm_emergency_cmd_frac: float = 0.90

    arm_calm_vel: float = 1.0
    arm_calm_rate: float = 15.0
    arm_calm_ms: float = 400.0
    arm_event_end_calm_ms: float = -1.0
    measured_arm_arrival: bool = False
    hard_stop_latches: bool = False

    # Safety
    safe_tilt_min: float = 30.0
    safe_tilt_max: float = 150.0
    safe_err_max: float = 35.0
    safe_err_ms: int = 2000
    safe_rate_max: float = 200.0
    safe_rate_ms: int = 500
    safe_sat_ms: int = 3000
    bailout_deg: float = 45.0

    # Historical experiment switches; current_firmware_config reads today's source.
    # Carrot ramp: the base setpoint may lead the measured tilt by at most
    # carrot_lead_deg during the standup ramp -- the sp follows the robot
    # instead of towing it.
    carrot_ramp: bool = False
    carrot_lead_deg: float = 0.5
    # Carrot on the EFFECTIVE setpoint (base + sp_offset): in Speed mode the
    # glide velocity is Kp * (effective_sp - tilt), so capping the effective
    # lead directly caps the tow velocity at Kp * lead.
    carrot_effective: bool = False
    carrot_eff_lead_deg: float = 0.75
    # Disable the ramp-phase velocity damper (it forms a positive-feedback
    # loop with the glide: vel err raises sp_offset raises commanded vel)
    no_ramp_damper: bool = False
    # Bound the ramp-phase sp_offset instead of removing it: damping stays,
    # but the offset cannot add more than Kp*clamp rad/s to the tow.
    ramp_off_clamp: float = 0.0    # 0 = disabled (full sp_offset_max)
    # Tighter outer-loop gate during the standup ramp: velocity-based sp
    # corrections assume the robot is TRACKING (glide semantics, tilt ~ sp);
    # during a catch transient they push the wrong way. 0 = use pos_gate_err.
    ramp_gate_err: float = 0.0
    # Proportional arm return: scale per-arm speeds so both arms reach the
    # forward pose TOGETHER. Equal-rate return (current firmware) finishes
    # the right arm (1.96 rad) before the left (2.71 rad), which the axis
    # decomposition reads as a center excursion -- the equilibrium dips
    # ~0.9 deg mid-return for no reason.
    proportional_return: bool = False
    # Rejected September experiment; not enabled in firmware.
    arm_return_acceleration: float = 0.0
    # Equilibrium-tracking standup: replaces the time-based ramp entirely.
    # base_sp starts at the engage tilt and (1) follows the scheduled-curve
    # DELTA as the arms move (feedforward for the known equilibrium shift),
    # (2) walks toward the true equilibrium with a velocity-zeroing
    # integrator (base -= eq_track_k * filtered_vel * dt). It converges to
    # whatever equilibrium exists instead of dragging the robot to a
    # scheduled absolute level that may be wrong.
    eq_track: bool = False
    eq_track_k: float = 0.6        # deg per rad of wheel travel (integrator gain)
    eq_track_rate_max: float = 6.0  # deg/s cap on base_sp motion
    eq_track_done_vel: float = 0.6  # rad/s: "converged" gate for ramp_complete
    eq_track_done_ms: int = 400
    eq_track_timeout_ms: int = 10000
    # Position P live during the ramp (origin at engage), scheduled low
    early_pos_p: bool = False
    early_drift_kp: float = 0.03
    early_drift_max_vel: float = 0.6
    # Arm return waits for the robot: the return (which moves the true
    # equilibrium) only advances while the robot is tracking and not gliding
    arm_return_gate: bool = False
    arm_gate_err_deg: float = 1.0
    arm_gate_vel: float = 2.5
    arm_gate_timeout_ms: int = 8000   # failsafe: resume return regardless
    # Dead-man stall behavior: Core 0 continues the base-sp ramp toward the
    # last known raw target during a stall instead of freezing the setpoint
    stall_ramp_continue: bool = False


@dataclass
class PlantParams:
    A: float = 8.82        # 1/s^2 per deg of (roll - eq)
    B: float = 5.885       # deg per rad/s^2 of wheel accel
    damping: float = 0.0   # 1/s viscous tilt damping (bearing/tire losses)
    tau_m: float = 0.20    # motor velocity-loop lag, s (trajectory fit)
    acc_limit: float = 100.0
    # True equilibrium map (what the robot actually balances at):
    eq_fwd: float = 84.0            # true equilibrium, arms forward
    eq_tip_delta: float = -1.9      # eq(tip) - eq(fwd), measured -1.0..-3.0
    eq_center_slope: float = -6.3   # deg per center_frac
    # Noise (fit_balance_model.py --speed-only quiet windows)
    accel_noise_deg: float = 1.5    # raw accelerometer angle noise (200 Hz)
    gyro_noise_dps: float = 1.2
    feedback_velocity_scale: float = 1.0
    vel_meas_noise: float = 0.33    # wheel velocity feedback noise (rad/s)
    process_accel_noise: float = 0.5  # rad/s^2 random wheel-accel disturbance
    # Wheel feedback arrives via the round-robin CAN scan (6 motors at 50 Hz
    # -> each wheel refreshes every ~120 ms; L/R staggered -> ~60 ms average
    # refresh). The outer loop consumes a zero-order-held measurement.
    fb_hold_ticks: int = 3          # outer ticks between wheel feedback updates

    def true_eq(self, tip_frac: float, center_frac: float) -> float:
        # Monotonic rise tip -> forward (linear in tip_frac; the measured
        # SHAPE is close to linear over the return: run deltas +1.0..+3.0)
        return (self.eq_fwd
                + tip_frac * self.eq_tip_delta
                + center_frac * self.eq_center_slope)


def load_fitted_params(path: Path) -> PlantParams:
    p = PlantParams()
    if path.exists():
        fit = json.loads(path.read_text())
        p.A = float(fit.get("A", p.A))
        p.B = float(fit.get("B", p.B))
        p.tau_m = float(fit.get("tau_m", p.tau_m))
        noise = fit.get("noise", {})
        if noise:
            p.vel_meas_noise = float(noise.get("wheel_vel_std", p.vel_meas_noise))
            p.gyro_noise_dps = float(noise.get("rate_std_dps", p.gyro_noise_dps))
    return p


# ---------------------------------------------------------------------------
# Outer controller (Core 1) -- faithful port of BalanceController::update()
# ---------------------------------------------------------------------------

class OuterController:
    def __init__(self, cfg: FirmwareConfig, engage_tilt: float, engage_trim: float,
                 now_ms: float):
        self.cfg = cfg
        c = cfg

        # Arm state: positions relative to forward ref (deltas)
        self.arm_l_target = c.arm_tip_left
        self.arm_r_target = c.arm_tip_right
        self.arm_l_goal = c.arm_tip_left
        self.arm_r_goal = c.arm_tip_right
        self.arm_speed = 0.0

        scheduled = self.scheduled_setpoint(c.arm_tip_left, c.arm_tip_right)
        self.engage_arm_frac = self.tip_frac(c.arm_tip_left, c.arm_tip_right)
        self.engage_trim = clampf(engage_trim, -c.sp_offset_max, c.sp_offset_max)
        self.engage_capture_shift = clampf(engage_tilt - (scheduled + self.engage_trim),
                                           -c.capture_shift_max, c.capture_shift_max)
        initial_base = scheduled + self.engage_trim + self.engage_capture_shift
        self.smoothed_base_sp = initial_base
        self.effective_setpoint = clampf(initial_base, c.setpoint_min, c.setpoint_max)

        self.run_curve_shift = 0.0
        self.balance_start_ms = now_ms
        self.capture_stable = False
        self.capture_stable_start_ms = 0.0
        self.arms_returning = False
        self.return_elapsed = 0.0
        self.return_start_l = self.arm_l_target
        self.return_start_r = self.arm_r_target
        self.arms_returned = False
        self.ramp_complete = False
        self.ramp_complete_ms = 0.0
        self.capture_was_settled = False

        self.startup_triggered = False
        self.startup_active = False
        self.startup_ms = 0.
        self.recovery_calm_ms = 0.
        self.startup_accel = 0.
        self.startup_previous_vel = None
        self.startup_qualifying_ms = 0.
        self.startup_direction = 0.
        self.recoil_direction = 0.
        self.recoil_qualifying_ms = 0.
        self.recoil_blend = 0.
        self.recoil_confirmed = False
        self.hold_drift = 0.
        self.pilot = None  # optional production-C++ pilot adapter for drive screening
        self.vel_sp_integral = 0.0
        self.sp_offset = 0.0
        self.filtered_wheel_vel = 0.0
        self.last_target_vel = 0.0
        self.wheel_start_pos = 0.0   # engage origin (sim wheel pos starts at 0)
        self.last_raw_base_sp = initial_base
        self.prev_scheduled_sp = scheduled
        self.eq_done_ms = 0.0
        if cfg.eq_track:
            # base starts exactly at the engage tilt -- inherently continuous
            self.smoothed_base_sp = engage_tilt
            self.effective_setpoint = clampf(engage_tilt, cfg.setpoint_min,
                                             cfg.setpoint_max)

        self.arm_assist_frac = c.arm_assist_bias
        self.arm_assist_vel = 0.0
        self.arm_stage = 3   # COOLDOWN at engage
        self.arm_sign = 0.0
        self.arm_calm_ms = 0.0

        # Safety timers
        self.safe_err_timing = False
        self.safe_err_start = 0.0
        self.safe_rate_timing = False
        self.safe_rate_start = 0.0
        self.safe_sat_timing = False
        self.safe_sat_start = 0.0
        self.abort_reason: str | None = None

    # --- arm axis decomposition (armAxisFractions) ---
    def axis_fractions(self, d_l: float, d_r: float) -> tuple[float, float]:
        c = self.cfg
        T_l, T_r = c.arm_tip_left, c.arm_tip_right
        C_l, C_r = c.arm_center_left, c.arm_center_right
        det = T_l * C_r - T_r * C_l
        if abs(det) < 0.5:
            tip_avg = (T_l + T_r) * 0.5
            return clampf((d_l + d_r) * 0.5 / tip_avg, 0.0, 1.0), 0.0
        tip = clampf((d_l * C_r - d_r * C_l) / det, 0.0, 1.0)
        center = clampf((T_l * d_r - T_r * d_l) / det, -0.5, 1.2)
        return tip, center

    def tip_frac(self, d_l: float, d_r: float) -> float:
        return self.axis_fractions(d_l, d_r)[0]

    def scheduled_setpoint(self, d_l: float, d_r: float) -> float:
        c = self.cfg
        tip, center = self.axis_fractions(d_l, d_r)
        curve = c.sp_curve
        base = curve[-1][1]
        if tip <= curve[0][0]:
            base = curve[0][1]
        else:
            for i in range(1, len(curve)):
                lo_f, lo_v = curve[i - 1]
                hi_f, hi_v = curve[i]
                if tip <= hi_f:
                    span = hi_f - lo_f
                    t = (tip - lo_f) / span if span > 1e-4 else 0.0
                    base = lo_v + t * (hi_v - lo_v)
                    break
        base += center * (c.setpoint_arms_center - c.setpoint_arms_fwd)
        return base

    # --- one 50 Hz tick ---
    def tick(self, now_ms: float, tilt: float, rate: float, last_cmd: float,
             wheel_pos: float, wheel_vel_meas: float, arm_l_meas: float,
             arm_r_meas: float, dt: float) -> None:
        c = self.cfg
        meas_drift = wheel_pos - self.wheel_start_pos
        abs_cmd = abs(last_cmd)

        # --- safety (same thresholds; abort just flags the run) ---
        if tilt < c.safe_tilt_min or tilt > c.safe_tilt_max:
            self.abort_reason = "tilt out of range (fallen)"
            return
        eff_err = abs(self.effective_setpoint - tilt)
        if eff_err > c.safe_err_max:
            if not self.safe_err_timing:
                self.safe_err_timing = True
                self.safe_err_start = now_ms
            elif now_ms - self.safe_err_start > c.safe_err_ms:
                self.abort_reason = "sustained error"
                return
        else:
            self.safe_err_timing = False
        if abs(rate) > c.safe_rate_max:
            if not self.safe_rate_timing:
                self.safe_rate_timing = True
                self.safe_rate_start = now_ms
            elif now_ms - self.safe_rate_start > c.safe_rate_ms:
                self.abort_reason = "extreme rate"
                return
        else:
            self.safe_rate_timing = False
        if abs_cmd >= c.max_drive_speed * 0.95:
            if not self.safe_sat_timing:
                self.safe_sat_timing = True
                self.safe_sat_start = now_ms
            elif now_ms - self.safe_sat_start > c.safe_sat_ms:
                self.abort_reason = "saturation"
                return
        else:
            self.safe_sat_timing = False
        if abs(self.effective_setpoint - tilt) > c.bailout_deg:
            self.abort_reason = "bailout"
            return

        # --- scheduled base + capture shift ---
        scheduled_sp = self.scheduled_setpoint(arm_l_meas, arm_r_meas)
        capture_shift = 0.0
        arm_frac = self.tip_frac(arm_l_meas, arm_r_meas)
        capture_weight = 1.0
        if self.arms_returned:
            capture_weight = 0.0
        elif self.arms_returning:
            capture_weight = (clampf(arm_frac / self.engage_arm_frac, 0.0, 1.0)
                              if self.engage_arm_frac > 0.05 else 1.0)
        capture_shift = self.engage_capture_shift * capture_weight

        raw_base_sp = clampf(scheduled_sp + self.engage_trim + self.run_curve_shift
                             + capture_shift, c.setpoint_min, c.setpoint_max)
        self.last_raw_base_sp = raw_base_sp

        if c.eq_track and not self.ramp_complete:
            # Equilibrium-tracking standup: base_sp = engage tilt
            #   + curve-shape feedforward as the arms move
            #   + velocity-zeroing correction (glide means the sp is on the
            #     wrong side of the true equilibrium -- walk toward it).
            ff = scheduled_sp - self.prev_scheduled_sp
            corr = -c.eq_track_k * self.filtered_wheel_vel * dt
            step = clampf(ff + corr, -c.eq_track_rate_max * dt,
                          c.eq_track_rate_max * dt)
            self.smoothed_base_sp = clampf(self.smoothed_base_sp + step,
                                           c.setpoint_min, c.setpoint_max)
            if self.arms_returned and abs(self.filtered_wheel_vel) < c.eq_track_done_vel:
                self.eq_done_ms += dt * 1000.0
            else:
                self.eq_done_ms = 0.0
            timed_out = now_ms - self.balance_start_ms > c.eq_track_timeout_ms
            if (self.arms_returned and self.eq_done_ms >= c.eq_track_done_ms) \
                    or timed_out:
                self.ramp_complete = True
                self.ramp_complete_ms = now_ms
                self.vel_sp_integral = 0.0
            base_effective = self.smoothed_base_sp
            self.prev_scheduled_sp = scheduled_sp
        else:
            ramp_rate = c.base_sp_rate_max
            if not self.ramp_complete and not self.startup_triggered:
                vel_gate = 1.0 - clampf(abs(self.filtered_wheel_vel) / c.ramp_vel_slow,
                                        0.0, 1.0)
                vel_gate = max(vel_gate, c.ramp_vel_gate_floor)
                ramp_rate *= vel_gate
            proposed = move_toward(self.smoothed_base_sp, raw_base_sp, ramp_rate, dt)
            if c.carrot_ramp and not self.ramp_complete:
                # The setpoint may lead the robot by at most carrot_lead_deg:
                # it waits for the tilt to follow instead of towing the wheels.
                if proposed > self.smoothed_base_sp:      # ramping up
                    proposed = min(proposed, max(tilt + c.carrot_lead_deg,
                                                 self.smoothed_base_sp))
                elif proposed < self.smoothed_base_sp:    # ramping down
                    proposed = max(proposed, min(tilt - c.carrot_lead_deg,
                                                 self.smoothed_base_sp))
            self.smoothed_base_sp = proposed
            base_effective = self.smoothed_base_sp
            self.prev_scheduled_sp = scheduled_sp

            if (not self.ramp_complete and self.arms_returned
                    and abs(self.smoothed_base_sp - raw_base_sp) < 0.1):
                self.ramp_complete = True
                self.ramp_complete_ms = now_ms
                if not self.startup_triggered:
                    self.vel_sp_integral = 0.0

        # --- capture / arm return ---
        capture_ok = (abs(self.effective_setpoint - tilt) <= c.capture_err_max
                      and abs(rate) <= c.capture_rate_max
                      and abs_cmd <= c.capture_cmd_max)
        if capture_ok:
            if not self.capture_stable:
                self.capture_stable = True
                self.capture_stable_start_ms = now_ms
        else:
            self.capture_stable = False
            self.capture_stable_start_ms = 0.0
        capture_settled = (self.capture_stable
                           and now_ms - self.capture_stable_start_ms >= c.capture_settle_ms)
        hold_timed_out = now_ms - self.balance_start_ms >= c.arm_hold_max_ms

        if not self.arms_returning and (capture_settled or hold_timed_out):
            self.arms_returning = True
            self.return_start_l = self.arm_l_target
            self.return_start_r = self.arm_r_target
            self.arm_l_goal = 0.0
            self.arm_r_goal = 0.0
            if capture_settled:
                self.capture_was_settled = True
                scheduled_now = self.scheduled_setpoint(arm_l_meas, arm_r_meas)
                if c.absolute_capture_trim:
                    self.run_curve_shift = (clampf(tilt - scheduled_now,
                                                   -c.sp_offset_max, c.sp_offset_max)
                                            - self.engage_trim)
                else:
                    self.run_curve_shift = clampf(tilt - (scheduled_now + self.engage_trim),
                                                  -6.0, 6.0)
                self.engage_capture_shift = 0.0

        if self.arms_returning and not self.ramp_complete:
            crisis = abs(last_cmd) > 10.0 or abs(rate) > 30.0
            if c.arm_return_gate and not crisis:
                lagging = (abs(self.effective_setpoint - tilt) > c.arm_gate_err_deg
                           or abs(self.filtered_wheel_vel) > c.arm_gate_vel)
                timed_out = now_ms - self.balance_start_ms > c.arm_gate_timeout_ms
                if lagging and not timed_out:
                    crisis = True  # pause the return; robot hasn't caught up
            if not crisis:
                if c.arm_return_acceleration > 0:
                    self.return_elapsed += dt
                    longest = max(abs(self.return_start_l), abs(self.return_start_r), .0001)
                    a = c.arm_return_acceleration
                    ramp = min(c.arm_return_speed / a, math.sqrt(longest / a))
                    speed = a * ramp
                    cruise = max(0., longest / speed - ramp)
                    duration = 2*ramp + cruise
                    t = self.return_elapsed
                    if t >= duration: traveled = longest
                    elif t < ramp: traveled = .5*a*t*t
                    elif t < ramp+cruise: traveled = .5*a*ramp*ramp + speed*(t-ramp)
                    else: traveled = longest - .5*a*(duration-t)**2
                    self.arm_l_target = self.return_start_l * (1-traveled/longest)
                    self.arm_r_target = self.return_start_r * (1-traveled/longest)
                else:
                    speed_l = c.arm_return_speed
                    speed_r = c.arm_return_speed
                    if c.proportional_return:
                        dist_l = abs(self.arm_l_goal - self.arm_l_target)
                        dist_r = abs(self.arm_r_goal - self.arm_r_target)
                        longest = max(dist_l, dist_r)
                        if longest > 1e-3:
                            speed_l = c.arm_return_speed * dist_l / longest
                            speed_r = c.arm_return_speed * dist_r / longest
                    self.arm_l_target = move_toward(self.arm_l_target, self.arm_l_goal,
                                                    speed_l, dt)
                    self.arm_r_target = move_toward(self.arm_r_target, self.arm_r_goal,
                                                    speed_r, dt)
            elif c.arm_return_acceleration > 0:
                self.return_elapsed = 0.
                self.return_start_l, self.return_start_r = self.arm_l_target, self.arm_r_target
            if not self.arms_returned:
                if (abs(self.arm_l_target - self.arm_l_goal) < (.005 if c.arm_return_acceleration else .08)
                        and abs(self.arm_r_target - self.arm_r_goal) < (.005 if c.arm_return_acceleration else .08)
                        and (not c.measured_arm_arrival or
                             (abs(arm_l_meas - self.arm_l_goal) <= 0.15
                              and abs(arm_r_meas - self.arm_r_goal) <= 0.15))):
                    self.arms_returned = True

        # --- outer cascade ---
        gate_err = c.pos_gate_err
        if c.ramp_gate_err > 0.0 and not self.ramp_complete:
            gate_err = c.ramp_gate_err
        pos_gate = 1.0 - clampf(abs(self.effective_setpoint - tilt) / gate_err,
                                0.0, 1.0)
        self.filtered_wheel_vel += c.vel_filter_alpha * (wheel_vel_meas
                                                         - self.filtered_wheel_vel)
        # The plant model supplies fresh wheel samples; CAN loss and the
        # production 30 ms eligibility guard are covered separately in native tests.
        if c.startup_recovery and not self.startup_triggered:
            acceleration = (0. if self.startup_previous_vel is None else
                            (self.filtered_wheel_vel-self.startup_previous_vel)/dt)
            self.startup_previous_vel = self.filtered_wheel_vel
            self.startup_accel += clampf(dt/c.startup_accel_tau,0,1)*(acceleration-self.startup_accel)
            direction = 1. if self.filtered_wheel_vel>0 else -1.
            eligible = self.arms_returning and not self.ramp_complete and arm_frac<=c.startup_tip_max
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
        pilot_moving = False
        if self.pilot is not None:
            was_moving = self.pilot.moving
            tracking_error = abs(self.effective_setpoint-tilt)
            self.pilot.update(now_ms, self.ramp_complete and not self.startup_active,
                              self.filtered_wheel_vel, rate, tracking_error, dt)
            pilot_moving = self.pilot.moving
            if was_moving or pilot_moving:
                self.hold_drift = meas_drift
        target_vel = 0.0
        if self.startup_active:
            pass  # stop first, then hold the settled position
        elif pilot_moving:
            target_vel = self.pilot.velocity
        elif self.ramp_complete:
            target_vel = clampf(-c.drift_vel_kp * (meas_drift-self.hold_drift),
                                -c.drift_max_vel, c.drift_max_vel)
        elif c.early_pos_p:
            target_vel = clampf(-c.early_drift_kp * meas_drift,
                                -c.early_drift_max_vel, c.early_drift_max_vel)
        self.last_target_vel = target_vel
        vel_err = self.filtered_wheel_vel - target_vel

        # Mirror balance_math::RecoilUnwind; fresh feedback supplied by model.
        unwind_multiplier = 1.
        eligible = c.recoil_unwind and self.startup_active and self.ramp_complete
        direction = 1. if vel_err > 0 else -1.
        if (not eligible or vel_err*self.vel_sp_integral >= 0
                or not math.isfinite(dt) or dt <= 0 or dt > .1):
            self.recoil_direction = self.recoil_qualifying_ms = self.recoil_blend = 0.
            self.recoil_confirmed = False
        else:
            if direction != self.recoil_direction:
                self.recoil_qualifying_ms = self.recoil_blend = 0.
                self.recoil_confirmed = False
                self.recoil_direction = direction
            speed = abs(vel_err)
            if self.recoil_confirmed and speed <= c.recoil_exit_speed:
                self.recoil_confirmed = False
                self.recoil_qualifying_ms = 0.
            if not self.recoil_confirmed:
                self.recoil_qualifying_ms = (self.recoil_qualifying_ms+dt*1000
                                             if speed >= c.recoil_enter_speed else 0.)
                self.recoil_confirmed = self.recoil_qualifying_ms+.001 >= c.recoil_confirm_ms
            self.recoil_blend = clampf(self.recoil_blend + (1 if self.recoil_confirmed else -1)
                                       * dt*1000/c.recoil_blend_ms, 0, 1)
            unwind_multiplier = 1+self.recoil_blend*(c.recoil_multiplier-1)

        if self.startup_active:
            ki = (c.recovery_ki if not self.ramp_complete and now_ms-self.startup_ms<c.recovery_boost_ms
                  else c.vel_sp_ki)
            desired_rate = ki*vel_err
            if unwind_multiplier > 1 and desired_rate*self.vel_sp_integral < 0:
                ceiling = max(abs(self.vel_sp_integral)/dt,abs(desired_rate))
                desired_rate = math.copysign(min(abs(desired_rate)*unwind_multiplier,ceiling),desired_rate)
            change = clampf(desired_rate,-c.recovery_rate,c.recovery_rate)*dt
            if abs(self.sp_offset)<c.recovery_limit-.05 or change*self.sp_offset<0:
                self.vel_sp_integral = clampf(self.vel_sp_integral+change,-c.recovery_limit,c.recovery_limit)
        elif self.ramp_complete:
            ki = c.vel_sp_ki
            calm = abs(rate) < c.glide_rate_max and abs(last_cmd) < c.glide_cmd_max
            if not pilot_moving and calm and abs(vel_err) > c.glide_vel_err:
                ki *= c.glide_ki_boost
            self.vel_sp_integral += ki * vel_err * pos_gate * dt
            self.vel_sp_integral = clampf(self.vel_sp_integral,
                                          -c.sp_offset_max, c.sp_offset_max)

        authority = (clampf((now_ms - self.ramp_complete_ms) / c.outer_handoff_ms, 0.0, 1.0)
                     if self.ramp_complete and c.outer_handoff_ms > 0 else float(self.ramp_complete))
        high_kp = c.vel_sp_kp_low + authority * (c.vel_sp_kp - c.vel_sp_kp_low)
        abs_err = abs(vel_err)
        if (c.no_ramp_damper or c.eq_track) and not self.ramp_complete:
            # eq_track: the base tracker IS the standup velocity response;
            # a parallel damper would double-count (lesson 43).
            p_term = 0.0
        elif abs_err <= c.vel_sp_knee or not self.ramp_complete:
            p_term = c.vel_sp_kp_low * vel_err
        else:
            sign = 1.0 if vel_err >= 0.0 else -1.0
            p_term = sign * (c.vel_sp_kp_low * c.vel_sp_knee
                             + high_kp * (abs_err - c.vel_sp_knee))

        arm_dev = abs(self.arm_assist_frac - c.arm_assist_bias)
        arm_share = clampf(arm_dev / c.arm_assist_range_pos, 0.0, 1.0)
        p_term *= (1.0 - 0.6 * arm_share)

        shed = 1.0 - clampf((abs(self.filtered_wheel_vel) - c.shed_vel_start)
                            / (c.shed_vel_full - c.shed_vel_start), 0.0, 1.0)

        off_max = c.sp_offset_max
        if c.ramp_off_clamp > 0.0 and not self.ramp_complete:
            off_max = c.ramp_off_clamp
        if self.ramp_complete and c.outer_handoff_ms > 0 and c.ramp_off_clamp > 0:
            off_max = c.ramp_off_clamp + authority * (c.sp_offset_max - c.ramp_off_clamp)
        if self.startup_active:
            off_max = c.recovery_limit
        sp_offset_target = clampf(p_term * pos_gate * shed + self.vel_sp_integral,
                                  -off_max, off_max)
        self.sp_offset = move_toward(self.sp_offset, sp_offset_target,
                                     c.sp_offset_rate, dt)

        # --- arm assist lifecycle ---
        if self.ramp_complete:
            self.arm_assist_vel += clampf(dt / c.arm_assist_vel_tau, 0.0, 1.0) \
                * (vel_err - self.arm_assist_vel)
            excess = 0.0
            if self.arm_assist_vel > c.arm_assist_thresh:
                excess = self.arm_assist_vel - c.arm_assist_thresh
            elif self.arm_assist_vel < -c.arm_assist_thresh:
                excess = self.arm_assist_vel + c.arm_assist_thresh
            demand = clampf(c.arm_assist_gain * excess,
                            -c.arm_assist_range_neg, c.arm_assist_range_pos)

            calm_now = abs(vel_err) < c.arm_calm_vel and abs(rate) < c.arm_calm_rate
            if calm_now:
                self.arm_calm_ms += dt * 1000.0
            else:
                self.arm_calm_ms = 0.0

            dev = self.arm_assist_frac - c.arm_assist_bias
            near_neutral = abs(dev) < 0.10
            dsign = 1.0 if demand > 0.0 else (-1.0 if demand < 0.0 else 0.0)

            target = c.arm_assist_bias
            tau = c.arm_assist_tau_out

            if self.arm_stage == 0:
                if dsign != 0.0:
                    self.arm_sign = dsign
                    self.arm_stage = 1
                    target = c.arm_assist_bias + demand
                    tau = c.arm_assist_tau_in
            elif self.arm_stage in (1, 2):
                if dsign == self.arm_sign and abs(demand) > abs(dev):
                    target = c.arm_assist_bias + demand
                    tau = c.arm_assist_tau_in
                elif (dsign == -self.arm_sign and abs(demand) > 0.10
                      and self.arm_stage == 1):
                    if near_neutral:
                        self.arm_sign = dsign
                        self.arm_stage = 2
                        target = c.arm_assist_bias + demand
                        tau = c.arm_assist_tau_in
                    else:
                        tau = c.arm_assist_tau_in * 2.0
                if near_neutral and abs(demand) < 0.05 and self.arm_calm_ms > c.arm_event_end_calm_ms:
                    self.arm_stage = 3
                    self.arm_calm_ms = 0.0
            else:
                if self.arm_calm_ms > c.arm_calm_ms:
                    self.arm_stage = 0

            wheels_railed = abs(last_cmd) >= c.max_drive_speed * c.arm_emergency_cmd_frac
            if wheels_railed and abs(vel_err) > c.arm_assist_thresh:
                self.arm_sign = 1.0 if vel_err > 0.0 else -1.0
                self.arm_stage = 1
                if vel_err > 0.0:
                    target = c.arm_assist_bias + c.arm_assist_range_pos
                else:
                    target = c.arm_assist_bias - c.arm_assist_range_neg
                tau = c.arm_assist_tau_in

            alpha = clampf(dt / tau, 0.0, 1.0)
            self.arm_assist_frac += alpha * (target - self.arm_assist_frac)
            self.arm_l_target = self.arm_assist_frac * c.arm_center_left
            self.arm_r_target = self.arm_assist_frac * c.arm_center_right

        effective = base_effective + self.sp_offset
        if c.carrot_effective and not self.ramp_complete:
            effective = clampf(effective, tilt - c.carrot_eff_lead_deg,
                               tilt + c.carrot_eff_lead_deg)
        self.effective_setpoint = clampf(effective,
                                         c.setpoint_min, c.setpoint_max)


# ---------------------------------------------------------------------------
# Full simulation (plant + inner loop + outer loop + stalls)
# ---------------------------------------------------------------------------

@dataclass
class StallWindow:
    start_s: float
    duration_s: float


@dataclass
class Push:
    at_s: float
    delta_vel: float = 0.0    # rad/s added to wheel velocity (floor shove)
    delta_rate: float = 0.0   # deg/s added to roll rate (body shove)


@dataclass
class SimResult:
    t: list = field(default_factory=list)
    roll: list = field(default_factory=list)
    setpoint: list = field(default_factory=list)
    cmd: list = field(default_factory=list)
    wheel_vel: list = field(default_factory=list)
    drift: list = field(default_factory=list)
    sp_offset: list = field(default_factory=list)
    eq: list = field(default_factory=list)
    target_vel: list = field(default_factory=list)
    fell: bool = False
    abort_reason: str | None = None
    ramp_complete_s: float | None = None
    # metrics
    drift_at_ramp: float = 0.0
    peak_drift_standup: float = 0.0
    peak_vel_standup: float = 0.0
    final_drift: float = 0.0

    def summary(self) -> str:
        status = "FELL(" + (self.abort_reason or "?") + ")" if self.fell else "ok"
        ramp = f"{self.ramp_complete_s:.1f}s" if self.ramp_complete_s else "never"
        return (f"{status:24} ramp={ramp:>6} "
                f"drift@ramp={self.drift_at_ramp:+6.2f} "
                f"peak_drift={self.peak_drift_standup:+6.2f} "
                f"peak_vel={self.peak_vel_standup:5.2f} "
                f"final_drift={self.final_drift:+6.2f}")


def simulate(cfg: FirmwareConfig, plant: PlantParams,
             engage_offset_deg: float = 0.5,
             duration_s: float = 25.0,
             stalls: list[StallWindow] | None = None,
             pushes: list[Push] | None = None,
             seed: int = 1,
             engage_trim: float = 0.0,
             deadman_v1: bool = False, pilot_factory=None) -> SimResult:
    """Run one standup + balance episode.

    deadman_v1: replicate the run-13-era dead-man (wheel STOP at soft
    threshold) for validation.
    """
    rng = random.Random(seed)
    stalls = stalls or []
    pushes = pushes or []
    c = cfg

    # --- state ---
    # Arms start at tip pose; true equilibrium at tip:
    tip0, cen0 = 1.0, 0.0
    eq0 = plant.true_eq(tip0, cen0)
    theta = eq0 + engage_offset_deg   # engage close to but off equilibrium
    theta_dot = 0.0
    wheel_pos = 0.0
    wheel_vel = 0.0

    # Actual arm positions (deltas from forward ref); track targets fast
    arm_l = c.arm_tip_left
    arm_r = c.arm_tip_right
    ARM_ACT_SPEED = 6.0   # rad/s actuator slew toward Core-1 target

    outer = OuterController(cfg, engage_tilt=theta, engage_trim=engage_trim,
                            now_ms=0.0)
    if pilot_factory is not None:
        outer.pilot = pilot_factory()

    # Inner-loop filter state
    tilt_est = theta
    gyro_filt = 0.0
    last_cmd = 0.0
    last_outer_ms = 0.0
    core0_base_sp = outer.smoothed_base_sp
    held_vel = 0.0
    held_pos = 0.0

    res = SimResult()
    n_steps = int(duration_s / INNER_DT)

    def in_stall(t_s: float) -> bool:
        return any(s.start_s <= t_s < s.start_s + s.duration_s for s in stalls)

    hard_stop_latched = False
    for step in range(n_steps):
        t = step * INNER_DT
        now_ms = t * 1000.0

        # --- pushes ---
        for p in pushes:
            if abs(p.at_s - t) < INNER_DT / 2:
                wheel_vel += p.delta_vel
                theta_dot += p.delta_rate

        # --- 200 Hz inner tick (Core 0) ---
        accel_angle = theta + rng.gauss(0.0, plant.accel_noise_deg)
        gyro_raw = theta_dot + rng.gauss(0.0, plant.gyro_noise_dps)
        tilt_est = (c.comp_alpha * (tilt_est + gyro_raw * INNER_DT)
                    + (1.0 - c.comp_alpha) * accel_angle)
        gyro_filt = c.gyro_lpf_alpha * gyro_raw + (1.0 - c.gyro_lpf_alpha) * gyro_filt

        update_age = now_ms - last_outer_ms
        if c.stall_ramp_continue and update_age > c.deadman_soft_ms:
            # Candidate fix: during a stall Core 0 keeps walking its shadow
            # copy of the base setpoint toward the last raw target at 30% of
            # the ramp rate (the vel-gate floor), so the setpoint is not left
            # parked below equilibrium (runs 12/13 stale-setpoint runaway).
            core0_base_sp = move_toward(core0_base_sp, outer.last_raw_base_sp,
                                        c.base_sp_rate_max * c.ramp_vel_gate_floor,
                                        INNER_DT)
            setpoint = clampf(core0_base_sp + outer.sp_offset,
                              c.setpoint_min, c.setpoint_max)
        else:
            core0_base_sp = outer.smoothed_base_sp
            setpoint = outer.effective_setpoint

        cmd_max = c.max_drive_speed
        stopped = False
        if deadman_v1:
            if update_age > c.deadman_soft_ms:
                stopped = True
        else:
            if update_age > c.deadman_hard_ms:
                stopped = True
            elif update_age > c.deadman_soft_ms:
                cmd_max = c.deadman_soft_cmd_max

        if stopped and c.hard_stop_latches:
            hard_stop_latched = True
        stopped = stopped or hard_stop_latched
        if stopped:
            cmd = 0.0
            # wheels commanded to 0 speed; the motor tracks it
        else:
            angle_err = setpoint - tilt_est
            cmd = c.kp * angle_err - c.kd * gyro_filt
            if outer.pilot is not None and outer.pilot.moving:
                cmd += outer.pilot.velocity
            cmd = clampf(cmd, -cmd_max, cmd_max)
        last_cmd = cmd

        # --- motor + plant integration ---
        dv = (INNER_DT / (plant.tau_m + INNER_DT)) * (cmd - wheel_vel)
        max_dv = plant.acc_limit * INNER_DT
        dv = clampf(dv, -max_dv, max_dv)
        dv += rng.gauss(0.0, plant.process_accel_noise) * INNER_DT
        wheel_accel = dv / INNER_DT
        wheel_vel += dv
        wheel_pos += wheel_vel * INNER_DT

        tip_true, cen_true = outer.axis_fractions(arm_l, arm_r)
        eq_now = plant.true_eq(tip_true, cen_true)
        theta_ddot = (plant.A * (theta - eq_now) + plant.B * wheel_accel
                      - plant.damping * theta_dot)
        theta_dot += theta_ddot * INNER_DT
        theta += theta_dot * INNER_DT

        # Arms track Core-1 targets (position servos, fast)
        arm_l = move_toward(arm_l, outer.arm_l_target, ARM_ACT_SPEED, INNER_DT)
        arm_r = move_toward(arm_r, outer.arm_r_target, ARM_ACT_SPEED, INNER_DT)

        # --- 50 Hz outer tick (Core 1), unless stalled ---
        if step % OUTER_EVERY == 0 and not in_stall(t):
            dt = 0.02 if step == 0 else (now_ms - last_outer_ms) / 1000.0
            outer_tick_i = step // OUTER_EVERY
            if outer_tick_i % max(plant.fb_hold_ticks, 1) == 0:
                held_vel = plant.feedback_velocity_scale * wheel_vel + rng.gauss(0.0, plant.vel_meas_noise)
                held_pos = wheel_pos
            outer.tick(now_ms, tilt_est, gyro_filt, last_cmd, held_pos,
                       held_vel, arm_l, arm_r, 0.02 if c.hard_stop_latches else min(dt, 0.1))
            last_outer_ms = now_ms
            if outer.ramp_complete and res.ramp_complete_s is None:
                res.ramp_complete_s = t
                res.drift_at_ramp = wheel_pos
            if outer.abort_reason:
                res.fell = True
                res.abort_reason = outer.abort_reason
                break

        # Hard fall regardless of Core 1 (e.g. mid-stall)
        if theta < 45.0 or theta > 135.0:
            res.fell = True
            res.abort_reason = res.abort_reason or "fell (tilt runaway)"
            break

        # --- log at 50 Hz ---
        if step % OUTER_EVERY == 0:
            res.t.append(t)
            res.roll.append(theta)
            res.setpoint.append(outer.effective_setpoint)
            res.cmd.append(last_cmd)
            res.wheel_vel.append(wheel_vel)
            res.drift.append(wheel_pos)
            res.sp_offset.append(outer.sp_offset)
            res.eq.append(eq_now)
            res.target_vel.append(outer.last_target_vel)
            in_standup = res.ramp_complete_s is None or t < res.ramp_complete_s + 2.0
            if in_standup:
                if abs(wheel_pos) > abs(res.peak_drift_standup):
                    res.peak_drift_standup = wheel_pos
                res.peak_vel_standup = max(res.peak_vel_standup, abs(wheel_vel))

    res.final_drift = wheel_pos
    if res.ramp_complete_s is None and not res.fell:
        res.drift_at_ramp = wheel_pos
    return res


# ---------------------------------------------------------------------------
# Controller variants
# ---------------------------------------------------------------------------

# Refit curve shape (Speed-mode data): total tip->fwd drop ~1.9 deg,
# close to linear over the return (per-run deltas +1.0..+3.0 deg).
CURVE_REFIT = ((0.00, 84.0), (0.50, 83.05), (1.00, 82.1))


def current_firmware_config() -> FirmwareConfig:
    """Read controller constants from the source used to build this candidate.

    Historical variants remain frozen for comparisons. This is an approximate
    planar plant: no tire slip, mounting flex, power faults or CAN arbitration.
    It cannot certify hardware reliability or self-righting contact mechanics.
    """
    source = (REPO_ROOT / 'src/config.h').read_text()
    def number(name):
        # Read arithmetic constant expressions without evaluating arbitrary code.
        import ast
        match = re.search(r'\b' + name + r'\s*=\s*([^;]+);', source)
        if not match: raise ValueError(f'Cannot read firmware constant {name}')
        expression = re.sub(r'(?<=\d)f\b', '', match[1])
        def value(node):
            if isinstance(node, ast.Constant) and isinstance(node.value, (int,float)): return float(node.value)
            if isinstance(node, ast.Name): return number(node.id)
            if isinstance(node, ast.BinOp):
                left,right=value(node.left),value(node.right)
                if isinstance(node.op, ast.Mult): return left*right
                if isinstance(node.op, ast.Div): return left/right
                if isinstance(node.op, ast.Add): return left+right
                if isinstance(node.op, ast.Sub): return left-right
            if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub): return -value(node.operand)
            raise ValueError(f'Unsupported constant expression: {name}')
        return value(ast.parse(expression.strip(),mode='eval').body)
    mapping = {
        'kp': 'BALANCE_KP', 'kd': 'BALANCE_KD', 'comp_alpha': 'COMPLEMENTARY_ALPHA',
        'drift_vel_kp': 'BALANCE_DRIFT_VEL_KP', 'drift_max_vel': 'BALANCE_DRIFT_MAX_VEL',
        'vel_sp_kp': 'BALANCE_VEL_SP_KP', 'vel_sp_kp_low': 'BALANCE_VEL_SP_KP_LOW',
        'vel_sp_ki': 'BALANCE_VEL_SP_KI', 'vel_sp_knee': 'BALANCE_VEL_SP_KNEE',
        'sp_offset_max': 'BALANCE_SP_OFFSET_MAX_DEG', 'sp_offset_rate': 'BALANCE_SP_OFFSET_RATE',
        'vel_filter_alpha': 'BALANCE_VEL_FILTER_ALPHA', 'pos_gate_err': 'BALANCE_POS_GATE_ERR_DEG',
        'ramp_off_clamp': 'BALANCE_RAMP_SP_OFFSET_MAX_DEG', 'base_sp_rate_max': 'BALANCE_BASE_SP_RATE_MAX',
        'ramp_vel_slow': 'BALANCE_RAMP_VEL_SLOW', 'arm_return_speed': 'BALANCE_ARM_RETURN_SPEED',
        'arm_emergency_cmd_frac': 'BALANCE_ARM_EMERGENCY_CMD_FRAC',
        'arm_calm_vel': 'BALANCE_ARM_CALM_VEL', 'arm_calm_rate': 'BALANCE_ARM_CALM_RATE',
        'arm_calm_ms': 'BALANCE_ARM_CALM_MS', 'arm_assist_thresh': 'BALANCE_ARM_ASSIST_THRESH',
        'arm_assist_gain': 'BALANCE_ARM_ASSIST_GAIN', 'arm_assist_bias': 'BALANCE_ARM_ASSIST_BIAS_FRAC',
        'arm_assist_range_pos': 'BALANCE_ARM_ASSIST_RANGE_POS', 'arm_assist_range_neg': 'BALANCE_ARM_ASSIST_RANGE_NEG',
        'arm_assist_vel_tau': 'BALANCE_ARM_ASSIST_VEL_TAU', 'arm_assist_tau_in': 'BALANCE_ARM_ASSIST_TAU_IN',
        'arm_assist_tau_out': 'BALANCE_ARM_ASSIST_TAU_OUT', 'max_drive_speed': 'BALANCE_MAX_DRIVE_SPEED',
        'setpoint_min': 'BALANCE_SETPOINT_MIN', 'setpoint_max': 'BALANCE_SETPOINT_MAX',
        'setpoint_arms_fwd': 'BALANCE_SETPOINT_ARMS_FWD', 'setpoint_arms_center': 'BALANCE_SETPOINT_ARMS_CENTER',
        'capture_shift_max': 'BALANCE_CAPTURE_SHIFT_MAX_DEG',
        'capture_err_max': 'BALANCE_CAPTURE_ERR_MAX_DEG', 'capture_rate_max': 'BALANCE_CAPTURE_RATE_MAX_DPS',
        'capture_cmd_max': 'BALANCE_CAPTURE_CMD_MAX', 'capture_settle_ms': 'BALANCE_CAPTURE_SETTLE_MS',
        'arm_hold_max_ms': 'BALANCE_ARM_HOLD_MAX_MS', 'arm_tip_left': 'BALANCE_ARM_TIP_LEFT',
        'arm_tip_right': 'BALANCE_ARM_TIP_RIGHT', 'early_drift_kp': 'BALANCE_RAMP_DRIFT_KP',
        'early_drift_max_vel': 'BALANCE_RAMP_DRIFT_MAX_VEL',
        'deadman_soft_ms': 'BALANCE_DEADMAN_SOFT_MS', 'deadman_soft_cmd_max': 'BALANCE_DEADMAN_SOFT_CMD_MAX',
        'deadman_hard_ms': 'BALANCE_DEADMAN_HARD_MS', 'shed_vel_start': 'BALANCE_SHED_VEL_START',
        'shed_vel_full': 'BALANCE_SHED_VEL_FULL', 'glide_vel_err': 'BALANCE_GLIDE_VEL_ERR',
        'glide_ki_boost': 'BALANCE_GLIDE_KI_BOOST', 'glide_rate_max': 'BALANCE_GLIDE_RATE_MAX_DPS',
        'glide_cmd_max': 'BALANCE_GLIDE_CMD_MAX',
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
        'recovery_calm_ms': 'BALANCE_START_RECOVERY_CALM_MS',
        'recoil_enter_speed': 'BALANCE_RECOIL_ENTER_SPEED',
        'recoil_exit_speed': 'BALANCE_RECOIL_EXIT_SPEED',
        'recoil_confirm_ms': 'BALANCE_RECOIL_CONFIRM_MS',
        'recoil_blend_ms': 'BALANCE_RECOIL_BLEND_MS',
        'recoil_multiplier': 'BALANCE_RECOIL_MULTIPLIER',
    }
    kwargs = {field: number(const) for field, const in mapping.items()}
    curve = source.split('BALANCE_SP_CURVE[] = {', 1)[1].split('};', 1)[0]
    kwargs['sp_curve'] = tuple((float(a), float(b)) for a, b in
                               re.findall(r'\{\s*([0-9.]+)f,\s*([0-9.]+)f\s*\}', curve))
    return replace(FirmwareConfig(), **kwargs, proportional_return=True, early_pos_p=True,
                   absolute_capture_trim=True, startup_recovery=True, recoil_unwind=True,
                   arm_event_end_calm_ms=150.0, measured_arm_arrival=True, hard_stop_latches=True)


def make_variant(name: str, base: FirmwareConfig | None = None) -> FirmwareConfig:
    if name == "current":
        return current_firmware_config()
    if name == "july33":
        return replace(current_firmware_config(), ramp_off_clamp=0.0, drift_vel_kp=0.05,
                       measured_arm_arrival=False, hard_stop_latches=False, absolute_capture_trim=False,
                       startup_recovery=False, recoil_unwind=False)
    cfg = base if base is not None else FirmwareConfig()
    for part in name.split("+"):
        if part == "baseline":
            pass
        elif part == "curve":
            cfg = replace(cfg, sp_curve=CURVE_REFIT)
        elif part == "carrot":
            cfg = replace(cfg, carrot_ramp=True)
        elif part == "earlypos":
            cfg = replace(cfg, early_pos_p=True)
        elif part == "armgate":
            cfg = replace(cfg, arm_return_gate=True)
        elif part == "effcarrot":
            cfg = replace(cfg, carrot_effective=True)
        elif part == "nodamper":
            cfg = replace(cfg, no_ramp_damper=True)
        elif part == "offclamp":
            cfg = replace(cfg, ramp_off_clamp=1.5)
        elif part == "slowreturn":
            cfg = replace(cfg, arm_return_speed=0.8)
        elif part == "eqtrack":
            cfg = replace(cfg, eq_track=True)
        elif part == "rampgate":
            cfg = replace(cfg, ramp_gate_err=2.5)
        elif part == "propreturn":
            cfg = replace(cfg, proportional_return=True)
        elif part == "fixed":
            # The Phase 2 winner: refit curve shape + proportional arm
            # return + position P live from engage. Velocity-based gating of
            # the arm return (armgate/rampgate) was rejected: it cut the tow
            # further but tripled standup failures -- the return RAISES the
            # equilibrium toward the robot, so pausing it during a glide
            # often prevents the self-correction (sim matrix, Jul 2026).
            cfg = replace(cfg,
                          sp_curve=CURVE_REFIT,
                          proportional_return=True,
                          early_pos_p=True)
        elif part == "rampcont":
            cfg = replace(cfg, stall_ramp_continue=True)
        else:
            raise SystemExit(f"unknown variant part: {part}")
    return cfg


VARIANTS = [
    "july33",
    "current",
    "baseline",
    "curve",
    "earlypos",
    "propreturn",
    "rampgate",
    "armgate",
    "propreturn+curve",
    "propreturn+curve+armgate",
    "fixed",
]


def uncertainty_models(plant: PlantParams) -> list[tuple[str, PlantParams]]:
    out = []
    for fa in (0.7, 1.0, 1.4):
        for fb in (0.8, 1.0, 1.25):
            out.append((f"A{fa}B{fb}", replace(plant, A=plant.A * fa, B=plant.B * fb)))
    return out


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def scenario_validate(plant: PlantParams) -> None:
    print("=" * 78)
    print("VALIDATION 1: standup tow with current firmware (logs: +6..+16 rad)")
    print("=" * 78)
    cfg = FirmwareConfig()
    for seed in (1, 2, 3):
        r = simulate(cfg, plant, engage_offset_deg=0.5, duration_s=20.0, seed=seed)
        print(f"  seed={seed}: {r.summary()}")
    print("  EXPECT: drift@ramp / peak_drift of order +5..+16 rad, no fall\n")

    print("=" * 78)
    print("VALIDATION 2: run-23 mechanism -- stall during the standup window")
    print("=" * 78)
    # Run 23: a disturbance arrived as a 430+ ms Core 1 stall hit during the
    # arm return; the robot fell where it otherwise would have caught itself.
    # Reproduce the mechanism: a -40 dps body shove that is ALWAYS caught
    # without a stall becomes a majority-fall with a 600 ms stall.
    # (The clamp 8-vs-20 difference from run 23 does not separate on this
    # fitted plant: sim falls here are driven by the frozen setpoint and
    # paused arm return, not the clamp value.)
    for label, stalls in (("no stall   ", []),
                          ("stall 0.6s ", [StallWindow(start_s=1.9, duration_s=0.6)])):
        falls = 0
        for seed in (1, 2, 3, 4, 5):
            r = simulate(FirmwareConfig(), plant, engage_offset_deg=1.5,
                         duration_s=15.0, stalls=stalls,
                         pushes=[Push(at_s=2.0, delta_rate=-40.0)], seed=seed)
            if r.fell:
                falls += 1
        print(f"  {label}: falls={falls}/5")
    print("  EXPECT: 0/5 without the stall, majority falls with it\n")

    print("=" * 78)
    print("VALIDATION 3: run-13-style 740 ms stall in steady balance")
    print("=" * 78)
    # Run 13 was standing QUIETLY when the stall hit. The fitted-A plant
    # limit-cycles in quiet stance harder than the real robot (documented
    # sim artifact), so this scenario uses the A x0.7 uncertainty corner
    # where the quiet stance amplitude is realistic.
    quiet_plant = replace(plant, A=plant.A * 0.7)
    stall = [StallWindow(start_s=12.0, duration_s=0.74)]
    for seed in (1, 2, 3):
        r = simulate(FirmwareConfig(), quiet_plant, duration_s=20.0, stalls=stall,
                     seed=seed, deadman_v1=True)
        print(f"  v1 (stop@300ms) : seed={seed}: {r.summary()}")
    for seed in (1, 2, 3):
        r = simulate(FirmwareConfig(), quiet_plant, duration_s=20.0, stalls=stall,
                     seed=seed)
        print(f"  v2 (two-stage)  : seed={seed}: {r.summary()}")
    print("  EXPECT: v1 falls (wheel stop = unpowered pole), v2 survives")


def scenario_standup(plant: PlantParams, variant: str, plot: bool,
                     seed: int = 1) -> None:
    cfg = make_variant(variant)
    r = simulate(cfg, plant, duration_s=25.0, seed=seed)
    print(f"standup [{variant}]: {r.summary()}")
    if plot:
        plot_run(r, f"standup_{variant.replace('+', '_')}")


def standup_cases(plant: PlantParams):
    """Realistic standup stress set.

    On the robot the capture-settle gate delivers the robot AT the tip
    equilibrium before the ramp starts, so the realistic uncertainties are:
    - how far the equilibrium moves during the arm return (measured
      +1.0..+3.3 deg across the July runs), which the fixed curve cannot
      know in advance, and
    - the plant stiffness A (closed-loop fit is biased),
    - a small residual capture offset (the settle gate is 1 deg wide).
    """
    for fa in (0.7, 1.0, 1.4):
        for eq_delta in (-1.0, -1.9, -3.3):
            p = replace(plant, A=plant.A * fa, eq_tip_delta=eq_delta)
            for off in (-0.8, 0.0, 0.8):
                for seed in (1, 2):
                    yield p, off, seed


def scenario_standup_matrix(plant: PlantParams) -> None:
    """Rank variants on STANDUP outcomes.

    standup_fail = fell before the ramp completed or within 3 s after it
    (the maneuver itself failed). Falls later than that are counted
    separately: the fitted-A plant limit-cycles in quiet stance harder than
    the real robot (documented sim artifact) and late falls are dominated
    by it, equally for all variants.
    """
    print(f"{'variant':30} {'standup':>8} {'late':>5} {'drift@ramp':>11} "
          f"{'peak_drift':>11} {'peak_vel':>9} {'ramp_s':>7}")
    for variant in VARIANTS:
        cfg = make_variant(variant)
        standup_fails = 0
        late_falls = 0
        n = 0
        dr, pk, pv, rs = [], [], [], []
        for p, off, seed in standup_cases(plant):
            r = simulate(cfg, p, engage_offset_deg=off, duration_s=16.0,
                         seed=seed)
            n += 1
            if r.fell:
                fall_t = r.t[-1] if r.t else 0.0
                if r.ramp_complete_s is None or fall_t < r.ramp_complete_s + 3.0:
                    standup_fails += 1
                    continue
                late_falls += 1
            dr.append(abs(r.drift_at_ramp))
            pk.append(abs(r.peak_drift_standup))
            pv.append(r.peak_vel_standup)
            if r.ramp_complete_s:
                rs.append(r.ramp_complete_s)

        def med(x):
            if not x:
                return float("nan")
            s = sorted(x)
            return s[len(s) // 2]
        print(f"{variant:30} {standup_fails:>4}/{n:<3} {late_falls:>5} "
              f"{med(dr):11.2f} {med(pk):11.2f} {med(pv):9.2f} {med(rs):7.1f}")


def scenario_push(plant: PlantParams, variant: str, plot: bool) -> None:
    cfg = make_variant(variant)
    for mag in (2.0, 4.0, 6.0):
        r = simulate(cfg, plant, duration_s=30.0,
                     pushes=[Push(at_s=15.0, delta_vel=mag)], seed=1)
        print(f"push {mag:+.1f} rad/s [{variant}]: {r.summary()}")
        if plot:
            plot_run(r, f"push{mag:.0f}_{variant.replace('+', '_')}")


def scenario_stall_campaign(plant: PlantParams, variant: str) -> None:
    cfg = make_variant(variant)
    durations = (0.1, 0.3, 0.5, 0.8, 1.2, 2.0, 3.0, 5.0)
    timings = {
        "capture (0.5s)": 0.5,
        "arm return (1.9s)": 1.9,
        "ramp (3.5s)": 3.5,
        "steady (12s)": 12.0,
    }
    print(f"variant: {variant}")
    print(f"{'timing':20}" + "".join(f"{d:>7.1f}s" for d in durations))
    for label, start in timings.items():
        cells = []
        for d in durations:
            falls = 0
            n = 0
            for mname, p in [("nom", plant)] + uncertainty_models(plant)[:3]:
                for seed in (1, 2):
                    r = simulate(cfg, p, duration_s=max(22.0, start + d + 8.0),
                                 stalls=[StallWindow(start_s=start, duration_s=d)],
                                 seed=seed)
                    n += 1
                    if r.fell:
                        falls += 1
            cells.append(f"{falls}/{n}")
        print(f"{label:20}" + "".join(f"{c:>8}" for c in cells))
    print("(cells = falls/runs across model set x seeds)")

    print("\nsteady-state push DURING stall (worst case: no outer loop, no arms):")
    for d in (0.5, 1.0, 2.0):
        for mag in (2.0, 4.0):
            falls = 0
            n = 0
            for seed in (1, 2, 3):
                r = simulate(cfg, plant, duration_s=25.0,
                             stalls=[StallWindow(start_s=14.8, duration_s=d)],
                             pushes=[Push(at_s=15.0, delta_vel=mag)], seed=seed)
                n += 1
                if r.fell:
                    falls += 1
            print(f"  stall {d:.1f}s + push {mag:.0f} rad/s: {falls}/{n} falls")


def plot_run(r: SimResult, name: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(r.t, r.roll, label="roll")
    axes[0].plot(r.t, r.setpoint, label="setpoint", alpha=0.7)
    axes[0].plot(r.t, r.eq, label="true eq", linestyle="--", alpha=0.7)
    axes[0].set_ylabel("deg")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].plot(r.t, r.cmd, label="cmd", alpha=0.7)
    axes[1].plot(r.t, r.wheel_vel, label="wheel vel")
    axes[1].set_ylabel("rad/s")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[2].plot(r.t, r.drift, label="drift")
    axes[2].plot(r.t, r.sp_offset, label="sp_offset", alpha=0.7)
    axes[2].set_ylabel("rad / deg")
    axes[2].set_xlabel("s")
    axes[2].legend(loc="upper right", fontsize=8)
    out = REPO_ROOT / "telemetry_logs" / "plots" / f"sim_{name}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario", choices=["validate", "standup", "standup-matrix",
                                             "push", "stall-campaign"])
    parser.add_argument("--variant", default="current",
                        help="+-joined parts: baseline curve carrot earlypos rampcont")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--fit", default=str(REPO_ROOT / "telemetry_logs" / "model_fit.json"))
    args = parser.parse_args()

    plant = load_fitted_params(Path(args.fit))
    print(f"plant: A={plant.A:.2f} B={plant.B:.3f} tau_m={plant.tau_m*1000:.0f}ms "
          f"eq_fwd={plant.eq_fwd:.1f} eq_tip_delta={plant.eq_tip_delta:+.1f}\n")

    if args.scenario == "validate":
        scenario_validate(plant)
    elif args.scenario == "standup":
        scenario_standup(plant, args.variant, args.plot, args.seed)
    elif args.scenario == "standup-matrix":
        scenario_standup_matrix(plant)
    elif args.scenario == "push":
        scenario_push(plant, args.variant, args.plot)
    elif args.scenario == "stall-campaign":
        scenario_stall_campaign(plant, args.variant)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
