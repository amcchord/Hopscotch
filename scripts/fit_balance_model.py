#!/usr/bin/env python3
"""Fit a wheeled-inverted-pendulum model from Hopscotch balance telemetry.

Estimates, from the accumulated bal_*.csv logs:
  1. Pendulum dynamics:  roll_accel = A * (roll - roll_eq) + B * wheel_accel
     - A  [1/s^2]   : gravity destabilization coefficient (omega0 = sqrt(A))
     - B  [deg/rad] : tilt reaction per unit wheel acceleration
  2. Motor velocity-loop lag tau_m (first-order fit of measured wheel velocity
     tracking the commanded velocity).
  3. Arm-position -> balance-point curve (tip fraction -> equilibrium roll),
     from stable-balance windows across all runs.
  4. Recommended outer-loop gains (position P -> velocity PI cascade adjusting
     the tilt setpoint) via eigenvalue analysis of the fitted linear model with
     the proven inner PD (Kp=2.0, Kd=0.08) in Speed mode.

Usage:
    .venv/bin/python scripts/fit_balance_model.py [logs...] [--json out.json]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

DT = 0.02  # 50 Hz log rate

# Firmware constants (mirror src/config.h)
ARM_TIP_LEFT = 2.71
ARM_TIP_RIGHT = 1.96
INNER_KP = 2.0   # rad/s per deg
INNER_KD = 0.08  # rad/s per deg/s


# ---------------------------------------------------------------------------
# Log loading
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="*", help="CSV files or directories (default: telemetry_logs/)")
    parser.add_argument("--json", help="Write fitted parameters to this JSON file")
    return parser.parse_args()


def expand_inputs(paths: list[str]) -> list[Path]:
    if not paths:
        paths = [str(REPO_ROOT / "telemetry_logs")]
    results: list[Path] = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            results.extend(sorted(path.glob("*.csv")))
        elif path.exists():
            results.append(path)
    return results


def load_rows(path: Path) -> list[dict[str, str]]:
    lines = path.read_text(errors="replace").splitlines()
    header_idx = next((i for i, line in enumerate(lines) if line.startswith("t_ms,")), None)
    if header_idx is None:
        return []
    end_idx = next(
        (i for i in range(header_idx + 1, len(lines)) if lines[i].startswith("[Balance]")),
        len(lines),
    )
    return list(csv.DictReader(lines[header_idx:end_idx]))


def col(rows: list[dict[str, str]], key: str) -> np.ndarray:
    out = np.full(len(rows), np.nan)
    for i, row in enumerate(rows):
        try:
            out[i] = float(row.get(key, ""))
        except (TypeError, ValueError):
            pass
    return out


def smooth(x: np.ndarray, window: int = 5) -> np.ndarray:
    if len(x) < window:
        return x
    kernel = np.ones(window) / window
    return np.convolve(x, kernel, mode="same")


def derivative(x: np.ndarray) -> np.ndarray:
    return np.gradient(x, DT)


# ---------------------------------------------------------------------------
# Per-run extraction
# ---------------------------------------------------------------------------

@dataclass
class RunData:
    name: str
    t: np.ndarray          # seconds within balance state
    roll: np.ndarray       # deg
    roll_rate: np.ndarray  # deg/s
    setpoint: np.ndarray   # deg
    cmd: np.ndarray        # rad/s commanded
    wheel_vel: np.ndarray  # rad/s measured
    tip_frac: np.ndarray   # 0=arms forward, 1=arms at tip
    arm_bal_frac: np.ndarray | None


def extract_run(path: Path) -> RunData | None:
    rows = load_rows(path)
    if not rows:
        return None

    # Forward reference: arm positions at the very start of the log
    # (tip-up always starts from forward; force-engage runs stay forward).
    fwd_l = None
    fwd_r = None
    for row in rows[:5]:
        try:
            fwd_l = float(row["arm_l"])
            fwd_r = float(row["arm_r"])
            break
        except (KeyError, TypeError, ValueError):
            continue

    bal = [r for r in rows if r.get("state") == "2"]
    if len(bal) < 100:  # need at least 2 s of balance data
        return None

    t = col(bal, "t_ms") / 1000.0
    t = t - t[0]
    roll = col(bal, "roll")
    roll_rate = col(bal, "roll_rate")
    setpoint = col(bal, "setpoint")
    cmd = col(bal, "motor_vel")
    wheel_vel = (col(bal, "bl_vel") + col(bal, "br_vel")) / 2.0

    arm_l = col(bal, "arm_l")
    arm_r = col(bal, "arm_r")
    if fwd_l is not None and fwd_r is not None:
        frac_l = (arm_l - fwd_l) / ARM_TIP_LEFT
        frac_r = (arm_r - fwd_r) / ARM_TIP_RIGHT
        tip_frac = np.clip((frac_l + frac_r) / 2.0, -1.0, 1.5)
    else:
        tip_frac = np.full(len(bal), np.nan)

    abf = col(bal, "arm_bal_frac") if "arm_bal_frac" in bal[0] else None

    return RunData(path.name, t, roll, roll_rate, setpoint, cmd, wheel_vel, tip_frac, abf)


# ---------------------------------------------------------------------------
# 1. Pendulum dynamics fit
# ---------------------------------------------------------------------------

def fit_pendulum(runs: list[RunData]) -> tuple[float, float, list[dict]]:
    """Regress roll_accel = A*roll + B*wheel_accel + c on active-balance segments."""
    seg_fits: list[dict] = []

    for run in runs:
        roll_rate_s = smooth(run.roll_rate)
        roll_accel = derivative(roll_rate_s)
        wheel_vel_s = smooth(run.wheel_vel)
        wheel_accel = derivative(wheel_vel_s)

        # Use dynamically active samples (motion present but not saturated/falling):
        # regression needs excitation, and near-steady samples only add noise.
        valid = (
            np.isfinite(roll_accel)
            & np.isfinite(wheel_accel)
            & (np.abs(run.roll - 90.0) < 25.0)
            & (np.abs(run.cmd) < 24.0)
        )
        if valid.sum() < 200:
            continue

        theta = run.roll[valid]
        acc = roll_accel[valid]
        wacc = wheel_accel[valid]

        X = np.column_stack([theta, wacc, np.ones(len(theta))])
        coef, residuals, rank, _ = np.linalg.lstsq(X, acc, rcond=None)
        if rank < 3:
            continue
        pred = X @ coef
        ss_res = float(np.sum((acc - pred) ** 2))
        ss_tot = float(np.sum((acc - acc.mean()) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

        a_est, b_est, c_est = coef
        if a_est <= 0:
            continue  # unphysical (pendulum must be unstable upright)
        seg_fits.append({
            "run": run.name,
            "n": int(valid.sum()),
            "A": float(a_est),
            "B": float(b_est),
            "theta_eq": float(-c_est / a_est),
            "r2": r2,
        })

    if not seg_fits:
        raise SystemExit("No usable segments for pendulum fit")

    # Weight by n * r2 (only positive-quality fits)
    good = [s for s in seg_fits if s["r2"] > 0.1]
    if not good:
        good = seg_fits
    weights = np.array([s["n"] * max(s["r2"], 0.01) for s in good])
    A = float(np.average([s["A"] for s in good], weights=weights))
    B = float(np.average([s["B"] for s in good], weights=weights))
    return A, B, seg_fits


# ---------------------------------------------------------------------------
# 2. Motor velocity-loop lag
# ---------------------------------------------------------------------------

def fit_motor_lag(runs: list[RunData]) -> float:
    """Fit v_dot = (cmd - v)/tau across all runs; return tau in seconds."""
    num = 0.0
    den = 0.0
    for run in runs:
        v = smooth(run.wheel_vel, 3)
        vdot = derivative(v)
        err = run.cmd - v
        valid = np.isfinite(vdot) & np.isfinite(err) & (np.abs(run.cmd) < 24.0)
        if valid.sum() < 100:
            continue
        # least squares for k in vdot = k*err  ->  tau = 1/k
        num += float(np.sum(err[valid] * vdot[valid]))
        den += float(np.sum(err[valid] ** 2))
    if den <= 0 or num <= 0:
        return 0.05  # fallback: 50 ms
    k = num / den
    return 1.0 / k


# ---------------------------------------------------------------------------
# 3. Arm-position -> balance-point curve
# ---------------------------------------------------------------------------

def stable_windows(run: RunData, err_max: float = 1.5, rate_max: float = 5.0,
                   min_len_s: float = 1.0) -> list[tuple[int, int]]:
    err = np.abs(run.setpoint - run.roll)
    ok = (err < err_max) & (np.abs(run.roll_rate) < rate_max) & np.isfinite(run.roll)
    windows: list[tuple[int, int]] = []
    start = None
    for i, flag in enumerate(ok):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if (i - start) * DT >= min_len_s:
                windows.append((start, i))
            start = None
    if start is not None and (len(ok) - start) * DT >= min_len_s:
        windows.append((start, len(ok)))
    return windows


def fit_arm_curve(runs: list[RunData]) -> list[dict]:
    """Collect (tip_frac, mean roll) points from stable windows, binned."""
    points: list[tuple[float, float, float]] = []  # (tip_frac, roll, weight)
    for run in runs:
        for lo, hi in stable_windows(run):
            frac = float(np.nanmean(run.tip_frac[lo:hi]))
            roll = float(np.nanmean(run.roll[lo:hi]))
            # During stable balance the robot IS at its equilibrium, so mean
            # roll in the window is a direct balance-point measurement.
            if math.isfinite(frac) and math.isfinite(roll):
                points.append((frac, roll, (hi - lo) * DT))

    if not points:
        return []

    bins = [(-0.10, 0.05), (0.05, 0.20), (0.20, 0.40), (0.40, 0.60),
            (0.60, 0.80), (0.80, 1.05)]
    curve: list[dict] = []
    for lo, hi in bins:
        sel = [(f, r, w) for f, r, w in points if lo <= f < hi]
        if not sel:
            continue
        wsum = sum(w for _, _, w in sel)
        frac = sum(f * w for f, _, w in sel) / wsum
        roll = sum(r * w for _, r, w in sel) / wsum
        curve.append({
            "tip_frac": round(frac, 3),
            "balance_deg": round(roll, 2),
            "windows": len(sel),
            "seconds": round(wsum, 1),
        })
    return curve


# ---------------------------------------------------------------------------
# 4. Outer-loop gain recommendation (eigenvalue analysis)
# ---------------------------------------------------------------------------

def closed_loop_matrix(A: float, B: float, tau_m: float,
                       kp: float, kd: float,
                       kx: float, kv: float, ki: float,
                       tau_f: float = 0.1) -> np.ndarray:
    """Linearized closed loop in Speed mode.

    States: [th (deg from eq), w (deg/s), x (wheel rad), v (rad/s),
             vf (filtered v), z (velocity-loop integral, deg)]

    Inner:  v_cmd = kp*(u - th) - kd*w        (u = outer setpoint offset, deg)
    Motor:  v'    = (v_cmd - v)/tau_m
    Plant:  w'    = A*th + B*v'
    Outer:  target = -kx*x
            e  = target - vf
            u  = -(kv*e + z)      (positive lean-back offset decelerates,
            z' = ki*e              so accelerating requires negative offset)
            vf' = (v - vf)/tau_f
    """
    M = np.zeros((6, 6))
    # u = -(kv*(-kx*x - vf) + z)  ->  du/d[states]
    u_x = kv * kx
    u_vf = kv
    u_z = -1.0

    # v' = (kp*(u - th) - kd*w - v)/tau_m
    vp = np.zeros(6)
    vp[0] = -kp / tau_m
    vp[1] = -kd / tau_m
    vp[2] = kp * u_x / tau_m
    vp[3] = -1.0 / tau_m
    vp[4] = kp * u_vf / tau_m
    vp[5] = kp * u_z / tau_m

    M[0, 1] = 1.0                       # th' = w
    M[1] = B * vp                       # w' = A*th + B*v'
    M[1, 0] += A
    M[2, 3] = 1.0                       # x' = v
    M[3] = vp                           # v'
    M[4, 3] = 1.0 / tau_f               # vf' = (v - vf)/tau_f
    M[4, 4] = -1.0 / tau_f
    M[5, 2] = -ki * kx                  # z' = ki*(-kx*x - vf)
    M[5, 4] = -ki
    return M


def model_uncertainty_set(A_fit: float, B_fit: float) -> list[tuple[float, float]]:
    """Plausible (A, B) pairs the recommended gains must stabilize.

    Closed-loop identification is biased (the PD correlates wheel accel with
    tilt), so require stability for A at 0.5x-2x and B at 0.7x-1.4x of the
    fitted values.
    """
    return [(A_fit * fa, B_fit * fb)
            for fa in (0.5, 1.0, 2.0)
            for fb in (0.7, 1.0, 1.4)]


def recommend_gains(A_fit: float, B_fit: float, tau_m: float) -> list[dict]:
    """Grid-search outer gains stable across the whole model uncertainty set."""
    models = model_uncertainty_set(A_fit, B_fit)
    # The CSP-derived tau_m is an artifact of the position servo and does not
    # predict the Speed-mode velocity loop. Require stability across a
    # realistic Speed-mode lag range instead.
    taus = (0.03, 0.08)
    _ = tau_m

    kx_grid = [0.02, 0.05, 0.08, 0.12, 0.20]
    kv_grid = [0.3, 0.5, 0.6, 0.8, 1.0, 1.4]
    ki_grid = [0.02, 0.05, 0.10, 0.20]

    results: list[dict] = []
    for kx in kx_grid:
        for kv in kv_grid:
            for ki in ki_grid:
                worst_real = -math.inf
                worst_damp = 1.0
                stable = True
                for (a, b) in models:
                    for tau in taus:
                        M = closed_loop_matrix(a, b, tau, INNER_KP, INNER_KD, kx, kv, ki)
                        eig = np.linalg.eigvals(M)
                        max_real = float(np.max(eig.real))
                        if max_real >= -1e-4:
                            stable = False
                            break
                        worst_real = max(worst_real, max_real)
                        for lam in eig:
                            if abs(lam.imag) > 1e-6:
                                worst_damp = min(worst_damp, -lam.real / abs(lam))
                    if not stable:
                        break
                if not stable:
                    continue
                # Speed of the slow drift-return pole on the *fitted* model
                # (the metric a user actually feels: how fast it re-centers)
                eig_fit = np.linalg.eigvals(closed_loop_matrix(
                    A_fit, B_fit, min(taus), INNER_KP, INNER_KD, kx, kv, ki))
                fitted_slow = float(np.max(eig_fit.real))
                results.append({
                    "kx": kx, "kv": kv, "ki": ki,
                    "worst_max_real": worst_real,
                    "worst_damping": worst_damp,
                    "fitted_slow_pole": fitted_slow,
                })

    # Rank: well-damped oscillatory modes first, then fastest drift return
    results.sort(key=lambda r: (-min(r["worst_damping"], 0.3), r["fitted_slow_pole"]))
    return results[:10]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()
    paths = expand_inputs(args.paths)
    runs = [run for path in paths if (run := extract_run(path)) is not None]
    print(f"Loaded {len(runs)} runs with >=2s of balance data (from {len(paths)} files)\n")

    # 1. Pendulum fit
    A, B, seg_fits = fit_pendulum(runs)
    omega0 = math.sqrt(A)
    print("=== Pendulum dynamics:  roll_accel = A*(roll - eq) + B*wheel_accel ===")
    print(f"  A       = {A:8.3f}  1/s^2   (gravity destabilization)")
    print(f"  omega0  = {omega0:8.3f}  rad/s   (unstable pole, time-to-double "
          f"{math.log(2)/omega0*1000:.0f} ms)")
    print(f"  B       = {B:8.4f}  deg per rad/s^2 of wheel accel")
    good = sorted((s for s in seg_fits if s["r2"] > 0.1), key=lambda s: -s["r2"])
    print(f"  fits: {len(seg_fits)} runs, {len(good)} with r2>0.1; best:")
    for s in good[:5]:
        print(f"    {s['run']:28} A={s['A']:7.2f} B={s['B']:7.3f} "
              f"eq={s['theta_eq']:6.2f} r2={s['r2']:.2f} n={s['n']}")

    # 2. Motor lag
    tau_m = fit_motor_lag(runs)
    print(f"\n=== Motor velocity tracking (CSP-era data, first-order fit) ===")
    print(f"  tau_m   = {tau_m*1000:6.1f}  ms")
    print("  NOTE: measured through the CSP position servo. Speed mode should be")
    print("  similar or faster; re-run against Speed-mode logs after Phase 2.")

    # 3. Arm curve
    curve = fit_arm_curve(runs)
    print("\n=== Arm tip-fraction -> balance point (stable windows) ===")
    if curve:
        print(f"  {'tip_frac':>8} {'balance_deg':>11} {'windows':>7} {'seconds':>8}")
        for c in curve:
            print(f"  {c['tip_frac']:8.3f} {c['balance_deg']:11.2f} "
                  f"{c['windows']:7d} {c['seconds']:8.1f}")
        print("  -> use as BALANCE_SP_CURVE anchors in src/config.h")
    else:
        print("  no stable windows found")

    # 4. Gain recommendation
    print("\n=== Recommended outer gains (position P -> velocity PI, Speed mode) ===")
    print(f"  fitted model A={A:.2f}, B={B:.3f}; gains must stabilize A x0.5-2,")
    print("  B x0.7-1.4, Speed-mode motor lag 30-80 ms. "
          f"Inner Kp={INNER_KP}, Kd={INNER_KD}, vel filter 100 ms.")
    recs = recommend_gains(A, B, tau_m)
    if recs:
        print(f"  {'kx':>6} {'kv':>6} {'ki':>6} {'wrst_re':>8} {'wrst_dmp':>8} {'slow_pole':>9}")
        for r in recs:
            print(f"  {r['kx']:6.2f} {r['kv']:6.2f} {r['ki']:6.2f} "
                  f"{r['worst_max_real']:8.4f} {r['worst_damping']:8.3f} "
                  f"{r['fitted_slow_pole']:9.4f}")
        best = recs[0]
        print(f"\n  -> suggest BALANCE_DRIFT_VEL_KP={best['kx']:.2f} (rad/s per rad drift),")
        print(f"             BALANCE_VEL_SP_KP={best['kv']:.2f} (deg per rad/s vel err),")
        print(f"             BALANCE_VEL_SP_KI={best['ki']:.2f} (deg/s per rad/s vel err)")
        print("  Margins are structurally thin for this plant; treat these as starting")
        print("  points and re-fit from Speed-mode telemetry after the first sessions.")
    else:
        print("  no stable gain combination found in grid -- check model fit quality")

    if args.json:
        payload = {
            "A": A, "B": B, "omega0": omega0, "tau_m": tau_m,
            "arm_curve": curve, "gain_candidates": recs,
            "segment_fits": seg_fits,
        }
        Path(args.json).write_text(json.dumps(payload, indent=2))
        print(f"\nWrote {args.json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
