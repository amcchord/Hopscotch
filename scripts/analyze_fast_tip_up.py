#!/usr/bin/env python3
"""Archived-data diagnosis and kinematic screening, NOT a contact dynamics model."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read_rows(path):
    with path.open() as stream:
        return [{k: float(v) for k, v in row.items()}
                for row in csv.DictReader(line for line in stream if not line.startswith("#"))]


def summarize(path):
    rows = read_rows(path)
    tipping = [row for row in rows if row["state"] == 1]
    first_balance = next(row for row in rows if row["state"] == 2)
    t0 = tipping[0]["t_ms"]
    thresholds = {}
    for deg in [60, 70, 75, 80]:
        hit = next((row for row in tipping if row["roll"] >= deg), None)
        thresholds[str(deg)] = None if hit is None else (hit["t_ms"] - t0) / 1000
    return {
        "file": path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "rows": len(rows), "tip_rows": len(tipping),
        "tip_to_balance_s": (first_balance["t_ms"] - t0) / 1000,
        "first_crossing_deg_s": thresholds,
        "engage_roll_deg": first_balance["roll"],
        "engage_rate_dps": first_balance["roll_rate"],
        "peak_tip_rate_dps": max(abs(row["roll_rate"]) for row in tipping),
        "peak_target_tracking_error_rad": {
            side: max(abs(row[f"arm_{side}_tgt"] - row[f"arm_{side}"]) for row in tipping)
            for side in ["l", "r"]
        },
        "peak_arm_torque_nm": {
            side: max(abs(row[f"arm_{side}_torque"]) for row in tipping) for side in ["l", "r"]
        },
    }, tipping


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latest", type=Path, help="Additional same-project archive, read in place")
    parser.add_argument("--output", type=Path, default=ROOT / "evidence/fast-tip-up")
    args = parser.parse_args()
    paths = [ROOT / "telemetry_logs" / name for name in [
        "bal_20260920_lowering_trial_wifi.csv", "bal_20260920_forward_catch_v2_wifi.csv",
        "bal_20260920_recovered_v2_bailout.csv"]]
    if args.latest:
        paths.append(args.latest)
    results, runs = zip(*(summarize(path) for path in paths))
    args.output.mkdir(parents=True, exist_ok=True)
    executable = ROOT / "output/test_balance_tip_up"
    subprocess.run(["clang++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Isrc",
                    "tests/test_balance_tip_up.cpp", "-o", str(executable)], cwd=ROOT, check=True)
    test_result = subprocess.check_output([str(executable)], text=True).strip()
    trajectory = subprocess.check_output([str(executable), "--dump"], text=True)
    (args.output / "trajectory.csv").write_text(trajectory)
    planned = np.genfromtxt(io.StringIO(trajectory), delimiter=",", names=True)

    # Replay the historical body angle as a function of left-arm travel, along
    # the preserved arm path. This ONLY estimates geometry/timing: accelerating
    # reaction torque, friction, slip, contact loss and IMU bias are absent.
    estimates = []
    for summary, run in zip(results, runs):
        travel = np.array([row["arm_l"] - run[0]["arm_l"] for row in run])
        angle = np.array([row["roll"] for row in run])
        # Bin the noisy measured trace in 0.04-rad intervals, taking medians.
        points = [(0., float(angle[0]))]
        for edge in np.arange(.04, travel.max(), .04):
            mask = (travel >= edge - .02) & (travel < edge + .02)
            if mask.any():
                points.append((float(np.median(travel[mask])), float(np.median(angle[mask]))))
        points.append((float(travel.max()), float(np.median(angle[-10:]))))
        x, y = np.array(points).T
        replay = np.interp(planned["left"], x, y)
        # 100 ms rate window reduces derivative amplification of quantized logs.
        rate = np.diff(replay[::5]) / np.diff(planned["t_s"][::5])
        estimates.append({"file": summary["file"],
                          "kinematic_peak_100ms_rate_dps": float(np.max(np.abs(rate))),
                          "kinematic_final_roll_deg": float(replay[-1])})

    length = 2.71
    duration_comparison = [{"trajectory_s": seconds,
        "quintic_left_peak_speed_rad_s": 1.875 * length / seconds,
        "quintic_left_peak_acceleration_rad_s2": 10 / np.sqrt(3) * length / seconds**2}
        for seconds in [1., 2.6, 3.]]
    output = {"kind": "kinematic_screen_not_physical_validation", "archived_runs": results,
              "production_policy_test": test_result, "trajectory_duration_comparison": duration_comparison,
              "archived_geometry_replays": estimates,
              "limits": ["No fast hardware trial has occurred.",
                         "Geometry replay assumes the slow contact path survives faster acceleration.",
                         "Body/contact inertia, torque demand, slip and sensor acceleration bias are unmodeled.",
                         "Startup motor-mode setup and 500 ms CH11 recognition delay precede logged tip time.",
                         "Existing capture hold and arm return follow balance engagement; total settling is longer."]}
    (args.output / "analysis.json").write_text(json.dumps(output, indent=2) + "\n")

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.4))
    for summary, run in zip(results, runs):
        axes[0].plot([(r["t_ms"]-run[0]["t_ms"])/1000 for r in run],
                     [r["roll"] for r in run], linewidth=1.5,
                     label=f'{summary["tip_to_balance_s"]:.3f} s to balance')
    axes[0].set(title="Recorded slow tip-ups", xlabel="Seconds after tip-up logging starts", ylabel="Body angle (degrees)")
    axes[0].legend(fontsize=8)
    axes[1].plot(planned["t_s"], planned["left"], label="Left arm target")
    axes[1].plot(planned["t_s"], planned["right"], label="Right arm target")
    axes[1].axvline(2.6, color="#555", linestyle="--", linewidth=1, label="Planned motion complete")
    axes[1].set(title="New fast trajectory — untested on robot", xlabel="Trajectory time (seconds)", ylabel="Travel from forward pose (radians)")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.2)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Replace the long slowdown with a smooth 2.6-second arm trajectory", fontsize=14)
    fig.tight_layout()
    fig.savefig(args.output / "tip-up-comparison.png", dpi=160)
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
