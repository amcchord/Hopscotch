#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize Hopscotch balance telemetry logs."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="CSV files or directories. Defaults to telemetry_logs/.",
    )
    parser.add_argument(
        "--details",
        action="store_true",
        help="Print a short per-file detail block in addition to the summary table.",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Write a per-run PNG (tilt/setpoint, commands, drift) to telemetry_logs/plots/.",
    )
    parser.add_argument(
        "--plot-dir",
        default=str(REPO_ROOT / "telemetry_logs" / "plots"),
        help="Output directory for --plot (default: telemetry_logs/plots/).",
    )
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


def parse_config(path: Path) -> dict[str, str]:
    """Parse # KEY=VALUE lines at the top of the CSV."""
    config: dict[str, str] = {}
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if line.startswith("t_ms,"):
            break
        if not line.startswith("#"):
            continue  # several old USB captures begin with a partial debug line
        if "=" in line:
            key, _, value = line.lstrip("# ").partition("=")
            config[key.strip()] = value.strip()
    return config


def load_rows(path: Path) -> list[dict[str, str]]:
    lines = path.read_text(errors="replace").splitlines()
    header_idx = next((i for i, line in enumerate(lines) if line.startswith("t_ms,")), None)
    if header_idx is None:
        return []

    end_idx = next(
        (i for i in range(header_idx + 1, len(lines)) if lines[i].startswith("[Balance] --- End of log ---")),
        len(lines),
    )
    return list(csv.DictReader(lines[header_idx:end_idx]))


def as_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def as_int(row: dict[str, str], key: str) -> int:
    try:
        return int(float(row.get(key, "0")))
    except (TypeError, ValueError):
        return 0


def vel_offset_series(rows: Iterable[dict[str, str]]) -> list[float]:
    result: list[float] = []
    for row in rows:
        if "sp_offset" in row:
            result.append(as_float(row, "sp_offset"))
        elif "vel_offset" in row:
            result.append(as_float(row, "vel_offset"))
        elif "vel_integ" in row:
            result.append(as_float(row, "vel_integ"))
        else:
            result.append(math.nan)
    return result


def mean(values: Iterable[float]) -> float:
    cleaned = [v for v in values if not math.isnan(v)]
    if not cleaned:
        return math.nan
    return sum(cleaned) / len(cleaned)


def max_abs(values: Iterable[float]) -> float:
    cleaned = [abs(v) for v in values if not math.isnan(v)]
    return max(cleaned, default=math.nan)


def max_value(values: Iterable[float]) -> float:
    cleaned = [v for v in values if not math.isnan(v)]
    return max(cleaned, default=math.nan)


def min_value(values: Iterable[float]) -> float:
    cleaned = [v for v in values if not math.isnan(v)]
    return min(cleaned, default=math.nan)


def percentile(values: Iterable[float], fraction: float) -> float:
    cleaned = sorted(v for v in values if not math.isnan(v))
    if not cleaned:
        return math.nan
    index = min(len(cleaned) - 1, max(0, round((len(cleaned) - 1) * fraction)))
    return cleaned[index]


def fmt(value: float, width: int = 5, precision: int = 2) -> str:
    if math.isnan(value):
        return " " * (width - 2) + "--"
    return f"{value:{width}.{precision}f}"


def summarize(path: Path) -> dict[str, object] | None:
    rows = load_rows(path)
    if not rows:
        return None

    config = parse_config(path)
    balance_rows = [row for row in rows if row.get("state") == "2"]
    tip_rows = [row for row in rows if row.get("state") == "1"]
    offset_name = next((key for key in ("sp_offset", "vel_offset", "vel_integ") if key in rows[0]), "offset")
    offset_unit = "deg" if offset_name == "sp_offset" else "rad/s"
    if not balance_rows:
        return {
            "file": path.name,
            "tip_s": (as_float(tip_rows[-1], "t_ms") - as_float(tip_rows[0], "t_ms")) / 1000.0 if len(tip_rows) > 1 else math.nan,
            "bal_s": math.nan,
            "eng_roll": math.nan,
            "eng_rate": math.nan,
            "return_s": math.nan,
            "pre_err": math.nan,
            "pre_rate": math.nan,
            "pre_cmd": math.nan,
            "final_sp": math.nan,
            "final_vel_off": math.nan,
            "offset_name": offset_name,
            "offset_unit": offset_unit,
            "final_drift": math.nan,
            "config": config,
            "diag": {},
            "note": "no balance state",
        }

    engage = balance_rows[0]
    engage_time = as_float(engage, "t_ms")
    engage_arm_l = as_float(engage, "arm_l")
    engage_arm_r = as_float(engage, "arm_r")

    return_idx = next(
        (
            i
            for i, row in enumerate(balance_rows[1:], start=1)
            if abs(as_float(row, "arm_l") - engage_arm_l) > 0.02
            or abs(as_float(row, "arm_r") - engage_arm_r) > 0.02
        ),
        None,
    )

    if return_idx is not None:
        return_row = balance_rows[return_idx]
        return_s = (as_float(return_row, "t_ms") - engage_time) / 1000.0
        pre_window = balance_rows[max(0, return_idx - 10) : return_idx + 1]
        pre_err = mean(abs(as_float(row, "setpoint") - as_float(row, "roll")) for row in pre_window)
        pre_rate = mean(abs(as_float(row, "roll_rate")) for row in pre_window)
        pre_cmd = mean(abs(as_float(row, "motor_vel")) for row in pre_window)
    else:
        return_s = math.nan
        pre_err = math.nan
        pre_rate = math.nan
        pre_cmd = math.nan

    final_window = balance_rows[-min(100, len(balance_rows)) :]
    final_vel_values = vel_offset_series(final_window)

    notes: list[str] = []
    has_new_schema = any(key in balance_rows[0]
                         for key in ("sp_offset", "vel_offset", "vel_integ"))
    if not has_new_schema:
        notes.append("legacy schema")
    if not math.isnan(pre_rate) and pre_rate > 4.0:
        notes.append("arm return before settle")
    if not math.isnan(pre_cmd) and pre_cmd > 1.0:
        notes.append("high cmd at return")
    drift_key = "meas_drift" if "meas_drift" in balance_rows[0] else ""

    diag: dict[str, float | int] = {}
    diag["sample_dt_p99_ms"] = percentile(
        (as_float(row, "sample_dt_ms") for row in rows), 0.99
    )
    gaps = [as_float(b, "t_ms") - as_float(a, "t_ms") for a, b in zip(rows, rows[1:])]
    diag["sample_dt_max_ms"] = max_value(gaps)
    diag["sample_dt_p99_ms"] = percentile(gaps, 0.99)
    diag["imu_age_max_ms"] = max_value(as_float(row, "imu_age_ms") for row in rows)
    diag["imu_fault_rows"] = sum(bool(as_int(row, "diag_flags") & 0x0400) for row in rows)
    diag["can_tx_failed_rows"] = sum(bool(as_int(row, "diag_flags") & 0x0800) for row in rows)
    diag["inner_dt_max_us"] = max_value(as_float(row, "inner_dt_max_us") for row in rows)
    diag["inner_tick_low_rows"] = sum(
        1 for row in rows if "inner_ticks" in row and as_int(row, "inner_ticks") < 3
    )
    diag["feedback_age_max_ms"] = max_value(
        max(as_float(row, "feedback_age_l_ms"), as_float(row, "feedback_age_r_ms"))
        for row in rows
    )
    diag["max_angle_err_deg"] = max_abs(as_float(row, "angle_err") for row in balance_rows)
    diag["max_cmd_raw_rad_s"] = max_abs(
        as_float(row, "motor_vel_raw" if "motor_vel_raw" in row else "motor_vel")
        for row in balance_rows
    )
    diag["max_wheel_vel_rad_s"] = max_abs(
        max(abs(as_float(row, "bl_vel")), abs(as_float(row, "br_vel")))
        for row in balance_rows
    )
    diag["max_drift_rad"] = max_abs(as_float(row, drift_key) for row in balance_rows) if drift_key else math.nan
    diag["max_imu_disagreement_deg"] = max_abs(
        as_float(row, "roll") - as_float(row, "accel_angle") for row in balance_rows
    ) if "accel_angle" in balance_rows[0] else math.nan
    diag["accel_norm_min_g"] = min_value(as_float(row, "accel_norm") for row in balance_rows)
    diag["accel_norm_max_g"] = max_value(as_float(row, "accel_norm") for row in balance_rows)
    diag["min_bus_voltage"] = min_value(as_float(row, "bus_voltage") for row in balance_rows)
    diag["max_total_current"] = max_value(as_float(row, "total_current") for row in balance_rows)
    diag["max_rear_torque_nm"] = max_abs(
        max(abs(as_float(row, "bl_torque")), abs(as_float(row, "br_torque")))
        for row in balance_rows
    )
    diag["saturated_rows"] = sum(
        1 for row in balance_rows
        if as_int(row, "inner_sat_ticks") > 0 or as_int(row, "diag_flags") & 0x0001
    )
    diag["sp_clamped_rows"] = sum(
        1 for row in balance_rows if as_int(row, "diag_flags") & 0x0080
    )
    diag["emergency_arm_rows"] = sum(
        1 for row in balance_rows if as_int(row, "diag_flags") & 0x0040
    )
    diag["arm_active_rows"] = sum(
        1 for row in balance_rows if as_int(row, "arm_stage") in (1, 2)
    )
    if as_int(config, "telemetry_features") & 16:
        origin_ms = as_float(balance_rows[0], "t_ms") if balance_rows else 0
        for label, bit in (("active",0x1000),("boost",0x2000),("limited",0x4000),("settled",0x8000)):
            matching = [row for row in balance_rows if as_int(row,"diag_flags") & bit]
            diag[f"startup_recovery_{label}_rows"] = len(matching)
            diag[f"startup_recovery_{label}_first_s"] = ((as_float(matching[0],"t_ms")-origin_ms)/1000
                                                        if matching else math.nan)
    if as_int(config, "telemetry_features") & 32:
        matching = [row for row in balance_rows if as_int(row, "flags") & 0x01]
        origin_ms = as_float(balance_rows[0], "t_ms")
        diag["startup_recoil_rows"] = len(matching)
        for label, row in (("first", matching[:1]), ("last", matching[-1:])):
            diag[f"startup_recoil_{label}_s"] = ((as_float(row[0], "t_ms")-origin_ms)/1000
                                                if row else math.nan)
    diag["markers"] = max((as_int(row, "marker") for row in rows), default=0)

    if len(rows) >= 2999 and not config.get("end_reason"):
        notes.append("possible log cutoff; end cause unknown")
    if diag["imu_fault_rows"]:
        notes.append("IMU freshness fault")
    if diag["can_tx_failed_rows"]:
        notes.append("CAN transmit failure")
    if config.get("checksum_valid") == "0":
        notes.append("BAD CHECKSUM")
    if config.get("end_reason") not in (None, "balance_switch_off", "duration_limit"):
        notes.append(f"end={config['end_reason']}")
    if diag["sample_dt_max_ms"] > 50:
        notes.append("sample gap")
    if diag["saturated_rows"]:
        notes.append("inner saturation")
    if not notes:
        notes.append("clean capture")

    return {
        "file": path.name,
        "tip_s": (as_float(tip_rows[-1], "t_ms") - as_float(tip_rows[0], "t_ms")) / 1000.0 if len(tip_rows) > 1 else math.nan,
        "bal_s": (as_float(balance_rows[-1], "t_ms") - engage_time) / 1000.0,
        "eng_roll": as_float(engage, "roll"),
        "eng_rate": as_float(engage, "roll_rate"),
        "return_s": return_s,
        "pre_err": pre_err,
        "pre_rate": pre_rate,
        "pre_cmd": pre_cmd,
        "final_sp": mean(as_float(row, "setpoint") for row in final_window),
        "final_vel_off": mean(final_vel_values),
        "offset_name": offset_name,
        "offset_unit": offset_unit,
        "final_drift": mean(as_float(row, drift_key) for row in final_window) if drift_key else math.nan,
        "config": config,
        "diag": diag,
        "note": ", ".join(notes),
    }


def plot_run(path: Path, out_dir: Path) -> Path | None:
    """Write a 3-panel PNG (tilt/setpoint, commands/velocity, drift) for one run."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping plots "
              "(install with: .venv/bin/pip install matplotlib)")
        return None

    rows = load_rows(path)
    if not rows:
        return None

    def series(key: str) -> list[float]:
        return [as_float(row, key) for row in rows]

    t = [v / 1000.0 for v in series("t_ms")]
    states = [row.get("state", "") for row in rows]
    roll = series("roll")
    setpoint = series("setpoint")
    cmd = series("motor_vel")
    bl_vel = series("bl_vel")
    br_vel = series("br_vel")
    wheel_vel = [(a + b) / 2.0 for a, b in zip(bl_vel, br_vel)]
    drift = series("meas_drift") if "meas_drift" in rows[0] else None
    sp_off_key = next((k for k in ("sp_offset", "vel_trim", "pos_shift") if k in rows[0]), None)

    engage_t = next((tv for tv, st in zip(t, states) if st == "2"), None)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    ax = axes[0]
    ax.plot(t, roll, label="roll (deg)", linewidth=0.9)
    ax.plot(t, setpoint, label="setpoint (deg)", linewidth=0.9, linestyle="--")
    ax.set_ylabel("deg")
    ax.set_title(path.name)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, cmd, label="motor_vel cmd (rad/s)", linewidth=0.9)
    ax.plot(t, wheel_vel, label="measured wheel vel (rad/s)", linewidth=0.9, alpha=0.8)
    if sp_off_key is not None:
        ax.plot(t, series(sp_off_key), label=f"{sp_off_key} (deg)", linewidth=0.9, linestyle=":")
    ax.set_ylabel("rad/s / deg")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    if drift is not None:
        ax.plot(t, drift, label="meas_drift (rad)", linewidth=0.9)
    else:
        bl_pos = series("bl_pos")
        br_pos = series("br_pos")
        avg0 = (bl_pos[0] + br_pos[0]) / 2.0 if bl_pos else 0.0
        ax.plot(t, [(a + b) / 2.0 - avg0 for a, b in zip(bl_pos, br_pos)],
                label="wheel pos - start (rad)", linewidth=0.9)
    ax.set_ylabel("rad")
    ax.set_xlabel("time (s)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    if engage_t is not None:
        for ax in axes:
            ax.axvline(engage_t, color="green", alpha=0.4, linewidth=1)

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / (path.stem + ".png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def main() -> int:
    args = parse_args()
    paths = expand_inputs(args.paths)
    summaries = [summary for path in paths if (summary := summarize(path)) is not None]

    if not summaries:
        print("No parseable balance logs found.")
        return 1

    header = (
        f"{'file':24} {'tip_s':>6} {'bal_s':>6} {'eng_roll':>8} {'eng_rate':>8} "
        f"{'ret_s':>6} {'pre_err':>8} {'pre_rate':>9} {'pre_cmd':>8} "
        f"{'final_sp':>8} {'offset':>8} {'unit':>5} {'drift':>8}  note"
    )
    print(header)
    print("-" * len(header))
    for summary in summaries:
        print(
            f"{summary['file']:24} "
            f"{fmt(summary['tip_s'], 6, 2)} "
            f"{fmt(summary['bal_s'], 6, 2)} "
            f"{fmt(summary['eng_roll'], 8, 2)} "
            f"{fmt(summary['eng_rate'], 8, 2)} "
            f"{fmt(summary['return_s'], 6, 2)} "
            f"{fmt(summary['pre_err'], 8, 2)} "
            f"{fmt(summary['pre_rate'], 9, 2)} "
            f"{fmt(summary['pre_cmd'], 8, 2)} "
            f"{fmt(summary['final_sp'], 8, 2)} "
            f"{fmt(summary['final_vel_off'], 8, 2)} "
            f"{summary['offset_unit']:>5} "
            f"{fmt(summary['final_drift'], 8, 2)}  "
            f"{summary['note']}"
        )

    if args.details:
        print()
        for path in paths:
            config = parse_config(path)
            summary = next((s for s in summaries if s["file"] == path.name), None)
            if summary is None:
                continue

            print(summary["file"])
            if summary['diag'] and 'imu_age_max_ms' in summary['diag']:
                diag = summary['diag']
                print(f"  IMU max age={fmt(diag['imu_age_max_ms'], 0, 0)} ms, "
                      f"freshness-fault rows={diag['imu_fault_rows']}, CAN TX-failure rows={diag['can_tx_failed_rows']}")
            if config:
                gains = []
                for key in ("inner_kp", "inner_kd", "drift_vel_kp", "vel_sp_kp", "vel_sp_kp_low", "vel_sp_ki"):
                    if key in config:
                        gains.append(f"{key}={config[key]}")
                if gains:
                    print(f"  config: {', '.join(gains)}")
                extras = []
                for key in ("sp_offset_max", "ramp_sp_offset_max", "max_drive_speed", "base_sp_fwd", "base_sp_tip"):
                    if key in config:
                        extras.append(f"{key}={config[key]}")
                if extras:
                    print(f"          {', '.join(extras)}")

            print(
                f"  engage: roll={fmt(summary['eng_roll'], 0, 2).strip()} deg, "
                f"rate={fmt(summary['eng_rate'], 0, 2).strip()} dps"
            )
            print(
                f"  arm return: {fmt(summary['return_s'], 0, 2).strip()} s, "
                f"pre-return |err|={fmt(summary['pre_err'], 0, 2).strip()} deg, "
                f"|rate|={fmt(summary['pre_rate'], 0, 2).strip()} dps, "
                f"|cmd|={fmt(summary['pre_cmd'], 0, 2).strip()} rad/s"
            )
            print(
                f"  final window: sp={fmt(summary['final_sp'], 0, 2).strip()} deg, "
                f"{summary['offset_name']}={fmt(summary['final_vel_off'], 0, 2).strip()} {summary['offset_unit']}, "
                f"drift={fmt(summary['final_drift'], 0, 2).strip()} rad"
            )
            print(f"  note: {summary['note']}")
            diag = summary.get("diag", {})
            if diag:
                print(
                    "  timing: "
                    f"sample p99/max={fmt(diag['sample_dt_p99_ms'], 0, 1).strip()}/"
                    f"{fmt(diag['sample_dt_max_ms'], 0, 1).strip()} ms, "
                    f"inner max={fmt(diag['inner_dt_max_us'], 0, 0).strip()} us, "
                    f"low-tick rows={diag['inner_tick_low_rows']}, "
                    f"feedback max={fmt(diag['feedback_age_max_ms'], 0, 0).strip()} ms"
                )
                print(
                    "  authority: "
                    f"|angle err|max={fmt(diag['max_angle_err_deg'], 0, 2).strip()} deg, "
                    f"|raw cmd|max={fmt(diag['max_cmd_raw_rad_s'], 0, 2).strip()} rad/s, "
                    f"|wheel vel|max={fmt(diag['max_wheel_vel_rad_s'], 0, 2).strip()} rad/s, "
                    f"|drift|max={fmt(diag['max_drift_rad'], 0, 2).strip()} rad"
                )
                print(
                    "  sensors/power: "
                    f"IMU disagree max={fmt(diag['max_imu_disagreement_deg'], 0, 2).strip()} deg, "
                    f"accel norm={fmt(diag['accel_norm_min_g'], 0, 2).strip()}.."
                    f"{fmt(diag['accel_norm_max_g'], 0, 2).strip()} g, "
                    f"bus min={fmt(diag['min_bus_voltage'], 0, 2).strip()} V, "
                    f"current max={fmt(diag['max_total_current'], 0, 2).strip()} A, "
                    f"rear torque max={fmt(diag['max_rear_torque_nm'], 0, 2).strip()} Nm"
                )
                print(
                    "  events: "
                    f"markers={diag['markers']}, saturated rows={diag['saturated_rows']}, "
                    f"SP-clamped rows={diag['sp_clamped_rows']}, "
                    f"arm active rows={diag['arm_active_rows']}, "
                    f"emergency-arm rows={diag['emergency_arm_rows']}"
                )
                if config:
                    print(
                        "  capture: "
                        f"schema={config.get('telemetry_schema', 'legacy')}, "
                        f"checksum={config.get('checksum_valid', 'n/a')}, "
                        f"end={config.get('end_reason', 'unknown')}, "
                        f"test_note={config.get('test_note', 'none')}"
                    )

    if args.plot:
        out_dir = Path(args.plot_dir)
        written = [out for path in paths if (out := plot_run(path, out_dir)) is not None]
        if written:
            print(f"\nWrote {len(written)} plot(s) to {out_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
