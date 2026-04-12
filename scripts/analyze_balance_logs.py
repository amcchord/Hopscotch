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
        if not line.startswith("#"):
            break
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


def vel_offset_series(rows: Iterable[dict[str, str]]) -> list[float]:
    result: list[float] = []
    for row in rows:
        if "vel_offset" in row:
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


def fmt(value: float, width: int = 5, precision: int = 2) -> str:
    if math.isnan(value):
        return " " * (width - 2) + "--"
    return f"{value:{width}.{precision}f}"


def summarize(path: Path) -> dict[str, object] | None:
    rows = load_rows(path)
    if not rows:
        return None

    balance_rows = [row for row in rows if row.get("state") == "2"]
    tip_rows = [row for row in rows if row.get("state") == "1"]
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
            "final_drift": math.nan,
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
    has_vel_offset = "vel_offset" in balance_rows[0]
    if not has_vel_offset and "vel_integ" not in balance_rows[0]:
        notes.append("legacy schema")
    if not math.isnan(pre_rate) and pre_rate > 4.0:
        notes.append("arm return before settle")
    if not math.isnan(pre_cmd) and pre_cmd > 1.0:
        notes.append("high cmd at return")
    if not notes:
        notes.append("clean capture")

    drift_key = "meas_drift" if "meas_drift" in balance_rows[0] else ""

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
        "final_drift": mean(as_float(row, drift_key) for row in final_window) if drift_key else math.nan,
        "note": ", ".join(notes),
    }


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
        f"{'final_sp':>8} {'vel_off':>8} {'drift':>8}  note"
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
            if config:
                gains = []
                for key in ("inner_kp", "inner_kd", "pos_kp", "pos_ki", "pos_kd", "vel_kp", "vel_kd"):
                    if key in config:
                        gains.append(f"{key}={config[key]}")
                if gains:
                    print(f"  config: {', '.join(gains)}")
                extras = []
                for key in ("pos_shift_max", "vel_max", "base_sp_fwd", "base_sp_tip"):
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
                f"vel_off={fmt(summary['final_vel_off'], 0, 2).strip()} rad/s, "
                f"drift={fmt(summary['final_drift'], 0, 2).strip()} rad"
            )
            print(f"  note: {summary['note']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
