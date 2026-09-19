#!/usr/bin/env python3
"""Validate the exact USB transfer, then emit a clean, analysis-ready CSV."""
from __future__ import annotations
import argparse
import csv
import io
import math
from pathlib import Path


def fnv1a(data: bytes) -> int:
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def validate(raw: bytes) -> tuple[bytes, int]:
    if b'[Balance] --- End of log ---' not in raw:
        raise ValueError('Incomplete transfer: end marker missing')
    lines = raw.splitlines(keepends=True)
    if any(b'transport_checksum=fnv1a32' in line for line in lines):
        start = next((i for i, line in enumerate(lines) if line.rstrip() == b'# === BALANCE CONFIG ==='), None)
        end = next((i for i, line in enumerate(lines) if line.startswith(b'# transport_fnv1a=')), None)
        if start is None or end is None or end <= start:
            raise ValueError('Missing transport checksum or start marker')
        expected = int(lines[end].split(b'=', 1)[1].strip(), 16)
        if fnv1a(b''.join(lines[start:end])) != expected:
            raise ValueError('USB transport checksum mismatch; retry download')
    clean_lines = [line.rstrip(b'\r\n') for line in lines
                   if line.startswith((b'#', b't_ms,')) or line[:1].isdigit()]
    config = {}
    for line in clean_lines:
        if line.startswith(b'#') and b'=' in line:
            key, value = line[1:].strip().split(b'=', 1)
            config[key.decode()] = value.decode()
    if int(config.get('telemetry_features', '0')) & 1 and config.get('transport_checksum') != 'fnv1a32':
        raise ValueError('Candidate log is missing transport protection')
    header_i = next((i for i, line in enumerate(clean_lines) if line.startswith(b't_ms,')), None)
    if header_i is None:
        raise ValueError('CSV header missing')
    csv_lines = [line for line in clean_lines[header_i:] if not line.startswith(b'#')]
    parsed = list(csv.reader(io.StringIO(b'\n'.join(csv_lines).decode('ascii'))))
    header, *rows = parsed
    if not rows or len(set(header)) != len(header):
        raise ValueError('No samples or duplicate column names')
    previous = -1
    for index, row in enumerate(rows, 1):
        if len(row) != len(header):
            raise ValueError(f'Row {index} has {len(row)} fields; expected {len(header)}')
        for key, value in zip(header, row):
            if key == 'imu_age_ms' and value == '' and not (int(config.get('telemetry_features', '0')) & 1):
                continue  # pre-extension v2 file: unknown, not a fabricated zero
            if (key in ('pilot_forward','pilot_steering','pilot_turn','pilot_flags') and value == ''
                    and int(config.get('telemetry_schema','1')) < 3
                    and not (int(config.get('telemetry_features','0')) & 64)):
                continue  # schema-2 log exported by new firmware: pilot intent unknown
            if (key == 'pilot_arm' and value == ''
                    and int(config.get('telemetry_schema','1')) < 4
                    and not (int(config.get('telemetry_features','0')) & 256)):
                continue  # old log: no planned-arm measurement
            if not math.isfinite(float(value)):
                raise ValueError(f'Row {index} has nonfinite {key}')
        timestamp = int(row[0])
        if timestamp < previous:
            raise ValueError(f'Row {index} timestamp runs backward')
        previous = timestamp
    expected_count = config.get('sample_count')
    if expected_count is not None and len(rows) != int(expected_count):
        raise ValueError(f'Truncated transfer: expected {expected_count} rows, received {len(rows)}')
    if int(config.get('telemetry_schema', '1')) >= 2:
        if config.get('checksum_valid') != '1' or expected_count is None:
            raise ValueError('Device file checksum or sample count is missing/invalid')
    # Keep metadata before the CSV header so downstream CSV tools see only rows.
    metadata = [line for line in clean_lines if line.startswith(b'#')]
    return b'\n'.join(metadata + csv_lines) + b'\n', len(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    try:
        output, count = validate(args.raw.read_bytes())
    except (ValueError, UnicodeError) as exc:
        raise SystemExit(f'ERROR: {exc}') from exc
    args.output.write_bytes(output)
    print(f'Validated {count} samples (file integrity, row structure, USB checksum when available)')

if __name__ == '__main__':
    main()
