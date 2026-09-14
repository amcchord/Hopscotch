#!/usr/bin/env python3
"""Request a disarmed USB log, retaining exact bytes even on interruption."""
import argparse
import time
from pathlib import Path


def receive(port, output, timeout, clock=time.monotonic):
    deadline = clock() + timeout
    pending = b''
    started = False
    received = 0
    while clock() < deadline:
        chunk = port.read(min(max(port.in_waiting, 1), 65536))
        if not chunk:
            continue
        output.write(chunk)
        output.flush()
        received += len(chunk)
        pending += chunk
        while b'\n' in pending:
            line, pending = pending.split(b'\n', 1)
            if line.startswith(b'# === BALANCE CONFIG ===') or line.startswith(b't_ms,'):
                started = True
            if b'[Balance] --- End of log ---' in line:
                return received
            if (b'REFUSED' in line or b'No log file' in line
                    or b'unsupported schema' in line or b'pending save' in line
                    or b'Log transfer FAILED' in line):
                raise ValueError(line.decode(errors='replace').strip())
            if started and line.startswith(b'[Loop]'):
                raise ValueError('Device resumed idle output before completing the log; retry download')
    raise TimeoutError(f'Log transfer did not complete within {timeout:g}s')


def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('output', type=Path)
    parser.add_argument('timeout', type=float)
    args = parser.parse_args()
    try:
        with args.output.open('wb') as output, serial.Serial(args.port, 115200, timeout=.1) as port:
            port.reset_input_buffer()
            port.write(b'bal log\r\n')
            # Read immediately, in chunks: no post-request sleep or byte-at-a-time readline.
            size = receive(port, output, args.timeout)
        print(f'Received {size} raw bytes')
    except (OSError, ValueError, serial.SerialException) as exc:
        raise SystemExit(f'ERROR: {exc}') from exc


if __name__ == '__main__':
    main()
