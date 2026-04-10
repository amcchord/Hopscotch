#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/telemetry_logs"

mkdir -p "$LOG_DIR"

PORT=""
for p in /dev/cu.usbmodem*; do
    if [ -e "$p" ]; then
        PORT="$p"
        break
    fi
done

if [ -z "$PORT" ]; then
    echo "ERROR: No USB serial device found"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTFILE="$LOG_DIR/bal_${TIMESTAMP}.csv"

echo "Downloading telemetry from $PORT..."
python3 -c "
import serial, time, sys

port = serial.Serial('$PORT', 115200, timeout=2)
time.sleep(0.5)
port.reset_input_buffer()

port.write(b'bal log\r\n')
time.sleep(0.5)

lines = []
deadline = time.time() + 30
while time.time() < deadline:
    raw = port.readline()
    if not raw:
        if lines and any('End of log' in l for l in lines[-5:]):
            break
        continue
    line = raw.decode('utf-8', errors='replace').rstrip()
    lines.append(line)
    if 'End of log' in line:
        break

port.close()

with open('$OUTFILE', 'w') as f:
    for l in lines:
        f.write(l + '\n')

print(f'Saved {len(lines)} lines to $OUTFILE')
"

echo "Done: $OUTFILE"
