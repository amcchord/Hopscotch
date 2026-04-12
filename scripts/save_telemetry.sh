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
RAW_TMP=$(mktemp)

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

with open('$RAW_TMP', 'w') as f:
    for l in lines:
        f.write(l + '\n')

print(f'Received {len(lines)} raw lines')
"

if [ ! -s "$RAW_TMP" ]; then
    echo "ERROR: No data received from device"
    rm -f "$RAW_TMP"
    exit 1
fi

grep -E '^(#|t_ms,|[0-9])' "$RAW_TMP" > "$OUTFILE" || true
rm -f "$RAW_TMP"

SAMPLE_COUNT=$(grep -c '^[0-9]' "$OUTFILE" || echo "0")
echo ""
echo "Saved: $OUTFILE"
echo "Samples: $SAMPLE_COUNT"

if [ "$SAMPLE_COUNT" -gt 0 ]; then
    FIRST_T=$(head -n 1 <(grep '^[0-9]' "$OUTFILE") | cut -d, -f1)
    LAST_T=$(tail -n 1 "$OUTFILE" | cut -d, -f1)
    if [ -n "$FIRST_T" ] && [ -n "$LAST_T" ]; then
        DURATION_MS=$((LAST_T - FIRST_T))
        echo "Duration: ${DURATION_MS}ms ($(echo "scale=1; $DURATION_MS / 1000" | bc)s)"
    fi
fi

CONFIG_LINES=$(grep -c '^#' "$OUTFILE" || echo "0")
if [ "$CONFIG_LINES" -gt 0 ]; then
    echo "Config: $CONFIG_LINES parameters recorded"
fi

ANALYZE="$SCRIPT_DIR/analyze_balance_logs.py"
if [ -x "$ANALYZE" ] || [ -f "$ANALYZE" ]; then
    echo ""
    echo "--- Analysis ---"
    python3 "$ANALYZE" "$OUTFILE" --details 2>/dev/null || true
fi

echo ""
echo "Done."
