#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/telemetry_logs"

mkdir -p "$LOG_DIR"

PORT=""
LABEL=""
TIMEOUT_SEC=240

while [ "$#" -gt 0 ]; do
    case "$1" in
        --port)
            PORT="${2:?--port requires a device path}"
            shift 2
            ;;
        --label)
            LABEL="${2:?--label requires text}"
            shift 2
            ;;
        --timeout)
            TIMEOUT_SEC="${2:?--timeout requires seconds}"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--port /dev/cu.usbmodem…] [--label name] [--timeout seconds]"
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1"
            exit 2
            ;;
    esac
done

if [ -z "$PORT" ]; then
    for p in /dev/cu.usbmodem* /dev/cu.usbserial*; do
        if [ -e "$p" ]; then
            PORT="$p"
            break
        fi
    done
fi

if [ -z "$PORT" ]; then
    echo "ERROR: No USB serial device found"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
if [ -n "$LABEL" ]; then
    SAFE_LABEL=$(printf '%s' "$LABEL" | tr '[:upper:]' '[:lower:]' | tr -cs 'a-z0-9_-' '_' | sed 's/^_*//;s/_*$//')
    OUTFILE="$LOG_DIR/bal_${TIMESTAMP}_${SAFE_LABEL:-run}.csv"
else
    OUTFILE="$LOG_DIR/bal_${TIMESTAMP}.csv"
fi
RAW_TMP=$(mktemp)
OUT_TMP=$(mktemp "$LOG_DIR/.bal_download.XXXXXX")
trap 'rm -f "$RAW_TMP" "$OUT_TMP"' EXIT

echo "Downloading telemetry from $PORT..."
python3 - "$PORT" "$RAW_TMP" "$TIMEOUT_SEC" <<'PY'
import serial, time, sys

device, output, timeout_text = sys.argv[1:4]
timeout = float(timeout_text)
port = serial.Serial(device, 115200, timeout=2)
time.sleep(0.5)
port.reset_input_buffer()

port.write(b'bal log\r\n')
time.sleep(0.5)

lines = []
deadline = time.time() + timeout
complete = False
while time.time() < deadline:
    raw = port.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='replace').rstrip()
    lines.append(line)
    if 'End of log' in line:
        complete = True
        break

port.close()

with open(output, 'w') as f:
    for l in lines:
        f.write(l + '\n')

print(f'Received {len(lines)} raw lines')
if not complete:
    print(f'ERROR: log transfer did not complete within {timeout:g}s', file=sys.stderr)
    raise SystemExit(3)
PY

if [ ! -s "$RAW_TMP" ]; then
    echo "ERROR: No data received from device"
    exit 1
fi

if grep -q 'REFUSED while balance mode is active' "$RAW_TMP"; then
    echo "ERROR: Robot is still in balance mode. Disengage it, wait for the idle save, and retry."
    exit 1
fi

grep -E '^(#|t_ms,|[0-9]+,)' "$RAW_TMP" > "$OUT_TMP" || true

SAMPLE_COUNT=$(grep -cE '^[0-9]+,' "$OUT_TMP" || true)
EXPECTED_COUNT=$(sed -n 's/^# sample_count=//p' "$OUT_TMP" | tail -n 1)
SCHEMA=$(sed -n 's/^# telemetry_schema=//p' "$OUT_TMP" | tail -n 1)
CHECKSUM_VALID=$(sed -n 's/^# checksum_valid=//p' "$OUT_TMP" | tail -n 1)

if [ "$SAMPLE_COUNT" -eq 0 ] || ! grep -q '^t_ms,' "$OUT_TMP"; then
    echo "ERROR: Download contained no parseable telemetry samples"
    exit 1
fi

if [ -n "$EXPECTED_COUNT" ] && [ "$SAMPLE_COUNT" -ne "$EXPECTED_COUNT" ]; then
    echo "ERROR: Truncated transfer: expected $EXPECTED_COUNT samples, received $SAMPLE_COUNT"
    exit 1
fi

if [ "${SCHEMA:-1}" -ge 2 ] && [ "$CHECKSUM_VALID" != "1" ]; then
    echo "ERROR: Device reported an invalid telemetry checksum"
    exit 1
fi

mv "$OUT_TMP" "$OUTFILE"

echo ""
echo "Saved: $OUTFILE"
echo "Samples: $SAMPLE_COUNT (schema ${SCHEMA:-legacy}, checksum ${CHECKSUM_VALID:-n/a})"

if [ "$SAMPLE_COUNT" -gt 0 ]; then
    FIRST_T=$(head -n 1 <(grep '^[0-9]' "$OUTFILE") | cut -d, -f1)
    LAST_T=$(tail -n 1 "$OUTFILE" | cut -d, -f1)
    if [ -n "$FIRST_T" ] && [ -n "$LAST_T" ]; then
        DURATION_MS=$((LAST_T - FIRST_T))
        echo "Duration: ${DURATION_MS}ms ($(echo "scale=1; $DURATION_MS / 1000" | bc)s)"
    fi
fi

CONFIG_LINES=$(grep -c '^#' "$OUTFILE" || true)
if [ "$CONFIG_LINES" -gt 0 ]; then
    echo "Config: $CONFIG_LINES parameters recorded"
fi

ANALYZE="$SCRIPT_DIR/analyze_balance_logs.py"
if [ -x "$ANALYZE" ] || [ -f "$ANALYZE" ]; then
    echo ""
    echo "--- Analysis ---"
    python3 "$ANALYZE" "$OUTFILE" --details || true
fi

echo ""
echo "Done."
