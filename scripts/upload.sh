#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

if command -v pio >/dev/null 2>&1; then
    PIO_BIN=$(command -v pio)
elif [ -x "$PROJECT_DIR/.venv-pio/bin/pio" ]; then
    PIO_BIN="$PROJECT_DIR/.venv-pio/bin/pio"
elif [ -x "$PROJECT_DIR/.venv/bin/pio" ]; then
    PIO_BIN="$PROJECT_DIR/.venv/bin/pio"
else
    echo "ERROR: PlatformIO not found (install pio or create .venv-pio)"
    exit 1
fi

echo "Building and uploading firmware..."
"$PIO_BIN" run -e m5stack-atoms3r --target upload
echo "Upload complete."
