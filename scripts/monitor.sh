#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

echo "Starting serial monitor (115200 baud)..."
echo "Press Ctrl+C to exit."
pio device monitor -e m5stack-atoms3r
