#!/usr/bin/env bash
# Offline integration validation. Does not contact or operate the robot.
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHON_BIN=python3
if [ -x .venv/bin/python ]; then PYTHON_BIN=.venv/bin/python; fi
./scripts/check_balance_candidate.sh
for script in scripts/*.sh; do bash -n "$script"; done
./scripts/check_radio_telemetry.sh
node tests/test_network_dashboard.js
"$PYTHON_BIN" scripts/check_ota_transport.py
echo "Project validation passed (no device operations)."
