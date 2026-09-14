#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"
mkdir -p output
PYTHON_BIN=python3
if [ -x .venv/bin/python ]; then PYTHON_BIN=.venv/bin/python; fi
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_balance_native.cpp -o output/test_balance_native
output/test_balance_native
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_motor_setup.cpp src/motor_manager.cpp -o output/test_motor_setup
output/test_motor_setup
"$PYTHON_BIN" -m unittest discover -s tests -v
"$PYTHON_BIN" -m py_compile scripts/*.py
bash -n scripts/*.sh
git diff --check
./scripts/build.sh
