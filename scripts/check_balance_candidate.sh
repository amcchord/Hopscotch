#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"
mkdir -p output
PYTHON_BIN=python3
if [ -x .venv/bin/python ]; then PYTHON_BIN=.venv/bin/python; fi
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_balance_native.cpp -o output/test_balance_native
output/test_balance_native
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_startup_recovery.cpp -o output/test_startup_recovery
output/test_startup_recovery
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_balance_pilot.cpp -o output/test_balance_pilot
output/test_balance_pilot
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_balance_drive.cpp -o output/test_balance_drive
output/test_balance_drive
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_motor_setup.cpp src/motor_manager.cpp -o output/test_motor_setup
output/test_motor_setup
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_crsf_native.cpp src/crsf.cpp -o output/test_crsf_native
output/test_crsf_native
clang++ -std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter -Itests/stubs -Isrc tests/test_motor_feedback.cpp src/robstride.cpp src/motor_manager.cpp -o output/test_motor_feedback
output/test_motor_feedback
clang++ -std=c++17 -Wall -Wextra -Werror -pthread -Isrc tests/test_network_safety.cpp -o output/test_network_safety
output/test_network_safety
"$PYTHON_BIN" -m unittest discover -s tests -v
"$PYTHON_BIN" -m py_compile scripts/*.py
bash -n scripts/*.sh
git diff --check -- . ':!*.serial'
./scripts/build.sh
