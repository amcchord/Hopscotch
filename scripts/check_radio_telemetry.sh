#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p output
clang++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Isrc tests/test_radio_status.cpp src/crsf.cpp -o output/test_radio_status
output/test_radio_status
luac -p radio/SCRIPTS/TELEMETRY/hop.lua
lua tests/radio/test_hop.lua
lua tests/radio/test_hop_logging.lua
