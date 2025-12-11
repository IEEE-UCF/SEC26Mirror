#!/bin/bash

set -euo pipefail

# 1. Define Workspace Path
WS_DIR="/home/ubuntu/mcu_workspaces/sec26mcu"

# 1a. Ensure we're in the PlatformIO project root
if [[ ! -f "$WS_DIR/platformio.ini" ]]; then
    echo "--------------------------------"
    echo "❌ Error: platformio.ini not found in $WS_DIR"
    echo "   Check that the volume is mounted: ./mcu_ws -> $WS_DIR"
    echo "--------------------------------"
    exit 1
fi

cd "$WS_DIR"

# 2. Flash the Teensy
if pio run -t upload -e robot; then
    echo "--------------------------------"
    echo "✅ Teensy flashed successfully"
    echo "--------------------------------"
else
    echo "--------------------------------"
    echo "❌ Error: Failed to flash Teensy"
    echo "--------------------------------"
    exit 1
fi

# 3. Flash the CommESP32
if pio run -t upload -e robotcomms; then
    echo "--------------------------------"
    echo "✅ Communication ESP32 flashed successfully"
    echo "--------------------------------"
else
    echo "--------------------------------"
    echo "❌ Error: Failed to flash Communication ESP32"
    echo "--------------------------------"
    exit 1
fi

exit 0
