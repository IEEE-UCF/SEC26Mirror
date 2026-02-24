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

# Ensure PlatformIO CLI and required platforms/tools are available
if ! command -v pio >/dev/null 2>&1; then
    echo "ℹ️ PlatformIO CLI not found; installing via pip"
    if command -v python3 >/dev/null 2>&1; then
        python3 -m pip install --user platformio
        export PATH="$HOME/.local/bin:$PATH"
    else
        echo "❌ Error: python3 not available to install PlatformIO"
        exit 1
    fi
fi

echo "ℹ️ Ensuring PlatformIO platforms are installed (teensy)"
pio platform install teensy || true

# 2. Flash the Teensy
TEENSY_HEX="$WS_DIR/.pio/build/robot/firmware.hex"
TEENSY_SUCCESS=false

if [[ -f "$TEENSY_HEX" ]]; then
    echo "ℹ️ Using prebuilt Teensy firmware at $TEENSY_HEX"
    # Try direct upload via teensy_loader_cli to bypass build on ARM
    CLI="$HOME/.platformio/packages/tool-teensy/teensy_loader_cli"
    if [[ -x "$CLI" ]]; then
        echo "▶️ Flashing via teensy_loader_cli (TEENSY41)"
        for attempt in {1..5}; do
            echo "🔄 Teensy flash attempt $attempt/3"
            if timeout 20 "$CLI" -mmcu=TEENSY41 -v -w -s "$TEENSY_HEX"; then
                echo "--------------------------------"
                echo "✅ Teensy flashed successfully on attempt $attempt"
                echo "--------------------------------"
                TEENSY_SUCCESS=true
                break
            else
                echo "⚠️ Teensy flash attempt $attempt failed"
                sleep 1
            fi
        done
        
        if [[ "$TEENSY_SUCCESS" == "false" ]]; then
            echo "--------------------------------"
            echo "❌ Error: Failed to flash Teensy after 3 attempts"
            echo "--------------------------------"
        fi
    else
        echo "ℹ️ teensy_loader_cli not found; using PlatformIO upload"
        for attempt in {1..5}; do
            echo "🔄 Teensy flash attempt $attempt/3"
            if timeout 20 pio run -t upload -e robot; then
                echo "--------------------------------"
                echo "✅ Teensy flashed successfully on attempt $attempt"
                echo "--------------------------------"
                TEENSY_SUCCESS=true
                break
            else
                echo "⚠️ Teensy flash attempt $attempt failed"
                sleep 1
            fi
        done
        
        if [[ "$TEENSY_SUCCESS" == "false" ]]; then
            echo "--------------------------------"
            echo "❌ Error: Failed to flash Teensy after 3 attempts"
            echo "--------------------------------"
        fi
    fi
else
    echo "ℹ️ Prebuilt Teensy firmware not found; building and uploading"
    for attempt in {1..5}; do
        echo "🔄 Teensy flash attempt $attempt/3"
        if timeout 20 pio run -t upload -e robot; then
            echo "--------------------------------"
            echo "✅ Teensy flashed successfully on attempt $attempt"
            echo "--------------------------------"
            TEENSY_SUCCESS=true
            break
        else
            echo "⚠️ Teensy flash attempt $attempt failed"
            sleep 1
        fi
    done
    
    if [[ "$TEENSY_SUCCESS" == "false" ]]; then
        echo "--------------------------------"
        echo "❌ Error: Failed to flash Teensy after 3 attempts"
        echo "--------------------------------"
    fi
fi

# 3. Flash Summary
echo "================================"
echo "📊 Flash Summary:"
echo "  Teensy: $([ "$TEENSY_SUCCESS" == "true" ] && echo "✅ SUCCESS" || echo "❌ FAILED")"
echo "================================"

if [[ "$TEENSY_SUCCESS" == "false" ]]; then
    echo "❌ Teensy failed to flash"
    exit 1
fi

echo "✅ Teensy flashed successfully"
exit 0
