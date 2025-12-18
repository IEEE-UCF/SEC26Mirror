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

echo "ℹ️ Ensuring PlatformIO platforms are installed (teensy, espressif32)"
pio platform install teensy || true
pio platform install espressif32 || true

# 2. Flash the Teensy
TEENSY_HEX="$WS_DIR/.pio/build/robot/firmware.hex"
if [[ -f "$TEENSY_HEX" ]]; then
    echo "ℹ️ Using prebuilt Teensy firmware at $TEENSY_HEX"
    # Try direct upload via teensy_loader_cli to bypass build on ARM
    CLI="$HOME/.platformio/packages/tool-teensy/teensy_loader_cli"
    if [[ -x "$CLI" ]]; then
        echo "▶️ Flashing via teensy_loader_cli (TEENSY41)"
        if "$CLI" -mmcu=TEENSY41 -v -w "$TEENSY_HEX"; then
            echo "--------------------------------"
            echo "✅ Teensy flashed successfully"
            echo "--------------------------------"
        else
            echo "--------------------------------"
            echo "❌ Error: teensy_loader_cli failed; falling back to PlatformIO upload"
            echo "--------------------------------"
            if ! pio run -t upload -e robot; then
                echo "❌ Error: Failed to flash Teensy"
                exit 1
            fi
        fi
    else
        echo "ℹ️ teensy_loader_cli not found; using PlatformIO upload"
        if ! pio run -t upload -e robot; then
            echo "❌ Error: Failed to flash Teensy"
            exit 1
        fi
    fi
else
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
fi
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
ESP32_BIN="$WS_DIR/.pio/build/robotcomms/firmware.bin"
ESP32_ELF="$WS_DIR/.pio/build/robotcomms/firmware.elf"
if [[ -f "$ESP32_BIN" ]]; then
    echo "ℹ️ Using prebuilt Comm ESP32 firmware at $ESP32_BIN"
    # Detect a likely serial port
    PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1 || true)
    if [[ -z "$PORT" ]]; then
        echo "❌ Error: No serial port found for ESP32"
        echo "🔁 Falling back to PlatformIO upload"
        if ! pio run -t upload -e robotcomms; then
            echo "❌ Error: Failed to flash Communication ESP32"
            exit 1
        fi
    else
        ESTOOL="$HOME/.platformio/packages/tool-esptoolpy/esptool.py"
        if [[ -f "$ESTOOL" ]]; then
            echo "▶️ Flashing via esptool.py on $PORT (offset 0x10000)"
            if python3 "$ESTOOL" --chip esp32 --port "$PORT" --baud 921600 write_flash -z 0x10000 "$ESP32_BIN"; then
                echo "--------------------------------"
                echo "✅ Communication ESP32 flashed successfully"
                echo "--------------------------------"
            else
                echo "--------------------------------"
                echo "❌ Error: esptool.py flash failed; falling back to PlatformIO upload"
                echo "--------------------------------"
                if ! pio run -t upload -e robotcomms; then
                    echo "❌ Error: Failed to flash Communication ESP32"
                    exit 1
                fi
            fi
        else
            echo "ℹ️ esptool.py not found; using PlatformIO upload"
            if ! pio run -t upload -e robotcomms; then
                echo "❌ Error: Failed to flash Communication ESP32"
                exit 1
            fi
        fi
    fi
else
    # If only ELF exists or nothing staged, use PlatformIO upload
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
fi
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
