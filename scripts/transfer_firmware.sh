#!/bin/bash
# Transfer compiled Teensy firmware from Windows Docker container to Raspberry Pi.
# Usage: ./transfer_firmware.sh [environment]
#   environment: PlatformIO env name (default: robot)
#
# Run from inside the Docker container on the Windows build machine.
# The Pi then flashes via: teensy_loader_cli -mmcu=TEENSY41 -v -w -s firmware.hex

set -euo pipefail

ENV="${1:-robot}"
PI_HOST="192.168.4.1"
PI_USER="ieee"
PI_DIR="/home/ieee/firmware"
WS_DIR="/home/ubuntu/mcu_workspaces/sec26mcu"
HEX="$WS_DIR/.pio/build/$ENV/firmware.hex"

if [[ ! -f "$HEX" ]]; then
    echo "Error: $HEX not found. Build first: pio run -e $ENV"
    exit 1
fi

echo "Transferring $ENV firmware to $PI_USER@$PI_HOST:$PI_DIR/"
ssh "$PI_USER@$PI_HOST" "mkdir -p $PI_DIR"
scp "$HEX" "$PI_USER@$PI_HOST:$PI_DIR/firmware-${ENV}.hex"
echo "Done. Flash on Pi with:"
echo "  teensy_loader_cli -mmcu=TEENSY41 -v -w -s $PI_DIR/firmware-${ENV}.hex"
