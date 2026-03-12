#!/bin/bash
# Transfer compiled Teensy firmware to Raspberry Pi for flashing.
# Usage: ./transfer_firmware.sh [options] [environment]
#   environment: PlatformIO env name (default: robot)
#
# Options:
#   --flash    Also flash the Teensy on the Pi after transfer
#   --build    Build firmware in the container before transferring
#
# Run from the HOST machine (not inside Docker).
# The script extracts firmware from the Docker build volume, then SCPs to the Pi.

set -euo pipefail

# --- Defaults ---
ENV="robot"
DO_FLASH=false
DO_BUILD=false
PI_HOST="192.168.4.1"
PI_USER="ieee"
PI_DIR="/home/ieee/firmware"

# --- Parse args ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --flash) DO_FLASH=true; shift ;;
        --build) DO_BUILD=true; shift ;;
        --host)  PI_HOST="$2"; shift 2 ;;
        --user)  PI_USER="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--build] [--flash] [--host IP] [--user USER] [environment]"
            echo "  environment   PlatformIO env name (default: robot)"
            echo "  --build       Build firmware in Docker before transfer"
            echo "  --flash       Flash Teensy on Pi after transfer"
            echo "  --host IP     Pi IP address (default: 192.168.4.1)"
            echo "  --user USER   Pi SSH user (default: ieee)"
            exit 0 ;;
        -*) echo "Unknown option: $1"; exit 1 ;;
        *)  ENV="$1"; shift ;;
    esac
done

# --- Paths inside the Docker container ---
CONTAINER_WS="/home/ubuntu/mcu_workspaces/sec26mcu"
CONTAINER_HEX="$CONTAINER_WS/.pio/build/$ENV/firmware.hex"

# --- Find repo root (script is in scripts/) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# --- Get the running container ID ---
CONTAINER=$(docker compose -f "$REPO_ROOT/docker-compose.yml" ps -q devcontainer 2>/dev/null || true)
if [[ -z "$CONTAINER" ]]; then
    echo "Error: Docker container 'devcontainer' is not running."
    echo "Start it with: docker compose up -d"
    exit 1
fi
echo "Using container: ${CONTAINER:0:12}"

# --- Optionally build firmware first ---
if [[ "$DO_BUILD" == true ]]; then
    echo "Building $ENV firmware in container..."
    docker exec "$CONTAINER" bash -c "cd $CONTAINER_WS && pio run -e $ENV"
fi

# --- Check firmware exists in container ---
if ! docker exec "$CONTAINER" test -f "$CONTAINER_HEX"; then
    echo "Error: $CONTAINER_HEX not found in container."
    echo "Build first with: $0 --build $ENV"
    echo "  or: docker compose exec devcontainer pio run -e $ENV"
    exit 1
fi

# --- Extract firmware from container ---
LOCAL_TMP="$(mktemp -d)"
LOCAL_HEX="$LOCAL_TMP/firmware-${ENV}.hex"
cleanup() { rm -rf "$LOCAL_TMP"; }
trap cleanup EXIT

echo "Extracting $ENV firmware from Docker container..."
docker cp "$CONTAINER:$CONTAINER_HEX" "$LOCAL_HEX"
echo "Extracted: $(wc -c < "$LOCAL_HEX") bytes"

# --- Transfer to Pi ---
echo "Transferring to $PI_USER@$PI_HOST:$PI_DIR/"
ssh "$PI_USER@$PI_HOST" "mkdir -p $PI_DIR"
scp "$LOCAL_HEX" "$PI_USER@$PI_HOST:$PI_DIR/firmware-${ENV}.hex"
echo "Transferred: firmware-${ENV}.hex"

# --- Flash on Pi ---
FLASH_CMD="teensy_loader_cli -mmcu=TEENSY41 -v -w -s $PI_DIR/firmware-${ENV}.hex"
FLASH_TIMEOUT=10

if [[ "$DO_FLASH" == true ]]; then
    echo "Flashing Teensy on Pi (timeout ${FLASH_TIMEOUT}s per attempt)..."
    FLASH_OK=false
    for attempt in 1 2 3; do
        echo "Flash attempt $attempt/3..."
        if ssh "$PI_USER@$PI_HOST" "timeout $FLASH_TIMEOUT $FLASH_CMD" 2>&1; then
            echo "Flash complete."
            FLASH_OK=true
            break
        else
            echo "Attempt $attempt failed (Teensy may not have entered bootloader)."
            if [[ $attempt -lt 3 ]]; then
                echo "Retrying in 2s... (try pressing the Teensy reset button)"
                sleep 2
            fi
        fi
    done
    if [[ "$FLASH_OK" == false ]]; then
        echo ""
        echo "Error: Failed to flash after 3 attempts."
        echo "Try pressing the Teensy reset button, then run on Pi:"
        echo "  $FLASH_CMD"
        exit 1
    fi
else
    echo ""
    echo "Done. Flash on Pi with:"
    echo "  ssh $PI_USER@$PI_HOST \"$FLASH_CMD\""
fi
