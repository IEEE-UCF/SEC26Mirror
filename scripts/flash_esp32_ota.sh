#!/bin/bash
# Flash an ESP32 via ArduinoOTA with retries.
# Usage: flash_esp32_ota.sh <IP> <firmware.bin> [retries]
#
# Requires espota.py (shipped with Arduino/PlatformIO ESP32 platform).

set -euo pipefail

IP="${1:?Usage: flash_esp32_ota.sh <IP> <firmware.bin> [retries]}"
FIRMWARE="${2:?Usage: flash_esp32_ota.sh <IP> <firmware.bin> [retries]}"
MAX_RETRIES="${3:-3}"
OTA_PORT=3232

if [ ! -f "$FIRMWARE" ]; then
  echo "ERROR: Firmware file not found: $FIRMWARE"
  exit 1
fi

# Locate espota.py
ESPOTA=""
for candidate in \
  "$HOME/.platformio/packages/framework-arduinoespressif32/tools/espota.py" \
  "/home/ubuntu/.platformio/packages/framework-arduinoespressif32/tools/espota.py" \
  "$(which espota.py 2>/dev/null)"; do
  if [ -f "$candidate" ]; then
    ESPOTA="$candidate"
    break
  fi
done

if [ -z "$ESPOTA" ]; then
  echo "ERROR: espota.py not found. Install ESP32 Arduino framework."
  exit 1
fi

echo "Flashing $FIRMWARE to $IP:$OTA_PORT (max $MAX_RETRIES attempts)"

for attempt in $(seq 1 "$MAX_RETRIES"); do
  echo "  Attempt $attempt/$MAX_RETRIES..."
  if python3 "$ESPOTA" -i "$IP" -p "$OTA_PORT" -f "$FIRMWARE" --timeout 30 2>&1; then
    echo "  SUCCESS: OTA flash complete for $IP"
    exit 0
  fi
  echo "  FAILED: Retrying in 5s..."
  sleep 5
done

echo "ERROR: OTA flash failed after $MAX_RETRIES attempts for $IP"
exit 1
