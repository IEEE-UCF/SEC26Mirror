#!/bin/sh
set -euxo pipefail

# 1) Chown volumes once using sentinel files
for d in \
  /ros2_build \
  /ros2_install \
  /ros2_log \
  /platformio_cache \
  /mcu_build_artifacts \
  /mcu_build_artifacts_microros \
  /mcu_lib_external \
; do
  echo "[bootstrap] Ensuring directory exists: $d"
  mkdir -p "$d"
  if [ -f "$d/.sec26_owner_set" ]; then
    echo "[bootstrap] Ownership already set for $d; skipping"
  else
    echo "[bootstrap] Chowning recursively to 1000:1000: $d"
    chown -R 1000:1000 "$d"
    echo "owner=1000:1000" > "$d/.sec26_owner_set"
  fi
done

# 2) Seed libs_external once using sentinel file
DATA=/mcu_lib_external
SEED=/seed

echo "[bootstrap] Preparing libs_external seed"
mkdir -p "$DATA"

if [ -f "$DATA/.sec26_libs_seeded" ]; then
  echo "[bootstrap] Sentinel found: $DATA/.sec26_libs_seeded — skipping seed"
  exit 0
fi

if [ ! -d "$SEED" ]; then
  echo "[bootstrap] ERROR: Seed directory not found: $SEED"
  exit 1
fi

if [ -z "$(ls -A "$SEED" 2>/dev/null)" ]; then
  echo "[bootstrap] ERROR: Seed directory is empty: $SEED"
  exit 1
fi

echo "[bootstrap] Copying from $SEED to $DATA"
cp -a "$SEED"/. "$DATA"/

echo "[bootstrap] Setting ownership to 1000:1000 on $DATA"
chown -R 1000:1000 "$DATA"

echo "seeded_from=$SEED" > "$DATA/.sec26_libs_seeded"

echo "[bootstrap] Completed successfully"