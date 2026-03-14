#!/bin/bash
# Launch the vision pipeline with camera settings from .env.
#
# Usage:
#   ./scripts/launch_vision.sh                    # uses CAMERA_PROFILE from .env
#   ./scripts/launch_vision.sh --test             # test mode: no tuning GUI, dummy servo
#   ./scripts/launch_vision.sh backend:=opencv    # override individual args
#
# To switch profiles, edit .env and set CAMERA_PROFILE=rpicam or CAMERA_PROFILE=webcam.

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ENV_FILE="${SCRIPT_DIR}/../.env"

if [ -f "${ENV_FILE}" ]; then
    set -a
    # shellcheck source=/dev/null
    source "${ENV_FILE}"
    set +a
    echo "[launch_vision] Loaded .env — CAMERA_PROFILE=${CAMERA_PROFILE}"
else
    echo "[launch_vision] No .env found at ${ENV_FILE}, using launch defaults"
fi

source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/../ros2_workspaces/install/setup.bash"

# --test flag: disable tuning GUI, enable dummy servo
EXTRA_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--test" ]; then
        EXTRA_ARGS+=("debug_filter:=none" "dummy_servo:=true")
    else
        EXTRA_ARGS+=("$arg")
    fi
done

exec ros2 launch secbot_vision vision.launch.py "${EXTRA_ARGS[@]}"

