#!/bin/bash
# Launch the vision pipeline on the Raspberry Pi with:
#   - rpicam TCP backend (assumes rpicam-stream.service is running on the Pi host)
#   - ROS2 web_video_server for browser-based image viewing / HSV tuning
#   - vision.launch.py with tuning/GUI disabled (tune from a separate machine)
#
# Usage:
#   ./scripts/launch_vision_rpi.sh                    # default: no GUI, tuning off
#   ./scripts/launch_vision_rpi.sh --test             # test mode: dummy servo
#   ./scripts/launch_vision_rpi.sh backend:=picamera2 # override individual args
#
# Prerequisites:
#   On the Pi HOST (outside Docker), enable the camera stream:
#     sudo systemctl enable --now rpicam-stream
#
# Then open a browser to http://<pi-ip>:8080 to view camera topics.

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ENV_FILE="${SCRIPT_DIR}/../.env"

if [ -f "${ENV_FILE}" ]; then
    set -a
    # shellcheck source=/dev/null
    source "${ENV_FILE}"
    set +a
fi

# Force RPi camera profile defaults
export CAMERA_PROFILE=rpicam
export RPICAM_TCP_HOST="${RPICAM_TCP_HOST:-127.0.0.1}"
export RPICAM_TCP_PORT="${RPICAM_TCP_PORT:-8554}"

echo "[launch_vision_rpi] RPi camera: tcp://${RPICAM_TCP_HOST}:${RPICAM_TCP_PORT}"

source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/../ros2_workspaces/install/setup.bash"

# Start web_video_server in background (serves camera topics at http://<ip>:8080)
echo "[launch_vision_rpi] Starting web_video_server on port 8080..."
ros2 run web_video_server web_video_server --ros-args -p port:=8080 &
WEB_VIDEO_PID=$!

cleanup() {
    echo "[launch_vision_rpi] Shutting down web_video_server (PID ${WEB_VIDEO_PID})..."
    kill "${WEB_VIDEO_PID}" 2>/dev/null || true
    wait "${WEB_VIDEO_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Parse args: --test flag maps to tuning-off + dummy servo
EXTRA_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--test" ]; then
        EXTRA_ARGS+=("debug_filter:=none" "dummy_servo:=true")
    else
        EXTRA_ARGS+=("$arg")
    fi
done

exec ros2 launch secbot_vision vision.launch.py \
    enable_slider_gui:=false \
    debug_filter:=none \
    "${EXTRA_ARGS[@]}"
