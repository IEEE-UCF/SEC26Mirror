#!/bin/bash
# Remote deployment for SEC26 robot.
# Builds firmware on the HOST (in Docker), transfers to Pi, flashes there.
# ROS2 is built on the Pi (git pull + colcon build in Pi's Docker).
#
# Usage: ./deploy_remote.sh [options] [targets...]
#
# Targets (specify one or more):
#   robot                         Teensy main robot firmware
#   teensy-test-all-subsystems    Teensy integration test firmware
#   beacon1                       ESP32 UWB beacon 1  (OTA → 192.168.4.20)
#   beacon2                       ESP32 UWB beacon 2  (OTA → 192.168.4.21)
#   beacon3                       ESP32 UWB beacon 3  (OTA → 192.168.4.22)
#   minibot                       ESP32 minibot       (OTA → 192.168.4.24)
#   drone                         ESP32 drone         (OTA → 192.168.4.25)
#   ros                           ROS2 workspace (git pull on Pi + colcon build)
#   all-mcu                       All MCU targets (robot + beacons + minibot + drone)
#   all                           Everything (all-mcu + ros)
#
# Options:
#   --no-build          Skip building, use existing firmware in Docker volume
#   --no-flash          Build only, don't transfer or flash
#   --clean-microros    Clean micro-ROS before building (after mcu_msgs changes)
#   --branch BRANCH     Git branch for ROS2 pull on Pi (default: prod)
#   --host IP           Pi IP address (default: 192.168.4.1)
#   --user USER         Pi SSH user (default: ieee)
#   --help              Show this help
#
# Examples:
#   ./deploy_remote.sh robot                    # Build + flash robot Teensy
#   ./deploy_remote.sh beacon1 beacon2          # Build + OTA flash 2 beacons
#   ./deploy_remote.sh --no-build robot         # Flash robot with existing firmware
#   ./deploy_remote.sh ros                      # Pull + build ROS2 on Pi
#   ./deploy_remote.sh all                      # Full deploy everything
#   ./deploy_remote.sh robot ros                # Robot firmware + ROS2

set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────
PI_HOST="192.168.4.1"
PI_USER="ieee"
BRANCH="prod"
DO_BUILD=true
DO_FLASH=true
CLEAN_MICROROS=false

# Pi paths
PI_REPO="/home/ieee/SEC26"
PI_FIRMWARE_DIR="/home/ieee/firmware"
PI_SCRIPTS="$PI_REPO/scripts"

# Host Docker paths
CONTAINER_WS="/home/ubuntu/mcu_workspaces/sec26mcu"

# ── Device table ──────────────────────────────────────────────────────────
# Format: method:firmware_ext:ip (ip only for esp32)
declare -A DEVICE_INFO=(
    [robot]="teensy:hex:"
    [teensy-test-all-subsystems]="teensy:hex:"
    [beacon1]="esp32:bin:192.168.4.20"
    [beacon2]="esp32:bin:192.168.4.21"
    [beacon3]="esp32:bin:192.168.4.22"
    [minibot]="esp32:bin:192.168.4.24"
    [drone]="esp32:bin:192.168.4.25"
)

ALL_MCU_TARGETS=(robot beacon1 beacon2 beacon3 minibot drone)

# ── Parse args ────────────────────────────────────────────────────────────
TARGETS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)       DO_BUILD=false; shift ;;
        --no-flash)       DO_FLASH=false; shift ;;
        --clean-microros) CLEAN_MICROROS=true; shift ;;
        --branch)         BRANCH="$2"; shift 2 ;;
        --host)           PI_HOST="$2"; shift 2 ;;
        --user)           PI_USER="$2"; shift 2 ;;
        --help|-h)
            sed -n '/^# Usage:/,/^[^#]/p' "$0" | head -n -1 | sed 's/^# \?//'
            exit 0 ;;
        -*)
            echo "Unknown option: $1"; exit 1 ;;
        all)
            TARGETS+=("${ALL_MCU_TARGETS[@]}" "ros"); shift ;;
        all-mcu)
            TARGETS+=("${ALL_MCU_TARGETS[@]}"); shift ;;
        ros|robot|teensy-test-all-subsystems|beacon1|beacon2|beacon3|minibot|drone)
            TARGETS+=("$1"); shift ;;
        *)
            echo "Unknown target: $1"
            echo "Run with --help for usage"
            exit 1 ;;
    esac
done

if [[ ${#TARGETS[@]} -eq 0 ]]; then
    echo "No targets specified. Run with --help for usage."
    exit 1
fi

# Deduplicate targets
TARGETS=($(printf '%s\n' "${TARGETS[@]}" | awk '!seen[$0]++'))

PI="$PI_USER@$PI_HOST"
START_TIME=$(date +%s)

# ── Helpers ───────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ssh_pi() {
    ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$PI" "$@"
}

get_container_id() {
    docker compose -f "$REPO_ROOT/docker-compose.yml" ps -q devcontainer 2>/dev/null || true
}

docker_exec() {
    local container
    container=$(get_container_id)
    if [[ -z "$container" ]]; then
        echo "ERROR: Docker container 'devcontainer' is not running on host."
        echo "Start it with: docker compose up -d"
        exit 1
    fi
    docker exec "$container" bash -c "$*"
}

step() {
    echo ""
    echo "════════════════════════════════════════════════════"
    echo "  $1"
    echo "════════════════════════════════════════════════════"
}

# ── Collect MCU targets (non-ros) ─────────────────────────────────────────
MCU_TARGETS=()
DO_ROS=false
for t in "${TARGETS[@]}"; do
    if [[ "$t" == "ros" ]]; then
        DO_ROS=true
    else
        MCU_TARGETS+=("$t")
    fi
done

# ── Verify host Docker container ─────────────────────────────────────────
if [[ ${#MCU_TARGETS[@]} -gt 0 ]]; then
    step "Checking host Docker container"
    CONTAINER=$(get_container_id)
    if [[ -z "$CONTAINER" ]]; then
        echo "ERROR: Docker container not running on host."
        echo "Start it with: docker compose up -d"
        exit 1
    fi
    echo "Using container: ${CONTAINER:0:12}"
fi

# ── Clean micro-ROS (if requested) ───────────────────────────────────────
if [[ "$CLEAN_MICROROS" == true && ${#MCU_TARGETS[@]} -gt 0 ]]; then
    step "Cleaning micro-ROS libraries"
    # Clean for first MCU target (clears shared libmicroros)
    docker_exec "cd $CONTAINER_WS && pio run -e ${MCU_TARGETS[0]} -t clean_microros"
    echo "micro-ROS cleaned — next build will regenerate"
fi

# ── Process each MCU target ──────────────────────────────────────────────
declare -A RESULTS=()

for env in "${MCU_TARGETS[@]}"; do
    IFS=':' read -r method ext ip <<< "${DEVICE_INFO[$env]}"

    step "$env ($method)"

    # ── Build ──
    if [[ "$DO_BUILD" == true ]]; then
        echo "Building $env in Docker..."
        if docker_exec "cd $CONTAINER_WS && pio run -e $env"; then
            echo "Build OK"
        else
            echo "BUILD FAILED for $env"
            RESULTS[$env]="BUILD_FAILED"
            continue
        fi
    fi

    if [[ "$DO_FLASH" == false ]]; then
        RESULTS[$env]="BUILT"
        continue
    fi

    # ── Extract firmware from Docker ──
    CONTAINER=$(get_container_id)
    CONTAINER_FW="$CONTAINER_WS/.pio/build/$env/firmware.$ext"

    if ! docker exec "$CONTAINER" test -f "$CONTAINER_FW"; then
        echo "ERROR: $CONTAINER_FW not found in container"
        echo "Build first or remove --no-build"
        RESULTS[$env]="NOT_FOUND"
        continue
    fi

    LOCAL_TMP=$(mktemp -d)
    LOCAL_FW="$LOCAL_TMP/firmware-${env}.$ext"
    docker cp "$CONTAINER:$CONTAINER_FW" "$LOCAL_FW"
    FW_SIZE=$(wc -c < "$LOCAL_FW")
    echo "Extracted: firmware-${env}.$ext ($FW_SIZE bytes)"

    # ── Verify Pi connectivity ──
    if ! ssh_pi "echo ok" &>/dev/null; then
        echo "ERROR: Cannot SSH to Pi at $PI"
        rm -rf "$LOCAL_TMP"
        RESULTS[$env]="NO_PI"
        continue
    fi

    # ── Transfer to Pi ──
    ssh_pi "mkdir -p $PI_FIRMWARE_DIR"
    scp -q "$LOCAL_FW" "$PI:$PI_FIRMWARE_DIR/firmware-${env}.$ext"
    echo "Transferred to Pi"

    # ── Flash ──
    case "$method" in
        teensy)
            echo "Flashing Teensy ($env)..."
            FLASH_OK=false
            FLASH_CMD="teensy_loader_cli -mmcu=TEENSY41 -v -w -s $PI_FIRMWARE_DIR/firmware-${env}.hex"
            for attempt in 1 2 3; do
                echo "  Attempt $attempt/3..."
                if ssh_pi "timeout 20 $FLASH_CMD" 2>&1; then
                    FLASH_OK=true
                    break
                fi
                if [[ $attempt -lt 3 ]]; then
                    echo "  Failed — retrying in 3s (press Teensy reset button)..."
                    sleep 3
                fi
            done
            if [[ "$FLASH_OK" == true ]]; then
                echo "Teensy flash OK"
                RESULTS[$env]="OK"
            else
                echo "Teensy flash FAILED after 3 attempts"
                RESULTS[$env]="FLASH_FAILED"
            fi
            ;;

        esp32)
            # Check device reachability from Pi
            if ! ssh_pi "ping -c1 -W2 $ip" &>/dev/null; then
                echo "SKIP: $env unreachable at $ip"
                RESULTS[$env]="UNREACHABLE"
                rm -rf "$LOCAL_TMP"
                continue
            fi

            echo "OTA flashing $env at $ip..."
            if ssh_pi "bash $PI_SCRIPTS/flash_esp32_ota.sh $ip $PI_FIRMWARE_DIR/firmware-${env}.bin 3" 2>&1; then
                echo "OTA flash OK"
                RESULTS[$env]="OK"
            else
                echo "OTA flash FAILED for $env"
                RESULTS[$env]="FLASH_FAILED"
            fi
            ;;
    esac

    rm -rf "$LOCAL_TMP"
done

# ── ROS2 deploy on Pi ────────────────────────────────────────────────────
if [[ "$DO_ROS" == true ]]; then
    step "ROS2: Git Pull on Pi ($BRANCH)"
    ssh_pi "cd $PI_REPO && \
        git config --global --add safe.directory $PI_REPO 2>/dev/null || true && \
        git fetch origin && \
        git stash --include-untracked 2>/dev/null || true && \
        git checkout $BRANCH && \
        git reset --hard origin/$BRANCH && \
        git submodule update --init --recursive && \
        echo 'HEAD:' && git log --oneline -1"

    step "ROS2: Colcon Build on Pi"
    # Ensure Pi Docker container is running
    if ! ssh_pi "docker compose -f $PI_REPO/docker-compose.yml ps -q devcontainer" &>/dev/null; then
        echo "Starting Pi Docker container..."
        ssh_pi "cd $PI_REPO && docker compose up -d"
        sleep 10
    fi
    ssh_pi "docker compose -f $PI_REPO/docker-compose.yml exec -T devcontainer bash -c \
        'source /opt/ros/jazzy/setup.bash && cd /home/ubuntu/ros2_workspaces && colcon build --executor sequential'"
    echo "Colcon build complete"
    RESULTS[ros]="OK"
fi

# ── Summary ──────────────────────────────────────────────────────────────
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

step "Summary (${DURATION}s)"
for target in "${TARGETS[@]}"; do
    result="${RESULTS[$target]:-SKIPPED}"
    case "$result" in
        OK)            symbol="OK" ;;
        BUILT)         symbol="BUILT (no flash)" ;;
        BUILD_FAILED)  symbol="BUILD FAILED" ;;
        FLASH_FAILED)  symbol="FLASH FAILED" ;;
        UNREACHABLE)   symbol="UNREACHABLE" ;;
        NOT_FOUND)     symbol="FW NOT FOUND" ;;
        NO_PI)         symbol="PI UNREACHABLE" ;;
        *)             symbol="$result" ;;
    esac
    printf "  %-35s %s\n" "$target" "$symbol"
done
echo ""
