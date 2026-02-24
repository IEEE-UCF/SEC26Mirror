# Scripts — Deployment & Automation

This directory contains all scripts used for deploying the SEC26 robot, flashing firmware, building ROS 2, and automating CI/CD on the Raspberry Pi.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Raspberry Pi Setup (One-Time)](#raspberry-pi-setup-one-time)
- [Button Daemon (Physical Deploy Button)](#button-daemon-physical-deploy-button)
- [CI/CD Workflow (Gitea Actions)](#cicd-workflow-gitea-actions)
- [In-Container Scripts](#in-container-scripts)
- [Utility Scripts](#utility-scripts)
- [Script Reference](#script-reference)
- [Troubleshooting](#troubleshooting)

---

## Architecture Overview

The deployment pipeline has three layers:

```
 Physical button press on Pi
         |
         v
 button-trigger-workflow.py   (runs as systemd service on Pi host)
         |  checks internet connectivity first
         v
 Gitea workflow_dispatch API  (triggers pi-deploy.yml)
         |
         v
 pi-deploy.yml CI workflow    (runs on Pi via Gitea Actions runner)
   |-- build-mcu-artifacts    (builds Teensy firmware on amd64 runner)
   |-- deploy-robot           (runs on rpi-ros2 runner)
         |-- LED blink via systemd transient unit
         |-- git pull + submodule update
         |-- docker compose build + up
         |-- deploy-all.py    (inside container)
               |-- stage_prebuilt_artifacts()
               |-- flash_mcu.sh   (Teensy only)
               |-- start_robot.sh (colcon build)
               |-- ros2 launch test node
```

---

## Raspberry Pi Setup (One-Time)

### Prerequisites

- Raspberry Pi 4/5 running Raspberry Pi OS (64-bit)
- Docker and Docker Compose installed
- Gitea Actions runner registered (label: `rpi-ros2`)
- Python 3 with `requests` and `gpiozero` packages
- `gpioset` from `libgpiod` (for LED control)
- Physical deploy button wired to BCM pin 22 (configurable via `BTN_PIN`)
- Status LED wired to GPIO pin 17 via `gpiochip0`
- Teensy 4.1 connected via USB

### 1. Clone the repository

```bash
cd /home/ieee
git clone --recurse-submodules https://gitea.syndric.org/ieeeucf/SEC26.git
cd SEC26
```

### 2. Initialize environment

```bash
./scripts/initialize-env.sh
```

This copies `.env.example` to `.env`. Edit `.env` and set:
- `BUILD_TARGET=prod` (for robot deployment)
- Display/network settings for your platform

### 3. Generate a Gitea API token

1. Go to `https://gitea.syndric.org` > Settings > Applications
2. Create a token with `repo` scope
3. Save the token for the next step

### 4. Install the button daemon

```bash
# Install Python dependencies on the Pi host
pip3 install requests gpiozero

# Copy the systemd service file
sudo cp scripts/sec26-button-dispatch.service /etc/systemd/system/

# Edit the service file — replace REPLACE_WITH_TOKEN with your Gitea API token
sudo nano /etc/systemd/system/sec26-button-dispatch.service
```

In the service file, update these `Environment=` lines as needed:

| Variable | Default | Description |
|----------|---------|-------------|
| `GITEA_TOKEN` | `REPLACE_WITH_TOKEN` | **Required.** Your Gitea API token |
| `GITEA_URL` | `https://gitea.syndric.org` | Gitea instance URL |
| `GITEA_OWNER` | `ieeeucf` | Repository owner |
| `GITEA_REPO` | `SEC26` | Repository name |
| `GITEA_WORKFLOW` | `pi-deploy.yml` | Workflow file to dispatch |
| `GITEA_REF` | `prod` | Branch to deploy |
| `BTN_PIN` | `22` | BCM pin for the physical button |

Then enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable sec26-button-dispatch
sudo systemctl start sec26-button-dispatch

# Verify it's running
sudo systemctl status sec26-button-dispatch
journalctl -u sec26-button-dispatch -f
```

### 5. Start the Docker environment

```bash
cd /home/ieee/SEC26
docker compose up -d
```

### 6. Register the Gitea Actions runner

Follow the Gitea documentation to register an Actions runner on the Pi with the label `rpi-ros2`. The runner executes the `deploy-robot` job from `pi-deploy.yml` directly on the Pi host.

---

## Button Daemon (Physical Deploy Button)

**File:** `button-trigger-workflow.py`
**Service:** `sec26-button-dispatch.service`

The button daemon runs as a systemd service on the Pi host. When the physical button is held for 300ms:

1. **Connectivity check** — pings `8.8.8.8` (3s timeout), falls back to `curl` against the Gitea API. If both fail, the dispatch is skipped with a warning.
2. **Workflow dispatch** — sends a POST to the Gitea API to trigger `pi-deploy.yml` on the configured branch.

### Monitoring

```bash
# Live logs
journalctl -u sec26-button-dispatch -f

# Restart after code changes
sudo systemctl restart sec26-button-dispatch
```

---

## CI/CD Workflow (Gitea Actions)

**File:** `.gitea/workflows/pi-deploy.yml`

Triggered by:
- Push to `prod` or `CD-testing` branches
- Manual `workflow_dispatch` (button press or Gitea UI)

### Job 1: `build-mcu-artifacts` (amd64 runner)

Builds the Teensy firmware on an x86 runner to avoid ARM64 compilation issues:
1. Checks out code with submodules
2. Copies `mcu_msgs` into `mcu_ws/extra_packages/`
3. Restores PlatformIO cache (cleans micro-ROS on cache miss)
4. Runs `pio run -e robot`
5. Uploads `firmware.hex` / `firmware.elf` as artifacts

### Job 2: `deploy-robot` (rpi-ros2 runner)

Runs on the Pi itself:
1. **Internet check** — aborts early if offline
2. **Downloads** prebuilt MCU artifacts from Job 1
3. **LED blink** — starts a `systemd-run` transient unit (`sec26-bootled`) to blink GPIO 17
4. **Git pull** — fetches and hard-resets to the deployed branch
5. **Verification** — runs `test_deploy.py` to log a timestamp
6. **Docker rebuild** — full `down`/`build`/`up` if Dockerfile changed, otherwise `stop`/`build`/`up`
7. **Deploy inside container** — runs `deploy-all.py` via `docker exec`:
   - Stages prebuilt Teensy firmware
   - Flashes Teensy (if `mcu_ws/` changed or manual dispatch)
   - Builds ROS 2 with colcon
   - Launches test node
8. **Cleanup** — prunes old Docker images
9. **LED off** — stops the `sec26-bootled` unit, sets GPIO 17 high (LED on steady = deploy complete)

---

## In-Container Scripts

These scripts run **inside** the Docker container (not on the Pi host directly).

### `deploy-all.py`

**Usage:**
```bash
python3 /home/ubuntu/scripts/deploy-all.py [--skip-mcu] [--skip-ros] [--skip-launch]
```

Orchestrates the full deployment sequence:

| Step | Flag to skip | What it does |
|------|-------------|--------------|
| 1. Flash MCU | `--skip-mcu` | Stages prebuilt artifacts, then runs `flash_mcu.sh` |
| 2. Build ROS | `--skip-ros` | Runs `rosdep install` + `start_robot.sh` (colcon build) |
| 3. Launch node | `--skip-launch` | Sources ROS install and launches test node |

The workflow automatically passes `--skip-mcu` when no `mcu_ws/` files changed.

### `flash_mcu.sh`

**Usage:**
```bash
bash /home/ubuntu/scripts/flash_mcu.sh
```

Flashes the Teensy 4.1 with up to 5 retry attempts:
1. Ensures PlatformIO and the Teensy platform are installed
2. If a prebuilt `firmware.hex` exists, flashes directly via `teensy_loader_cli`
3. Falls back to `pio run -t upload -e robot` if no prebuilt or no CLI

### `start_robot.sh`

**Usage:**
```bash
bash /home/ubuntu/scripts/start_robot.sh
```

Sources ROS 2 Jazzy and runs `colcon build --executor sequential` in the ROS 2 workspace.

---

## Utility Scripts

| Script | Where it runs | Description |
|--------|--------------|-------------|
| `initialize-env.sh` | Host | Copies `.env.example` to `.env` if not present |
| `init_bootstrap.sh` | Docker init container | Chowns Docker volumes and seeds `libs_external` |
| `test_deploy.py` | Pi host (via workflow) | Writes a timestamped verification file to `log/` |
| `export_urdf.sh` | Container | Exports URDF/STL from Onshape API |
| `launch_config.yaml` | Container | Configures which ROS packages to launch |

---

## Script Reference

```
scripts/
  button-trigger-workflow.py    # Systemd daemon: button -> Gitea dispatch
  sec26-button-dispatch.service # Systemd unit file for the button daemon
  deploy-all.py                 # Container: orchestrates MCU flash + ROS build + launch
  flash_mcu.sh                  # Container: flashes Teensy with retries
  start_robot.sh                # Container: colcon build for ROS 2
  test_deploy.py                # Pi host: deployment verification log
  initialize-env.sh             # Host: creates .env from .env.example
  init_bootstrap.sh             # Docker init: volume ownership + libs_external seed
  export_urdf.sh                # Container: Onshape URDF export
  launch_config.yaml            # Container: ROS launch configuration
  onshape-config.json           # Container: Onshape API configuration
```

---

## Troubleshooting

### Button press does nothing

1. Check the service is running: `sudo systemctl status sec26-button-dispatch`
2. Check logs: `journalctl -u sec26-button-dispatch -f`
3. If logs say "Dispatch skipped", the Pi has no internet. Check network.
4. Verify `GITEA_TOKEN` is set correctly in the service file.

### LED keeps blinking after deploy

The LED cleanup step runs `systemctl stop sec26-bootled`. If it didn't stop:

```bash
sudo systemctl stop sec26-bootled
gpioset -c gpiochip0 17=1
```

### Teensy won't flash

- Ensure the Teensy is connected via USB and visible (`ls /dev/ttyACM*`)
- Press the physical program button on the Teensy if it's unresponsive
- Check that PlatformIO's Teensy platform is installed inside the container
- Review `flash_mcu.sh` output for specific error messages

### Docker container won't start

```bash
# Check container logs
docker compose logs devcontainer

# Ensure init-bootstrap completed
docker compose logs init-bootstrap

# Full rebuild
docker compose down
docker compose build --no-cache
docker compose up -d
```

### Deploy aborts with "No internet connectivity"

The workflow checks connectivity before proceeding. Ensure the Pi has network access:

```bash
ping -c 1 8.8.8.8
curl -sf https://gitea.syndric.org/api/v1/version
```
