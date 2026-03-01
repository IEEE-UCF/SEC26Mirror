#!/usr/bin/env python3
"""
Deploy orchestrator — watches for trigger files and runs the deployment pipeline.

Runs as a systemd service on the Pi host. Triggered by:
  1. MCU button press (via secbot_deploy ROS2 node writing trigger file)
  2. Gitea CI workflow writing trigger file

Pipeline steps:
  1. Git pull (if online) — stashes local changes, checks out prod, force pulls
  2. Detect changes (git diff + firmware hash comparison)
  3. Flash ESP32s via OTA (skipped if firmware unchanged)
  4. Flash Teensy via USB (skipped if firmware unchanged, done last — reboots MCU)
  5. Docker restart (if needed)
  6. Colcon build
"""

import hashlib
import json
import logging
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_DIR = Path("/home/ieee/SEC26")
DEPLOY_DIR = REPO_DIR / "scripts" / ".deploy"
TRIGGER_FILE = DEPLOY_DIR / "trigger"
STATUS_FILE = DEPLOY_DIR / "status"
SHA_FILE = DEPLOY_DIR / "last_deployed_sha"
FIRMWARE_HASHES_FILE = DEPLOY_DIR / "firmware_hashes.json"
SCRIPTS_DIR = REPO_DIR / "scripts"

# ESP32 devices: (name, IP, PlatformIO environment)
ESP32_DEVICES = [
    ("beacon1", "192.168.4.20", "beacon1"),
    ("beacon2", "192.168.4.21", "beacon2"),
    ("beacon3", "192.168.4.22", "beacon3"),
    ("minibot", "192.168.4.24", "minibot"),
    ("drone", "192.168.4.25", "drone"),
]

# PlatformIO environments that use micro-ROS (need clean_microros on mcu_msgs change)
MICROROS_ENVS = ["robot", "beacon1", "beacon2", "beacon3", "minibot"]

DOCKER_COMPOSE = "docker compose"
CONTAINER_NAME = "sec26-devcontainer-1"
MCU_WS = "/home/ubuntu/mcu_workspaces/sec26mcu"

log = logging.getLogger("deploy")

# ─── Helpers ───────────────────────────────────────────────────────────────────


def write_status(phase: str, message: str = "", progress: str = ""):
    """Write current status to the IPC file for secbot_deploy to read."""
    DEPLOY_DIR.mkdir(parents=True, exist_ok=True)
    with open(STATUS_FILE, "w") as f:
        f.write(f"phase={phase}\n")
        if message:
            f.write(f"message={message}\n")
        if progress:
            f.write(f"progress={progress}\n")


def run(cmd: str, cwd: str | None = None, check: bool = True,
        timeout: int = 600,
        stream: bool = False) -> subprocess.CompletedProcess:
    """Run a shell command with logging.

    When stream=True, stdout/stderr are printed live to the log (useful for
    long-running commands like OTA flashing). Otherwise output is captured and
    logged after completion.
    """
    log.info("  $ %s", cmd)

    if stream:
        proc = subprocess.Popen(
            cmd, shell=True, cwd=cwd,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        stdout_lines = []
        try:
            deadline = time.time() + timeout
            for line in proc.stdout:
                line = line.rstrip("\n")
                stdout_lines.append(line)
                log.info("    %s", line)
                if time.time() > deadline:
                    proc.kill()
                    proc.wait()
                    raise subprocess.TimeoutExpired(cmd, timeout)
            proc.wait()
            if proc.returncode > 0 and check:
                raise subprocess.CalledProcessError(
                    proc.returncode, cmd,
                    output="\n".join(stdout_lines))
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            raise
        return subprocess.CompletedProcess(
            cmd, proc.returncode,
            stdout="\n".join(stdout_lines), stderr="")

    result = subprocess.run(
        cmd, shell=True, cwd=cwd, capture_output=True, text=True,
        check=check, timeout=timeout,
    )
    if result.stdout.strip():
        for line in result.stdout.strip().splitlines():
            log.debug("    %s", line)
    if result.stderr.strip():
        for line in result.stderr.strip().splitlines():
            log.debug("    (stderr) %s", line)
    return result


def check_cancelled() -> bool:
    """Check if a cancel request was written to the trigger file."""
    if TRIGGER_FILE.exists():
        content = TRIGGER_FILE.read_text().strip()
        if "cancel" in content:
            write_status("cancelled", "Deployment cancelled")
            log.warning("[CANCEL] Deployment cancelled by user")
            return True
    return False


def fmt_duration(seconds: float) -> str:
    """Format a duration in human-readable form."""
    if seconds < 60:
        return f"{seconds:.1f}s"
    minutes = int(seconds // 60)
    secs = seconds % 60
    return f"{minutes}m {secs:.0f}s"


# ─── Network ──────────────────────────────────────────────────────────────────


def is_online() -> bool:
    """Check if the Pi has internet connectivity."""
    try:
        result = subprocess.run(
            ["curl", "--max-time", "5", "-sf",
             "https://gitea.syndric.org/api/v1/version"],
            capture_output=True, timeout=10,
        )
        return result.returncode == 0
    except Exception:
        return False


def resolve_online(config: dict) -> bool:
    """Determine online status from config, auto-detecting if needed."""
    val = config.get("online", "auto")
    if val == "true":
        return True
    if val == "false":
        return False
    online = is_online()
    log.info("[NETWORK] Connectivity: %s", "ONLINE" if online else "OFFLINE")
    return online


def is_esp32_reachable(ip: str, port: int = 3232, timeout: float = 2.0) -> bool:
    """Check if an ESP32 is reachable via ICMP ping."""
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(timeout)), ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=timeout + 1,
        )
        return result.returncode == 0
    except Exception:
        return False


# ─── Git ───────────────────────────────────────────────────────────────────────


def get_current_sha() -> str | None:
    """Get current HEAD SHA."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True, text=True, cwd=str(REPO_DIR),
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return None


def get_last_deployed_sha() -> str | None:
    """Read the last successfully deployed SHA."""
    if SHA_FILE.exists():
        sha = SHA_FILE.read_text().strip()
        if sha:
            return sha
    return None


def save_deployed_sha(sha: str):
    """Save the current HEAD as the last deployed SHA."""
    SHA_FILE.write_text(sha + "\n")


def parse_trigger(content: str) -> dict:
    """Parse trigger file content into a dict."""
    config = {}
    for line in content.strip().splitlines():
        if "=" in line:
            key, val = line.split("=", 1)
            config[key.strip()] = val.strip()
    return config


def get_current_branch() -> str | None:
    """Get the current git branch name."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            capture_output=True, text=True, cwd=str(REPO_DIR),
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return None


# ─── Firmware hash tracking ───────────────────────────────────────────────────


def file_md5(path: str) -> str | None:
    """Compute MD5 hash of a file."""
    try:
        h = hashlib.md5()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None


def load_firmware_hashes() -> dict:
    """Load previously flashed firmware hashes."""
    if FIRMWARE_HASHES_FILE.exists():
        try:
            return json.loads(FIRMWARE_HASHES_FILE.read_text())
        except Exception:
            pass
    return {}


def save_firmware_hash(env_name: str, fw_hash: str):
    """Save a firmware hash after successful flash."""
    hashes = load_firmware_hashes()
    hashes[env_name] = fw_hash
    DEPLOY_DIR.mkdir(parents=True, exist_ok=True)
    FIRMWARE_HASHES_FILE.write_text(json.dumps(hashes, indent=2) + "\n")


def firmware_changed(env_name: str, firmware_path: str) -> bool:
    """Check if firmware binary differs from last successful flash."""
    current_hash = file_md5(firmware_path)
    if not current_hash:
        log.warning("  [HASH] Could not hash %s — will flash", firmware_path)
        return True

    prev_hashes = load_firmware_hashes()
    prev_hash = prev_hashes.get(env_name)

    if prev_hash == current_hash:
        log.info("  [HASH] %s firmware unchanged (md5:%s) — SKIPPING flash",
                 env_name, current_hash[:12])
        return False

    if prev_hash:
        log.info("  [HASH] %s firmware CHANGED: %s -> %s",
                 env_name, prev_hash[:12], current_hash[:12])
    else:
        log.info("  [HASH] %s firmware hash not tracked yet (md5:%s) — will flash",
                 env_name, current_hash[:12])
    return True


# ─── Pipeline steps ───────────────────────────────────────────────────────────


def step_git_pull(config: dict) -> tuple[bool, str | None]:
    """Step 1: Git pull latest code.

    Returns (success, original_branch) so the caller can restore afterward.
    """
    write_status("git_pull", "Checking for updates...")

    online = resolve_online(config)
    branch = config.get("branch", "prod")
    original_branch = get_current_branch()

    if not online:
        log.info("[GIT] Offline — skipping git pull, using local code")
        write_status("git_pull", "Offline — using local code")
        return True, original_branch

    try:
        run("git config --global --add safe.directory /home/ieee/SEC26",
            cwd=str(REPO_DIR), check=False)

        # Fetch first to check if there are remote changes before stashing
        run("git fetch --all", cwd=str(REPO_DIR))

        # Compare local HEAD with remote branch tip
        local_sha = get_current_sha()
        try:
            remote_result = subprocess.run(
                ["git", "rev-parse", f"origin/{branch}"],
                capture_output=True, text=True, cwd=str(REPO_DIR),
            )
            remote_sha = remote_result.stdout.strip() if remote_result.returncode == 0 else None
        except Exception:
            remote_sha = None

        already_on_branch = original_branch == branch
        if already_on_branch and remote_sha and local_sha == remote_sha:
            log.info("[GIT] Already on %s at %s — no remote changes, skipping pull",
                     branch, local_sha[:8])
            return True, original_branch

        if remote_sha:
            log.info("[GIT] Updating: %s -> %s",
                     local_sha[:8] if local_sha else "?",
                     remote_sha[:8])
        write_status("git_pull", "Pulling latest code...")

        log.info("[GIT] Stashing local changes...")
        run("git stash --include-untracked",
            cwd=str(REPO_DIR), check=False)

        log.info("[GIT] Checking out %s...", branch)
        run(f"git checkout {branch}", cwd=str(REPO_DIR))

        run(f"git reset --hard origin/{branch}", cwd=str(REPO_DIR))
        run("git submodule update --init --recursive", cwd=str(REPO_DIR))
        return True, original_branch
    except subprocess.CalledProcessError as e:
        log.error("[GIT] Failed: %s", e.stderr[:200] if e.stderr else str(e))
        write_status("git_pull", "Git pull failed — using local code")
        return True, original_branch


def step_git_restore(original_branch: str | None):
    """Restore the original branch and pop stash after deploy completes."""
    if not original_branch:
        return
    current = get_current_branch()
    if current == original_branch:
        run("git stash pop", cwd=str(REPO_DIR), check=False)
        return
    try:
        log.info("[GIT] Restoring branch %s...", original_branch)
        run(f"git checkout {original_branch}", cwd=str(REPO_DIR))
        run("git stash pop", cwd=str(REPO_DIR), check=False)
    except subprocess.CalledProcessError as e:
        log.error("[GIT] Failed to restore branch: %s",
                  e.stderr[:200] if e.stderr else str(e))


def _classify_changed_files(files: list[str]) -> tuple[bool, bool, bool]:
    """Classify a list of changed file paths.

    Returns (mcu_changed, ros_changed, microros_rebuild).
    microros_rebuild is True when changes require a micro-ROS clean rebuild:
      - mcu_msgs definitions (ros2_ws/src/mcu_msgs/)
      - micro_ros_platformio submodules (mcu_ws/libs_external/)
      - PlatformIO build config (mcu_ws/platformio.ini)
      - extra_packages (mcu_ws/extra_packages/)
    This matches the CI cache key in platformio-robot.yml.
    """
    mcu = any(f.startswith("mcu_ws/") for f in files)
    ros = any(f.startswith("ros2_ws/")
              or f.startswith("Dockerfile")
              or f.startswith("docker-compose")
              for f in files)

    microros_rebuild = any(
        f.startswith("ros2_ws/src/mcu_msgs/")
        or f.startswith("mcu_ws/libs_external/")
        or f.startswith("mcu_ws/extra_packages/")
        or f == "mcu_ws/platformio.ini"
        or f == "mcu_ws/custom_microros.meta"
        for f in files
    )

    if microros_rebuild:
        mcu = True  # micro-ROS changes require MCU firmware rebuild

    return mcu, ros, microros_rebuild


def detect_changes() -> tuple[bool, bool, bool]:
    """Step 2: Detect what changed.

    Returns (mcu_changed, ros_changed, microros_rebuild).
    """
    base_sha = get_last_deployed_sha()
    current_sha = get_current_sha()

    log.info("[DETECT] Last deployed SHA: %s",
             base_sha[:8] if base_sha else "(none)")
    log.info("[DETECT] Current HEAD SHA:  %s",
             current_sha[:8] if current_sha else "(unknown)")

    # If last deployed SHA matches HEAD, nothing has changed
    if base_sha and current_sha and base_sha == current_sha:
        log.info("[DETECT] HEAD matches last deploy — no git changes detected")
        return False, False, False

    # Diff against last deployed SHA
    if base_sha:
        try:
            result = run(f"git diff {base_sha} HEAD --name-only",
                         cwd=str(REPO_DIR), check=False)
            if result.returncode == 0 and result.stdout.strip():
                files = result.stdout.strip().splitlines()
                mcu, ros, microros_rebuild = _classify_changed_files(files)
                log.info("[DETECT] %d files changed since last deploy (%s..%s):",
                         len(files), base_sha[:8],
                         current_sha[:8] if current_sha else "HEAD")
                for f in files[:30]:
                    log.info("[DETECT]   %s", f)
                if len(files) > 30:
                    log.info("[DETECT]   ... and %d more", len(files) - 30)
                log.info("[DETECT] Result: MCU=%s, ROS=%s, microros_rebuild=%s",
                         mcu, ros, microros_rebuild)
                return mcu, ros, microros_rebuild
        except Exception as e:
            log.warning("[DETECT] SHA diff failed: %s", e)

    # Fallback: diff HEAD~1 (first deploy or SHA file missing)
    log.warning("[DETECT] No valid last_deployed_sha — "
                "falling back to HEAD~1 diff (first deploy?)")
    try:
        result = run("git diff HEAD~1 HEAD --name-only", cwd=str(REPO_DIR),
                      check=False)
        files = result.stdout.strip().splitlines() if result.stdout.strip() else []
        mcu, ros, microros_rebuild = _classify_changed_files(files)
        log.info("[DETECT] HEAD~1 fallback: MCU=%s, ROS=%s, microros_rebuild=%s",
                 mcu, ros, microros_rebuild)
        return mcu, ros, microros_rebuild
    except Exception:
        log.warning("[DETECT] All detection failed — assuming everything changed")
        return True, True, True


def find_firmware_binary(env_name: str) -> str | None:
    """Find a firmware binary for the given PIO environment.

    Search order:
      1. Loose file in repo root (e.g., beacon1.bin — from CI artifacts)
      2. Previously copied firmware in .deploy/firmware/<env>/
      3. Copy from Docker container (build artifacts are in a Docker volume)
    """
    # Repo root loose files
    for ext in [".hex", ".bin"]:
        path = REPO_DIR / f"{env_name}{ext}"
        if path.exists():
            return str(path)

    # Previously copied firmware in deploy cache
    fw_dir = DEPLOY_DIR / "firmware" / env_name
    for name in ["firmware.bin", "firmware.hex"]:
        path = fw_dir / name
        if path.exists():
            return str(path)

    # Try copying from Docker container (volume not visible on host)
    copied = copy_firmware_from_container(env_name)
    if copied:
        return copied

    return None


def copy_firmware_from_container(env_name: str) -> str | None:
    """Copy firmware binary from Docker container to host for OTA/hashing.

    The .pio directory is a Docker volume (not a bind mount), so build
    artifacts are only accessible inside the container.  This copies the
    binary out to scripts/.deploy/firmware/<env>/.
    """
    fw_dir = DEPLOY_DIR / "firmware" / env_name
    fw_dir.mkdir(parents=True, exist_ok=True)

    container_base = f"{MCU_WS}/.pio/build/{env_name}"

    # Try .bin first (ESP32), then .hex (Teensy)
    for ext in ["bin", "hex"]:
        container_path = f"{container_base}/firmware.{ext}"
        host_path = fw_dir / f"firmware.{ext}"
        try:
            run(f"docker cp {CONTAINER_NAME}:{container_path} {host_path}",
                check=True, timeout=30)
            if host_path.exists() and host_path.stat().st_size > 0:
                log.info("  [COPY] Copied %s -> %s", container_path, host_path)
                return str(host_path)
        except subprocess.CalledProcessError:
            continue

    return None


def build_firmware_in_docker(env_name: str) -> str | None:
    """Build firmware inside Docker container and copy binary to host."""
    log.info("  [BUILD] Building %s in Docker...", env_name)
    t0 = time.time()
    try:
        run(f"docker exec {CONTAINER_NAME} bash -c "
            f"'cd {MCU_WS} && pio run -e {env_name}'",
            timeout=900)
        elapsed = time.time() - t0
        log.info("  [BUILD] %s built in %s", env_name, fmt_duration(elapsed))

        # Copy firmware out of the container volume to host
        return copy_firmware_from_container(env_name)
    except subprocess.TimeoutExpired:
        log.error("  [BUILD] %s build timed out after 15m", env_name)
        return None
    except subprocess.CalledProcessError as e:
        log.error("  [BUILD] Failed to build %s: %s",
                  env_name, e.stderr[:200] if e.stderr else str(e))
        return None


def step_clean_microros() -> bool:
    """Clean shared micro-ROS libraries (libs_external).

    Required when mcu_msgs definitions, micro_ros_platformio submodules,
    custom_microros.meta, or platformio.ini change so that libmicroros is
    regenerated. All environments share the same libs_external, so a single
    clean suffices. Also clears cached firmware binaries for all micro-ROS
    environments to force fresh rebuilds.
    """
    write_status("clean_microros", "Cleaning micro-ROS libraries...")
    log.info("")
    log.info("─── Clean micro-ROS ─────────────────────────────────────")
    log.info("  [MICROROS] micro-ROS rebuild needed — cleaning shared libs")

    t0 = time.time()

    # Single clean — all envs share libs_external
    try:
        run(f"docker exec {CONTAINER_NAME} bash -c "
            f"'cd {MCU_WS} && pio run -e robot -t clean_microros'",
            timeout=120, check=False)
    except Exception as e:
        log.warning("  [MICROROS] Clean failed: %s", e)

    # Clear cached firmware binaries so stale builds aren't reused
    fw_cache = DEPLOY_DIR / "firmware"
    for env in MICROROS_ENVS:
        fw_dir = fw_cache / env
        if fw_dir.exists():
            for f in fw_dir.iterdir():
                f.unlink()
            log.info("  [MICROROS] Cleared cached firmware for %s", env)

    elapsed = time.time() - t0
    log.info("  [MICROROS] Clean complete (%s)", fmt_duration(elapsed))
    write_status("clean_microros", f"micro-ROS cleaned ({fmt_duration(elapsed)})")
    return True


def step_esp32_ota(config: dict, force: bool = False) -> bool:
    """Step 3: Flash ESP32s via OTA."""
    write_status("esp32_ota", "Starting ESP32 OTA flashing...")
    log.info("")
    log.info("─── ESP32 OTA Flash ─────────────────────────────────────")

    total = len(ESP32_DEVICES)
    flash_script = SCRIPTS_DIR / "flash_esp32_ota.sh"
    results = []  # (name, status)

    for idx, (name, ip, env) in enumerate(ESP32_DEVICES, 1):
        if check_cancelled():
            return False

        write_status("esp32_ota", f"Flashing {name} ({idx}/{total})...",
                     f"{idx}/{total}")
        log.info("")
        log.info("  [%d/%d] %s (%s, env=%s)", idx, total, name, ip, env)

        # Find firmware binary
        firmware = find_firmware_binary(env)
        if not firmware:
            # Build it
            built = build_firmware_in_docker(env)
            if built and os.path.exists(built):
                firmware = built

        if not firmware or not os.path.exists(firmware):
            log.warning("  [OTA] %s: No firmware binary found — SKIPPED", name)
            results.append((name, "SKIPPED"))
            write_status("esp32_ota", f"{name}: SKIPPED (no fw)", f"{idx}/{total}")
            continue

        # Check firmware hash before flashing (skip check if force)
        if not force and not firmware_changed(env, firmware):
            results.append((name, "SKIPPED"))
            write_status("esp32_ota", f"{name}: SKIPPED (unchanged)",
                         f"{idx}/{total}")
            continue

        # Check device reachability
        if not is_esp32_reachable(ip):
            log.warning("  [OTA] %s (%s): Device unreachable — SKIPPED", name, ip)
            results.append((name, "SKIPPED"))
            write_status("esp32_ota", f"{name}: SKIPPED (offline)",
                         f"{idx}/{total}")
            continue

        # Flash
        fw_hash = file_md5(firmware)
        log.info("  [OTA] %s: Flashing %s (md5:%s)...",
                 name, firmware, fw_hash[:12] if fw_hash else "?")
        t0 = time.time()
        try:
            run(f"bash {flash_script} {ip} {firmware}", timeout=120,
                stream=True)
            elapsed = time.time() - t0
            log.info("  [OTA] %s: SUCCESS in %s", name, fmt_duration(elapsed))
            if fw_hash:
                save_firmware_hash(env, fw_hash)
            # Clean up loose repo-root artifacts
            for ext in [".hex", ".bin"]:
                loose = REPO_DIR / f"{env}{ext}"
                if loose.exists():
                    os.unlink(str(loose))
            results.append((name, "SUCCESS"))
            write_status("esp32_ota", f"{name}: SUCCESS", f"{idx}/{total}")
        except subprocess.TimeoutExpired:
            elapsed = time.time() - t0
            log.error("  [OTA] %s: FAILED (timeout after %s)",
                      name, fmt_duration(elapsed))
            results.append((name, "FAILED"))
            write_status("esp32_ota", f"{name}: FAILED", f"{idx}/{total}")
        except subprocess.CalledProcessError:
            elapsed = time.time() - t0
            log.error("  [OTA] %s: FAILED after %s", name, fmt_duration(elapsed))
            results.append((name, "FAILED"))
            write_status("esp32_ota", f"{name}: FAILED", f"{idx}/{total}")

    # Summary
    log.info("")
    log.info("  ESP32 OTA Summary:")
    summary_lines = []
    for name, status in results:
        log.info("    %-10s %s", name, status)
        summary_lines.append(f"{name}:{status}")
    log.info("")

    # Send summary to OLED
    write_status("esp32_ota", " ".join(summary_lines))

    return True


def step_teensy_flash(config: dict, force: bool = False) -> bool:
    """Step 4: Flash Teensy via USB (done last — reboots MCU)."""
    target = config.get("target", "robot")
    write_status("mcu_flash", f"Flashing Teensy ({target})...")
    log.info("")
    log.info("─── Teensy Flash ────────────────────────────────────────")

    # Find or build firmware
    firmware = find_firmware_binary(target)
    if not firmware:
        built = build_firmware_in_docker(target)
        if built and os.path.exists(built):
            firmware = built

    if not firmware or not os.path.exists(firmware):
        log.error("  [TEENSY] No firmware binary found for '%s' — cannot flash",
                  target)
        write_status("mcu_flash", "Teensy: FAILED (no firmware)")
        return False

    # Check firmware hash (skip check if force)
    if not force and not firmware_changed(target, firmware):
        log.info("  [TEENSY] Firmware identical to last flash — SKIPPED")
        write_status("mcu_flash", "Teensy: SKIPPED (unchanged)")
        return True

    if force:
        log.info("  [TEENSY] FORCE mode — flashing regardless of hash")

    fw_hash = file_md5(firmware)
    log.info("  [TEENSY] Firmware: %s (md5:%s)",
             firmware, fw_hash[:12] if fw_hash else "?")

    for attempt in range(1, 4):
        log.info("  [TEENSY] Flash attempt %d/3...", attempt)
        t0 = time.time()
        try:
            run(f"docker exec {CONTAINER_NAME} bash -c "
                f"'cd {MCU_WS} && pio run -e {target} --target upload'",
                timeout=120, stream=True)
            elapsed = time.time() - t0
            log.info("  [TEENSY] SUCCESS on attempt %d (%s)", attempt,
                     fmt_duration(elapsed))
            write_status("mcu_flash",
                         f"Teensy: SUCCESS ({fmt_duration(elapsed)})")
            if fw_hash:
                save_firmware_hash(target, fw_hash)
            # Clean up loose artifacts
            for ext in [".hex", ".bin"]:
                loose = REPO_DIR / f"{target}{ext}"
                if loose.exists():
                    os.unlink(str(loose))
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            elapsed = time.time() - t0
            log.warning("  [TEENSY] Attempt %d/3 FAILED (%s)",
                        attempt, fmt_duration(elapsed))
            if attempt < 3:
                time.sleep(3)

    log.error("  [TEENSY] FAILED after 3 attempts")
    write_status("mcu_flash", "Teensy: FAILED")
    return False


def step_docker_restart(config: dict) -> bool:
    """Step 5: Restart Docker container."""
    write_status("docker", "Restarting Docker container...")
    log.info("")
    log.info("─── Docker Restart ──────────────────────────────────────")

    online = resolve_online(config)
    t0 = time.time()

    try:
        if online:
            log.info("  [DOCKER] Online — full rebuild cycle (down -> build -> up)")
            run(f"{DOCKER_COMPOSE} down", cwd=str(REPO_DIR), check=False)
            run(f"{DOCKER_COMPOSE} up --build -d", cwd=str(REPO_DIR),
                timeout=900)
        else:
            log.info("  [DOCKER] Offline — restart only (down -> up)")
            run(f"{DOCKER_COMPOSE} down", cwd=str(REPO_DIR), check=False)
            run(f"{DOCKER_COMPOSE} up -d", cwd=str(REPO_DIR), timeout=300)

        log.info("  [DOCKER] Waiting 10s for container to stabilize...")
        time.sleep(10)
        elapsed = time.time() - t0
        log.info("  [DOCKER] SUCCESS (%s)", fmt_duration(elapsed))
        write_status("docker", f"Docker: SUCCESS ({fmt_duration(elapsed)})")
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        elapsed = time.time() - t0
        stderr = getattr(e, "stderr", None)
        log.error("  [DOCKER] FAILED after %s: %s",
                  fmt_duration(elapsed),
                  stderr[:200] if stderr else str(e))
        write_status("failed", "Docker restart failed")
        return False


def step_colcon_build() -> bool:
    """Step 6: Build ROS2 packages inside container."""
    write_status("colcon_build", "Building ROS2 packages...")
    log.info("")
    log.info("─── Colcon Build ────────────────────────────────────────")

    t0 = time.time()
    try:
        run(f"docker exec {CONTAINER_NAME} bash -c "
            "'source /opt/ros/jazzy/setup.bash && "
            "cd /home/ubuntu/ros2_workspaces && colcon build'",
            timeout=600)
        elapsed = time.time() - t0
        log.info("  [COLCON] SUCCESS (%s)", fmt_duration(elapsed))
        write_status("colcon_build",
                     f"Colcon: SUCCESS ({fmt_duration(elapsed)})")
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        elapsed = time.time() - t0
        stderr = getattr(e, "stderr", None)
        log.error("  [COLCON] FAILED after %s: %s",
                  fmt_duration(elapsed),
                  stderr[:200] if stderr else str(e))
        write_status("failed", "Colcon build failed")
        return False


# ─── Main pipeline ────────────────────────────────────────────────────────────


def run_pipeline(config: dict):
    """Execute the full deployment pipeline."""
    pipeline_start = time.time()
    target = config.get("target", "robot")
    branch = config.get("branch", "prod")
    force = config.get("force", "false") == "true"

    log.info("")
    log.info("=" * 60)
    log.info("  DEPLOYMENT STARTED")
    log.info("    Target:  %s", target)
    log.info("    Branch:  %s", branch)
    log.info("    Force:   %s", force)
    log.info("    Config:  %s", config)
    log.info("=" * 60)

    original_branch = None
    steps_run = []

    try:
        # Step 1: Git pull
        log.info("")
        log.info("─── Step 1: Git Pull ────────────────────────────────────")
        success, original_branch = step_git_pull(config)
        if not success:
            return
        if check_cancelled():
            return

        current_sha = get_current_sha()
        last_sha = get_last_deployed_sha()
        log.info("[INFO] Previous deploy: %s",
                 last_sha[:8] if last_sha else "(first deploy)")
        log.info("[INFO] Current HEAD:    %s",
                 current_sha[:8] if current_sha else "(unknown)")

        # Step 2: Detect changes
        log.info("")
        log.info("─── Step 2: Detect Changes ──────────────────────────────")

        if force:
            log.info("[DETECT] FORCE mode — skipping change detection, "
                     "flashing everything")
            mcu_changed, ros_changed, microros_rebuild = True, True, False
        else:
            mcu_changed, ros_changed, microros_rebuild = detect_changes()

        if not mcu_changed and not ros_changed:
            log.info("")
            log.info("=" * 60)
            log.info("  NO CHANGES DETECTED - nothing to deploy")
            log.info("=" * 60)
            if current_sha:
                save_deployed_sha(current_sha)
            write_status("done", "No changes - nothing to deploy")
            return

        # If micro-ROS rebuild needed, MCU firmware must be force-flashed
        # (new micro-ROS libs → different binary)
        force_mcu = force or microros_rebuild

        log.info("")
        log.info("  Deploy plan:")
        if microros_rebuild:
            log.info("    [x] Clean micro-ROS (micro-ROS rebuild needed)")
        if mcu_changed:
            log.info("    [x] ESP32 OTA flash%s",
                     " (FORCED)" if force_mcu else "")
            log.info("    [x] Teensy flash%s",
                     " (FORCED)" if force_mcu else "")
        else:
            log.info("    [ ] ESP32 OTA flash (no MCU changes)")
            log.info("    [ ] Teensy flash (no MCU changes)")
        if ros_changed:
            log.info("    [x] Docker restart%s",
                     " (FORCED)" if force else "")
            log.info("    [x] Colcon build%s",
                     " (FORCED)" if force else "")
        else:
            log.info("    [ ] Docker restart (no ROS changes)")
            log.info("    [ ] Colcon build (no ROS changes)")

        # Step 2b: Clean micro-ROS if rebuild needed
        if microros_rebuild:
            if not step_clean_microros():
                return
            steps_run.append("Clean micro-ROS")
            if check_cancelled():
                return

        # Step 3: ESP32 OTA
        if mcu_changed:
            if not step_esp32_ota(config, force=force_mcu):
                return
            steps_run.append("ESP32 OTA")
            if check_cancelled():
                return

        # Step 4: Teensy flash (last MCU step — reboots MCU)
        if mcu_changed:
            if not step_teensy_flash(config, force=force_mcu):
                return
            steps_run.append("Teensy flash")
            if check_cancelled():
                return

        # Step 5: Docker restart
        if ros_changed:
            if not step_docker_restart(config):
                return
            steps_run.append("Docker restart")
            if check_cancelled():
                return

        # Step 6: Colcon build
        if ros_changed:
            if not step_colcon_build():
                return
            steps_run.append("Colcon build")

        # Save deployed SHA
        if current_sha:
            save_deployed_sha(current_sha)
            log.info("[SHA] Saved deployed SHA: %s", current_sha[:8])

        # Final summary
        elapsed = time.time() - pipeline_start
        log.info("")
        log.info("=" * 60)
        log.info("  DEPLOYMENT COMPLETE")
        log.info("    Duration:  %s", fmt_duration(elapsed))
        log.info("    SHA:       %s -> %s",
                 last_sha[:8] if last_sha else "(none)",
                 current_sha[:8] if current_sha else "(unknown)")
        log.info("    Steps run: %s", ", ".join(steps_run) if steps_run else "(none)")
        log.info("=" * 60)
        log.info("")

        summary_msg = f"Done ({fmt_duration(elapsed)})"
        if steps_run:
            summary_msg += f" [{', '.join(steps_run)}]"
        else:
            summary_msg += " [no steps needed]"
        write_status("done", summary_msg)

    finally:
        step_git_restore(original_branch)


def main():
    """Main loop — poll for trigger files."""
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        stream=sys.stdout,
    )

    DEPLOY_DIR.mkdir(parents=True, exist_ok=True)
    log.info("Deploy orchestrator started — watching %s", TRIGGER_FILE)
    log.info("  Firmware hashes: %s", FIRMWARE_HASHES_FILE)
    log.info("  Last deployed SHA: %s",
             get_last_deployed_sha() or "(none)")

    while True:
        if TRIGGER_FILE.exists():
            content = TRIGGER_FILE.read_text().strip()
            if not content or content == "cancel":
                TRIGGER_FILE.unlink(missing_ok=True)
                time.sleep(1)
                continue

            config = parse_trigger(content)
            TRIGGER_FILE.unlink(missing_ok=True)

            # Clear previous status
            STATUS_FILE.unlink(missing_ok=True)

            try:
                run_pipeline(config)
            except Exception as e:
                log.exception("[ERROR] Pipeline exception: %s", e)
                write_status("failed", f"Pipeline error: {str(e)[:100]}")

        time.sleep(2)


if __name__ == "__main__":
    main()
