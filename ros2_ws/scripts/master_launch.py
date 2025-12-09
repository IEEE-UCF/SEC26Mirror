#!/usr/bin/env python3
"""Master launch script to start all Python launch files in workspace `src/*/launch/`.

This script is intended to be run by a daemon at system startup. It finds all
`*.py` launch files under `src/<package>/launch/` and runs `ros2 launch
<package> <launch_file>` for each one in a subprocess. It also forwards
termination signals to child processes so they exit cleanly.
"""

import os
import signal
import subprocess
import sys
import time
import json
from datetime import datetime
from pathlib import Path


def discover_launch_files(workspace_root: Path):
    pattern = workspace_root / 'src' / '*' / 'launch' / '*.py'
    return sorted(Path().glob(str(pattern)))


def read_config(scripts_dir: Path):
    """Read `launch_config.yaml` or `launch_config.json` from `scripts_dir`.

    Returns a dict or None if no config is found.
    """
    yaml_path = scripts_dir / 'launch_config.yaml'
    json_path = scripts_dir / 'launch_config.json'

    if yaml_path.exists():
        try:
            import yaml
        except Exception:
            print("Found launch_config.yaml but PyYAML is not installed. Install 'pyyaml' or use launch_config.json.")
            return None
        with yaml_path.open('r', encoding='utf-8') as fh:
            return yaml.safe_load(fh)
    if json_path.exists():
        with json_path.open('r', encoding='utf-8') as fh:
            return json.load(fh)
    return None


def find_launch_for_package(workspace_root: Path, package: str):
    p = workspace_root / 'src' / package / 'launch'
    if not p.exists():
        return None
    files = sorted(p.glob('*.py'))
    return files[0].name if files else None


def start_launch(package: str, launch_file: str, logs_dir: Path):
    """Start a ros2 launch and return (proc, stdout_file, stderr_file).

    Logs are created under `logs_dir` with timestamps.
    """
    timestamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    safe_name = f"{package}-{launch_file}".replace(' ', '_')
    stdout_path = logs_dir / f"{safe_name}-{timestamp}.out.log"
    stderr_path = logs_dir / f"{safe_name}-{timestamp}.err.log"

    # Open log files in append mode; line-buffered for interactive-ish output.
    stdout_f = open(stdout_path, 'a', buffering=1, encoding='utf-8')
    stderr_f = open(stderr_path, 'a', buffering=1, encoding='utf-8')

    try:
        proc = subprocess.Popen(['ros2', 'launch', package, launch_file], stdout=stdout_f, stderr=stderr_f)
    except FileNotFoundError:
        # Fallback: try using python -m ros2cli if available
        try:
            cmd = [sys.executable, '-m', 'ros2cli', 'ros2', 'launch', package, launch_file]
            proc = subprocess.Popen(cmd, stdout=stdout_f, stderr=stderr_f)
        except Exception as e:
            stderr_f.write(f"Failed to start launch: {e}\n")
            proc = None
    return proc, stdout_f, stderr_f, stdout_path, stderr_path


def main():
    workspace_root = Path(__file__).resolve().parents[1]
    scripts_dir = Path(__file__).resolve().parent
    print(f"Workspace root: {workspace_root}")

    # Prepare logs dir
    logs_dir = scripts_dir.parent / 'log'
    logs_dir.mkdir(parents=True, exist_ok=True)

    cfg = read_config(scripts_dir)

    procs = []
    fhs = []

    entries = []  # list of (package, launch_file)

    if cfg and isinstance(cfg, dict) and cfg.get('launches'):
        for item in cfg.get('launches', []):
            pkg = item.get('package')
            if not pkg:
                continue
            launch_name = item.get('launch')
            if not launch_name:
                launch_name = find_launch_for_package(workspace_root, pkg)
                if not launch_name:
                    print(f"No launch file found for package '{pkg}', skipping.")
                    continue
            entries.append((pkg, launch_name))
    else:
        # Fallback: auto-discover all launch files
        launch_files = discover_launch_files(workspace_root)
        if not launch_files:
            print("No launch files found under src/*/launch/. Nothing to start.")
            return 0
        for p in launch_files:
            pkg = p.parts[-3]
            entries.append((pkg, p.name))

    print(f"Launching {len(entries)} configured launch(es). Logs in: {logs_dir}")

    def shutdown(signum, frame):
        print(f"Received signal {signum}, terminating children...")
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        # Give children time to exit
        time.sleep(1)
        for p in procs:
            try:
                if p.poll() is None:
                    p.kill()
            except Exception:
                pass
        # Close log file handles
        for fh in fhs:
            try:
                fh.close()
            except Exception:
                pass
        sys.exit(0)

    # Register signal handlers (both Unix and Windows-friendly)
    signal.signal(signal.SIGINT, shutdown)
    try:
        signal.signal(signal.SIGTERM, shutdown)
    except Exception:
        # SIGTERM may not exist on some Windows builds
        pass

    for pkg, launch_name in entries:
        print(f"Starting launch: package={pkg} file={launch_name}")
        proc, out_f, err_f, out_path, err_path = start_launch(pkg, launch_name, logs_dir)
        if proc is None:
            print(f"Failed to start {pkg}/{launch_name}, see logs for details.")
            try:
                out_f.close()
            except Exception:
                pass
            try:
                err_f.close()
            except Exception:
                pass
            continue
        procs.append(proc)
        fhs.extend([out_f, err_f])
        print(f"  logs: stdout={out_path} stderr={err_path}")

    try:
        # Monitor children: if all exit, master exits.
        while True:
            alive = [p for p in procs if p.poll() is None]
            if not alive:
                print("All launch processes have exited.")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown(signal.SIGINT, None)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
