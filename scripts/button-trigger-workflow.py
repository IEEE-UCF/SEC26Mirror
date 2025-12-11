#!/usr/bin/env python3
import os
import time
import requests
from gpiozero import Button

# Hardware pins
BUTTON_PIN = int(os.environ.get("BTN_PIN", "22"))   # BCM button, default 22

# Gitea configuration (defaults match working curl example)
GITEA_URL = os.environ.get("GITEA_URL", "https://gitea.syndric.org")
OWNER = os.environ.get("GITEA_OWNER", "ieeeucf")
REPO = os.environ.get("GITEA_REPO", "SEC26")
# Use workflow file name without leading .gitea/workflows path, as per API example
WORKFLOW = os.environ.get("GITEA_WORKFLOW", "pi-deploy.yml")
REF = os.environ.get("GITEA_REF", "prod")
TOKEN = os.environ.get("GITEA_TOKEN")

if not TOKEN:
    raise SystemExit("GITEA_TOKEN environment variable is required")

button = Button(BUTTON_PIN, pull_up=True, bounce_time=0.05)


def dispatch_workflow():
    url = f"{GITEA_URL}/api/v1/repos/{OWNER}/{REPO}/actions/workflows/{WORKFLOW}/dispatches"
    payload = {"ref": REF}
    headers = {
        "Authorization": f"token {TOKEN}",
        "Content-Type": "application/json",
    }
    try:
        r = requests.post(url, json=payload, headers=headers, timeout=10)
        r.raise_for_status()
        print(f"✅ Triggered workflow_dispatch for {OWNER}/{REPO} on {REF}")
    except Exception as e:
        print(f"❌ Dispatch error: {e}")
    finally:
        # brief pause to avoid bouncing repeated dispatches
        time.sleep(0.5)


def on_press():
    # Require short hold to avoid accidental triggers
    start = time.time()
    while button.is_pressed and time.time() - start < 0.3:
        time.sleep(0.01)
    if button.is_pressed:
        dispatch_workflow()


button.when_pressed = on_press
print("🔘 Button → Gitea workflow_dispatch listener running")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
