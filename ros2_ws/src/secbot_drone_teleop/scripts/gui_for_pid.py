#!/usr/bin/env python3
"""
Flask-based web GUI for drone PID tuning.

Mirrors the HSV color tuner pattern:
  - Flask serves a single HTML page
  - ROS2 node spins in a daemon thread
  - Browser POSTs param changes to /update -> node calls /mcu_drone/set_param
  - /state endpoint returns live telemetry for the status bar
  - /save endpoint triggers save_config on the drone

Publishes current PID state on /drone_pid_tunning (PIDTunning.msg) after
every change so other nodes can observe.

Run:
    ros2 run <your_pkg> pid_tuner_gui
Then open http://localhost:5001/
"""

import threading
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from flask import Flask, render_template_string, request, jsonify

from mcu_msgs.msg import DroneState
from mcu_msgs.srv import DroneSetParam, DroneTare
from std_msgs.msg import String

try:
    from mcu_msgs.msg import PIDTunning
except ImportError:
    PIDTunning = None  # graceful fallback if msg not yet built

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Default PID values (match DroneConfig.h / build_params() in the TUI)
# ---------------------------------------------------------------------------
DEFAULTS = {
    "roll_rate_kp":    0.003,
    "roll_rate_ki":    0.001,
    "roll_rate_kd":    0.00003,
    "pitch_rate_kp":   0.003,
    "pitch_rate_ki":   0.001,
    "pitch_rate_kd":   0.00003,
    "roll_angle_kp":   5.0,
    "roll_angle_ki":   0.5,
    "roll_angle_kd":   0.0,
    "pitch_angle_kp":  5.0,
    "pitch_angle_ki":  0.5,
    "pitch_angle_kd":  0.0,
    "yaw_rate_kp":     0.005,
    "yaw_rate_ki":     0.001,
    "yaw_rate_kd":     0.0,
    "altitude_kp":     1.2,
    "altitude_ki":     0.15,
    "altitude_kd":     0.0,
    "alt_vel_kp":      0.5,
    "alt_vel_ki":      0.0,
    "alt_vel_kd":      0.0,
}

STATE_NAMES = {
    0: "INIT", 1: "UNARMED", 2: "ARMED", 3: "LAUNCHING",
    4: "VEL_CTRL", 5: "LANDING", 6: "EMERG", 7: "READY",
}

# ---------------------------------------------------------------------------
# HTML
# ---------------------------------------------------------------------------
HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Drone PID Tuner</title>
  <link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    :root {
      --bg:        #0b0f1a;
      --surface:   #111827;
      --border:    #1e2d45;
      --accent:    #00d4ff;
      --accent2:   #ff6b35;
      --accent3:   #39ff88;
      --warn:      #ffcc00;
      --danger:    #ff3355;
      --text:      #c8d8e8;
      --muted:     #4a5a70;
      --mono:      'Share Tech Mono', monospace;
      --ui:        'Rajdhani', sans-serif;
    }

    body {
      font-family: var(--ui);
      background: var(--bg);
      color: var(--text);
      min-height: 100vh;
      padding: 28px 20px 48px;
      background-image:
        radial-gradient(ellipse 80% 40% at 50% -10%, rgba(0,212,255,0.07) 0%, transparent 70%),
        repeating-linear-gradient(0deg, transparent, transparent 39px, rgba(30,45,69,0.3) 40px),
        repeating-linear-gradient(90deg, transparent, transparent 39px, rgba(30,45,69,0.3) 40px);
    }

    .header {
      display: flex; align-items: center; justify-content: space-between;
      max-width: 1120px; margin: 0 auto 28px;
      border-bottom: 1px solid var(--border); padding-bottom: 16px;
    }
    .header-title {
      font-size: 1.5rem; font-weight: 700; letter-spacing: 4px;
      text-transform: uppercase; color: var(--accent);
      text-shadow: 0 0 20px rgba(0,212,255,0.4);
    }
    .header-title span { color: var(--text); opacity: 0.5; }

    .telemetry-bar {
      display: flex; gap: 24px; font-family: var(--mono); font-size: 0.78rem;
    }
    .tel-item { display: flex; flex-direction: column; align-items: center; gap: 2px; }
    .tel-label { color: var(--muted); font-size: 0.65rem; letter-spacing: 2px; }
    .tel-value { color: var(--accent); font-size: 0.9rem; }
    .tel-value.state { color: var(--accent3); }

    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 20px;
      max-width: 1120px;
      margin: 0 auto 20px;
    }
    @media (max-width: 760px) { .grid { grid-template-columns: 1fr; } }

    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 24px 28px;
      position: relative;
      overflow: hidden;
    }
    .card::before {
      content: '';
      position: absolute; top: 0; left: 0; right: 0; height: 2px;
      background: linear-gradient(90deg, transparent, var(--card-accent, var(--accent)), transparent);
    }
    .card.roll   { --card-accent: #00d4ff; }
    .card.pitch  { --card-accent: #ff6b35; }
    .card.yaw    { --card-accent: #39ff88; }
    .card.alt    { --card-accent: #ffcc00; }

    .card-title {
      font-size: 0.7rem; font-weight: 700; letter-spacing: 4px;
      text-transform: uppercase; margin-bottom: 20px;
      display: flex; align-items: center; gap: 10px;
    }
    .card-title .dot {
      width: 8px; height: 8px; border-radius: 50%;
      background: var(--card-accent, var(--accent));
      box-shadow: 0 0 8px var(--card-accent, var(--accent));
    }
    .card-subtitle {
      font-size: 0.6rem; letter-spacing: 2px; color: var(--muted);
      margin-left: auto; font-weight: 400;
    }

    .loop-section { margin-bottom: 20px; }
    .loop-section:last-child { margin-bottom: 0; }
    .loop-label {
      font-size: 0.6rem; letter-spacing: 3px; color: var(--muted);
      text-transform: uppercase; margin-bottom: 12px;
      border-bottom: 1px solid var(--border); padding-bottom: 6px;
    }

    .pid-row {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 12px;
    }

    .pid-field { display: flex; flex-direction: column; gap: 5px; }
    .pid-field label {
      font-size: 0.6rem; letter-spacing: 3px; color: var(--muted);
      text-transform: uppercase;
    }

    .pid-input-wrap { position: relative; }
    .pid-input {
      width: 100%; background: #0b0f1a;
      border: 1px solid var(--border);
      border-radius: 6px; padding: 8px 10px;
      font-family: var(--mono); font-size: 0.82rem;
      color: var(--text); outline: none;
      transition: border-color 0.15s, box-shadow 0.15s;
    }
    .pid-input:focus {
      border-color: var(--card-accent, var(--accent));
      box-shadow: 0 0 0 2px rgba(0,212,255,0.1);
    }
    .pid-input.changed { border-color: var(--warn); color: var(--warn); }
    .pid-input.pending { border-color: var(--accent3); animation: pulse 0.8s infinite; }

    @keyframes pulse {
      0%, 100% { box-shadow: 0 0 0 0 rgba(57,255,136,0.4); }
      50%       { box-shadow: 0 0 0 4px rgba(57,255,136,0); }
    }

    .step-btns {
      display: flex; gap: 4px; margin-top: 4px;
    }
    .step-btn {
      flex: 1; padding: 3px 0;
      background: var(--border); border: none; border-radius: 4px;
      color: var(--text); font-family: var(--mono); font-size: 0.7rem;
      cursor: pointer; transition: background 0.1s;
    }
    .step-btn:hover { background: var(--card-accent, var(--accent)); color: #000; }

    .card.wide { grid-column: 1 / -1; }

    .action-bar {
      display: flex; gap: 12px; flex-wrap: wrap; justify-content: center;
      margin-bottom: 16px;
    }
    .action-btn {
      padding: 10px 28px; border-radius: 8px; border: none; cursor: pointer;
      font-family: var(--ui); font-size: 0.75rem; font-weight: 700;
      letter-spacing: 2px; text-transform: uppercase;
      transition: opacity 0.15s, transform 0.1s;
    }
    .action-btn:hover  { opacity: 0.85; }
    .action-btn:active { transform: scale(0.97); }
    .btn-arm     { background: var(--accent3); color: #000; }
    .btn-disarm  { background: var(--muted);   color: #fff; }
    .btn-ready   { background: var(--accent);  color: #000; }
    .btn-takeoff { background: var(--warn);    color: #000; }
    .btn-land    { background: var(--accent2); color: #fff; }
    .btn-save    { background: var(--accent);  color: #000; }
    .btn-reset   { background: var(--border);  color: var(--text); }
    .btn-danger  { background: var(--danger);  color: #fff; }
    .btn-copy    { background: #1e3a5f;        color: var(--accent); border: 1px solid var(--accent); }

    .takeoff-row {
      display: flex; gap: 8px; align-items: center; justify-content: center;
      margin-top: 12px;
    }
    .takeoff-row label { font-size: 0.7rem; letter-spacing: 2px; color: var(--muted); }
    .takeoff-row input {
      width: 80px; background: #0b0f1a; border: 1px solid var(--border);
      border-radius: 6px; padding: 6px 10px;
      font-family: var(--mono); font-size: 0.85rem; color: var(--text); outline: none;
    }
    .takeoff-row input:focus { border-color: var(--warn); }

    .status-bar {
      max-width: 1120px; margin: 0 auto;
      background: var(--surface); border: 1px solid var(--border);
      border-radius: 10px; padding: 12px 20px;
      font-family: var(--mono); font-size: 0.72rem;
      color: var(--muted); text-align: center;
      transition: color 0.3s;
    }
    .status-bar.ok   { color: var(--accent3); }
    .status-bar.err  { color: var(--danger); }
    .status-bar.info { color: var(--accent); }

    /* ── Confirm bar ── */
    .confirm-bar {
      position: fixed; bottom: 0; left: 0; right: 0; z-index: 100;
      background: #0d1a0d;
      border-top: 2px solid var(--accent3);
      padding: 14px 32px;
      display: flex; align-items: center; gap: 20px;
      transform: translateY(100%);
      transition: transform 0.25s cubic-bezier(0.34, 1.56, 0.64, 1);
      box-shadow: 0 -8px 40px rgba(57,255,136,0.15);
    }
    .confirm-bar.visible { transform: translateY(0); }

    .confirm-bar-label {
      font-family: var(--mono); font-size: 0.75rem;
      color: var(--accent3); flex: 1;
      white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
    }
    .confirm-bar-label strong { color: #fff; }

    .btn-confirm {
      padding: 10px 32px; border-radius: 8px; border: none; cursor: pointer;
      font-family: var(--ui); font-size: 0.8rem; font-weight: 700;
      letter-spacing: 3px; text-transform: uppercase;
      background: var(--accent3); color: #000;
      box-shadow: 0 0 20px rgba(57,255,136,0.4);
      transition: opacity 0.15s, transform 0.1s, box-shadow 0.15s;
      white-space: nowrap;
    }
    .btn-confirm:hover  { opacity: 0.9; box-shadow: 0 0 30px rgba(57,255,136,0.6); }
    .btn-confirm:active { transform: scale(0.97); }

    .btn-discard {
      padding: 10px 20px; border-radius: 8px; cursor: pointer;
      font-family: var(--ui); font-size: 0.8rem; font-weight: 700;
      letter-spacing: 2px; text-transform: uppercase;
      background: transparent; color: var(--muted);
      border: 1px solid var(--border);
      transition: color 0.15s, border-color 0.15s;
      white-space: nowrap;
    }
    .btn-discard:hover { color: var(--danger); border-color: var(--danger); }

    .pending-count {
      font-family: var(--mono); font-size: 0.7rem; color: var(--warn);
      background: rgba(255,204,0,0.1); border: 1px solid rgba(255,204,0,0.3);
      border-radius: 20px; padding: 3px 10px; white-space: nowrap;
    }
  </style>
</head>
<body>

  <header class="header">
    <div class="header-title">DRONE <span>/</span> PID TUNER</div>
    <div class="telemetry-bar">
      <div class="tel-item"><span class="tel-label">STATE</span><span class="tel-value state" id="telState">—</span></div>
      <div class="tel-item"><span class="tel-label">ROLL</span><span class="tel-value" id="telRoll">0.0°</span></div>
      <div class="tel-item"><span class="tel-label">PITCH</span><span class="tel-value" id="telPitch">0.0°</span></div>
      <div class="tel-item"><span class="tel-label">YAW</span><span class="tel-value" id="telYaw">0.0°</span></div>
      <div class="tel-item"><span class="tel-label">ALT</span><span class="tel-value" id="telAlt">0.00m</span></div>
    </div>
  </header>

  <div class="grid">

    <div class="card roll">
      <div class="card-title">
        <span class="dot"></span> ROLL
        <span class="card-subtitle">INNER + OUTER CASCADE</span>
      </div>
      <div class="loop-section">
        <div class="loop-label">Rate Loop (inner)</div>
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="roll_rate_kp" value="0.003">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('roll_rate_kp', -1)">−</button>
              <button class="step-btn" onclick="step('roll_rate_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="roll_rate_ki" value="0.001">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('roll_rate_ki', -1)">−</button>
              <button class="step-btn" onclick="step('roll_rate_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="roll_rate_kd" value="0.00003">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('roll_rate_kd', -1)">−</button>
              <button class="step-btn" onclick="step('roll_rate_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
      <div class="loop-section">
        <div class="loop-label">Angle Loop (outer)</div>
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="roll_angle_kp" value="5.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('roll_angle_kp', -1)">−</button>
              <button class="step-btn" onclick="step('roll_angle_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="roll_angle_ki" value="0.5">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('roll_angle_ki', -1)">−</button>
              <button class="step-btn" onclick="step('roll_angle_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="roll_angle_kd" value="0.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('roll_angle_kd', -1)">−</button>
              <button class="step-btn" onclick="step('roll_angle_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="card pitch">
      <div class="card-title">
        <span class="dot"></span> PITCH
        <span class="card-subtitle">INNER + OUTER CASCADE</span>
      </div>
      <div class="loop-section">
        <div class="loop-label">Rate Loop (inner)</div>
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="pitch_rate_kp" value="0.003">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('pitch_rate_kp', -1)">−</button>
              <button class="step-btn" onclick="step('pitch_rate_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="pitch_rate_ki" value="0.001">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('pitch_rate_ki', -1)">−</button>
              <button class="step-btn" onclick="step('pitch_rate_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="pitch_rate_kd" value="0.00003">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('pitch_rate_kd', -1)">−</button>
              <button class="step-btn" onclick="step('pitch_rate_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
      <div class="loop-section">
        <div class="loop-label">Angle Loop (outer)</div>
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="pitch_angle_kp" value="5.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('pitch_angle_kp', -1)">−</button>
              <button class="step-btn" onclick="step('pitch_angle_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="pitch_angle_ki" value="0.5">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('pitch_angle_ki', -1)">−</button>
              <button class="step-btn" onclick="step('pitch_angle_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="pitch_angle_kd" value="0.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('pitch_angle_kd', -1)">−</button>
              <button class="step-btn" onclick="step('pitch_angle_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="card yaw">
      <div class="card-title">
        <span class="dot"></span> YAW RATE
        <span class="card-subtitle">RATE ONLY</span>
      </div>
      <div class="loop-section">
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="yaw_rate_kp" value="0.005">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('yaw_rate_kp', -1)">−</button>
              <button class="step-btn" onclick="step('yaw_rate_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="yaw_rate_ki" value="0.001">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('yaw_rate_ki', -1)">−</button>
              <button class="step-btn" onclick="step('yaw_rate_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="yaw_rate_kd" value="0.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('yaw_rate_kd', -1)">−</button>
              <button class="step-btn" onclick="step('yaw_rate_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="card alt">
      <div class="card-title">
        <span class="dot"></span> ALTITUDE
        <span class="card-subtitle">POSITION + VELOCITY CASCADE</span>
      </div>
      <div class="loop-section">
        <div class="loop-label">Position Loop</div>
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="altitude_kp" value="1.2">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('altitude_kp', -1)">−</button>
              <button class="step-btn" onclick="step('altitude_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="altitude_ki" value="0.15">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('altitude_ki', -1)">−</button>
              <button class="step-btn" onclick="step('altitude_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="altitude_kd" value="0.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('altitude_kd', -1)">−</button>
              <button class="step-btn" onclick="step('altitude_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
      <div class="loop-section">
        <div class="loop-label">Velocity Loop</div>
        <div class="pid-row">
          <div class="pid-field">
            <label>KP</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="alt_vel_kp" value="0.5">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('alt_vel_kp', -1)">−</button>
              <button class="step-btn" onclick="step('alt_vel_kp', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KI</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="alt_vel_ki" value="0.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('alt_vel_ki', -1)">−</button>
              <button class="step-btn" onclick="step('alt_vel_ki', +1)">+</button>
            </div>
          </div>
          <div class="pid-field">
            <label>KD</label>
            <div class="pid-input-wrap">
              <input class="pid-input" type="number" step="any" id="alt_vel_kd" value="0.0">
            </div>
            <div class="step-btns">
              <button class="step-btn" onclick="step('alt_vel_kd', -1)">−</button>
              <button class="step-btn" onclick="step('alt_vel_kd', +1)">+</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="card wide">
      <div class="card-title"><span class="dot" style="background:var(--text);box-shadow:none"></span> FLIGHT CONTROLS</div>
      <div class="action-bar">
        <button class="action-btn btn-arm"     onclick="sendCmd('arm')">ARM</button>
        <button class="action-btn btn-disarm"  onclick="sendCmd('disarm')">DISARM</button>
        <button class="action-btn btn-ready"   onclick="sendCmd('ready')">READY</button>
        <button class="action-btn btn-takeoff" onclick="sendTakeoff()">TAKEOFF</button>
        <button class="action-btn btn-land"    onclick="sendCmd('land')">LAND</button>
        <button class="action-btn btn-save"    onclick="sendCmd('save')">SAVE CONFIG</button>
        <button class="action-btn btn-copy"    onclick="copyRoll()">COPY ROLL→PITCH</button>
        <button class="action-btn btn-reset"   onclick="resetAll()">RESET ALL</button>
        <button class="action-btn btn-danger"  onclick="sendCmd('disarm')" title="Emergency disarm">EMERG DISARM</button>
      </div>
      <div class="takeoff-row">
        <label>TAKEOFF ALT (m)</label>
        <input type="number" id="takeoffAlt" value="1.0" min="0.1" max="10" step="0.1">
      </div>
    </div>

  </div>

  <div class="status-bar" id="statusBar">Connecting to drone...</div>

  <!-- Confirm bar — slides up when there are staged changes -->
  <div class="confirm-bar" id="confirmBar">
    <div class="confirm-bar-label" id="confirmLabel">Staged changes ready to send</div>
    <span class="pending-count" id="pendingCount">0 changes</span>
    <button class="btn-discard" onclick="discardChanges()">DISCARD</button>
    <button class="btn-confirm" onclick="confirmChanges()">✓ CONFIRM — SEND TO DRONE</button>
  </div>

<script>
  const PARAM_NAMES = [
    'roll_rate_kp','roll_rate_ki','roll_rate_kd',
    'pitch_rate_kp','pitch_rate_ki','pitch_rate_kd',
    'roll_angle_kp','roll_angle_ki','roll_angle_kd',
    'pitch_angle_kp','pitch_angle_ki','pitch_angle_kd',
    'yaw_rate_kp','yaw_rate_ki','yaw_rate_kd',
    'altitude_kp','altitude_ki','altitude_kd',
    'alt_vel_kp','alt_vel_ki','alt_vel_kd',
  ];

  const DEFAULTS = DEFAULTS_PLACEHOLDER;

  const STEP_FRACTIONS = {
    roll_rate_kp: 0.1, roll_rate_ki: 0.1, roll_rate_kd: 0.1,
    pitch_rate_kp: 0.1, pitch_rate_ki: 0.1, pitch_rate_kd: 0.1,
    roll_angle_kp: 0.05, roll_angle_ki: 0.05, roll_angle_kd: 0.05,
    pitch_angle_kp: 0.05, pitch_angle_ki: 0.05, pitch_angle_kd: 0.05,
    yaw_rate_kp: 0.1, yaw_rate_ki: 0.1, yaw_rate_kd: 0.1,
    altitude_kp: 0.05, altitude_ki: 0.05, altitude_kd: 0.05,
    alt_vel_kp: 0.05, alt_vel_ki: 0.05, alt_vel_kd: 0.05,
  };

  function getStep(name) {
    const def = DEFAULTS[name];
    const frac = STEP_FRACTIONS[name] || 0.1;
    return def !== 0 ? Math.abs(def) * frac : 0.001;
  }

  // ── Staged changes: name -> value, not yet sent to drone ──
  const staged = {};

  function updateConfirmBar() {
    const bar   = document.getElementById('confirmBar');
    const label = document.getElementById('confirmLabel');
    const count = document.getElementById('pendingCount');
    const keys  = Object.keys(staged);
    if (keys.length === 0) {
      bar.classList.remove('visible');
      return;
    }
    bar.classList.add('visible');
    count.textContent = `${keys.length} change${keys.length > 1 ? 's' : ''}`;
    // Show last changed param in label
    const last = keys[keys.length - 1];
    label.innerHTML = `Staged: <strong>${last}</strong> = <strong>${staged[last]}</strong>  — confirm to send all to drone`;
  }

  function stageChange(name, value) {
    staged[name] = value;
    const el = document.getElementById(name);
    if (el) {
      el.classList.add('changed');
      el.classList.remove('pending');
    }
    updateConfirmBar();
  }

  async function confirmChanges() {
    const keys = Object.keys(staged);
    if (keys.length === 0) return;
    const statusEl = document.getElementById('statusBar');
    statusEl.textContent = `Sending ${keys.length} change(s) to drone...`;
    statusEl.className = 'status-bar info';

    let ok = 0, fail = 0;
    for (const name of keys) {
      const value = staged[name];
      const el = document.getElementById(name);
      if (el) el.classList.add('pending');
      try {
        const res = await fetch('/update_param', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({ param_name: name, value })
        });
        const d = await res.json();
        if (el) { el.classList.remove('pending'); el.classList.remove('changed'); }
        if (d.ok) { ok++; delete staged[name]; }
        else { fail++; }
      } catch(e) {
        if (el) el.classList.remove('pending');
        fail++;
      }
    }
    updateConfirmBar();
    if (fail === 0) {
      statusEl.textContent = `✓ ${ok} param(s) sent to drone successfully`;
      statusEl.className = 'status-bar ok';
    } else {
      statusEl.textContent = `⚠ ${ok} sent, ${fail} failed`;
      statusEl.className = 'status-bar err';
    }
  }

  function discardChanges() {
    const keys = Object.keys(staged);
    keys.forEach(name => {
      const el = document.getElementById(name);
      if (el) {
        // Revert display value to last confirmed (DEFAULTS as fallback)
        el.value = DEFAULTS[name];
        el.classList.remove('changed');
      }
      delete staged[name];
    });
    updateConfirmBar();
    const statusEl = document.getElementById('statusBar');
    statusEl.textContent = 'Staged changes discarded.';
    statusEl.className = 'status-bar info';
  }

  PARAM_NAMES.forEach(name => {
    const el = document.getElementById(name);
    if (!el) return;
    el.addEventListener('change', () => {
      const v = parseFloat(el.value);
      if (!isNaN(v)) stageChange(name, v);
    });
    el.addEventListener('keydown', e => {
      if (e.key === 'Enter') {
        const v = parseFloat(el.value);
        if (!isNaN(v)) stageChange(name, v);
      }
    });
    el.addEventListener('input', () => {
      el.classList.toggle('changed', parseFloat(el.value) !== DEFAULTS[name]);
    });
  });

  function step(name, dir) {
    const el = document.getElementById(name);
    const s = getStep(name);
    let v = parseFloat(el.value) + dir * s;
    const decimals = s < 0.001 ? 6 : s < 0.01 ? 5 : s < 0.1 ? 4 : 3;
    v = parseFloat(v.toFixed(decimals));
    el.value = v;
    stageChange(name, v);
  }

  async function sendCmd(cmd) {
    const statusEl = document.getElementById('statusBar');
    try {
      const res = await fetch('/cmd', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ cmd })
      });
      const d = await res.json();
      statusEl.textContent = `${cmd.toUpperCase()}: ${d.message}`;
      statusEl.className = d.ok ? 'status-bar ok' : 'status-bar err';
    } catch(e) {
      statusEl.textContent = `CMD error: ${e}`;
      statusEl.className = 'status-bar err';
    }
  }

  async function sendTakeoff() {
    const alt = parseFloat(document.getElementById('takeoffAlt').value) || 1.0;
    const statusEl = document.getElementById('statusBar');
    try {
      const res = await fetch('/cmd', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ cmd: 'takeoff', altitude: alt })
      });
      const d = await res.json();
      statusEl.textContent = `TAKEOFF ${alt}m: ${d.message}`;
      statusEl.className = d.ok ? 'status-bar ok' : 'status-bar err';
    } catch(e) {
      statusEl.textContent = `TAKEOFF error: ${e}`;
      statusEl.className = 'status-bar err';
    }
  }

  function copyRoll() {
    const pairs = [
      ['roll_rate_kp', 'pitch_rate_kp'],
      ['roll_rate_ki', 'pitch_rate_ki'],
      ['roll_rate_kd', 'pitch_rate_kd'],
      ['roll_angle_kp', 'pitch_angle_kp'],
      ['roll_angle_ki', 'pitch_angle_ki'],
      ['roll_angle_kd', 'pitch_angle_kd'],
    ];
    pairs.forEach(([src, dst]) => {
      const srcEl = document.getElementById(src);
      const dstEl = document.getElementById(dst);
      const v = parseFloat(srcEl.value);
      dstEl.value = v;
      stageChange(dst, v);
    });
    document.getElementById('statusBar').textContent = 'Roll gains staged → pitch. Press CONFIRM to send.';
    document.getElementById('statusBar').className = 'status-bar info';
  }

  function resetAll() {
    PARAM_NAMES.forEach(name => {
      const el = document.getElementById(name);
      if (!el) return;
      el.value = DEFAULTS[name];
      stageChange(name, DEFAULTS[name]);
    });
    document.getElementById('statusBar').textContent = 'All gains reset to defaults — staged. Press CONFIRM to send.';
    document.getElementById('statusBar').className = 'status-bar info';
  }

  const STATE_NAMES = {0:"INIT",1:"UNARMED",2:"ARMED",3:"LAUNCHING",4:"VEL_CTRL",5:"LANDING",6:"EMERG",7:"READY"};

  async function pollTelemetry() {
    try {
      const res = await fetch('/state');
      const d = await res.json();
      document.getElementById('telState').textContent  = STATE_NAMES[d.state] || `?${d.state}`;
      document.getElementById('telRoll').textContent   = d.roll.toFixed(1)  + '°';
      document.getElementById('telPitch').textContent  = d.pitch.toFixed(1) + '°';
      document.getElementById('telYaw').textContent    = d.yaw.toFixed(1)   + '°';
      document.getElementById('telAlt').textContent    = d.alt.toFixed(2)   + 'm';

      if (d.config) {
        Object.entries(d.config).forEach(([name, val]) => {
          const el = document.getElementById(name);
          if (el && document.activeElement !== el) {
            el.value = val;
            el.classList.toggle('changed', val !== DEFAULTS[name]);
          }
        });
      }
    } catch(e) { /* no telemetry yet */ }
  }

  setInterval(pollTelemetry, 150);
  pollTelemetry();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class PidTunerGuiNode(Node):
    def __init__(self):
        super().__init__("drone_pid_tuner_gui")

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_sub = self.create_subscription(
            DroneState, "/mcu_drone/state", self._state_cb, best_effort
        )
        self.debug_sub = self.create_subscription(
            String, "/mcu_drone/debug", self._debug_cb, best_effort
        )

        self.set_param_cli  = self.create_client(DroneSetParam, "/mcu_drone/set_param")
        self.arm_cli        = self.create_client(DroneTare,     "/mcu_drone/arm")
        self.takeoff_cli    = self.create_client(DroneTare,     "/mcu_drone/takeoff")
        self.land_cli       = self.create_client(DroneTare,     "/mcu_drone/land")
        self.tare_cli       = self.create_client(DroneTare,     "/mcu_drone/tare")
        self.save_cli       = self.create_client(DroneTare,     "/mcu_drone/save_config")
        self.ready_cli      = self.create_client(DroneTare,     "/mcu_drone/ready_for_takeoff")
        self.get_config_cli = self.create_client(DroneTare,     "/mcu_drone/get_config")

        # Publisher for PIDTunning message
        if PIDTunning is not None:
            self.pid_pub = self.create_publisher(PIDTunning, "/drone_pid_tunning", 10)
        else:
            self.pid_pub = None
            self.get_logger().warn("PIDTunning message not found — skipping publisher")

        self._lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alt = 0.0
        self.state = 1
        self.last_result = "Waiting for drone..."
        self.pending_config = {}
        self._config_requested = False

        self.current_gains = dict(DEFAULTS)

    # ── Telemetry ──
    def _state_cb(self, msg: DroneState):
        should_request = False
        with self._lock:
            self.roll  = msg.roll
            self.pitch = msg.pitch
            self.yaw   = msg.yaw
            self.alt   = msg.altitude
            self.state = msg.state
            if not self._config_requested:
                self._config_requested = True
                should_request = True
        if should_request:
            self._request_config()

    def _debug_cb(self, msg: String):
        with self._lock:
            text = msg.data
            if text.startswith("CONFIG:"):
                for pair in text[7:].split(","):
                    if "=" in pair:
                        name, val_str = pair.split("=", 1)
                        try:
                            self.pending_config[name.strip()] = float(val_str.strip())
                        except ValueError:
                            pass
            else:
                self.last_result = text

    def get_telemetry(self):
        with self._lock:
            cfg = None
            if self.pending_config:
                cfg = self.pending_config.copy()
                self.pending_config.clear()
            return {
                "roll": self.roll, "pitch": self.pitch,
                "yaw": self.yaw, "alt": self.alt,
                "state": self.state, "last_result": self.last_result,
                "config": cfg,
            }

    def _request_config(self):
        req = DroneTare.Request()
        future = self.get_config_cli.call_async(req)
        future.add_done_callback(self._simple_done("GET_CONFIG"))

    # ── Service calls ──
    def send_param(self, name: str, value: float):
        req = DroneSetParam.Request()
        req.param_name = name
        req.value = float(value)
        future = self.set_param_cli.call_async(req)
        future.add_done_callback(lambda f, n=name, v=value: self._param_done(f, n, v))

    def _param_done(self, future, name, value):
        try:
            resp = future.result()
            with self._lock:
                self.last_result = resp.message
                self.current_gains[name] = value
            self._publish_gains()
            return True, resp.message
        except Exception as e:
            with self._lock:
                self.last_result = str(e)
            return False, str(e)

    def _publish_gains(self):
        """Publish PIDTunning message and log only the published field values."""
        if self.pid_pub is None or PIDTunning is None:
            return

        g = self.current_gains
        msg = PIDTunning()

        # Populate every field from current_gains
        for field in [
            "roll_rate_kp",  "roll_rate_ki",  "roll_rate_kd",
            "pitch_rate_kp", "pitch_rate_ki", "pitch_rate_kd",
            "roll_angle_kp", "roll_angle_ki", "roll_angle_kd",
            "pitch_angle_kp","pitch_angle_ki","pitch_angle_kd",
            "yaw_rate_kp",   "yaw_rate_ki",   "yaw_rate_kd",
            "altitude_kp",   "altitude_ki",   "altitude_kd",
            "alt_vel_kp",    "alt_vel_ki",    "alt_vel_kd",
        ]:
            setattr(msg, field, float(g.get(field, 0.0)))

        self.pid_pub.publish(msg)

        # ── Log only the published message fields ──
        self.get_logger().info(
            "[PIDTunning published]\n"
            f"  roll_rate   kp={msg.roll_rate_kp}  ki={msg.roll_rate_ki}  kd={msg.roll_rate_kd}\n"
            f"  pitch_rate  kp={msg.pitch_rate_kp}  ki={msg.pitch_rate_ki}  kd={msg.pitch_rate_kd}\n"
            f"  roll_angle  kp={msg.roll_angle_kp}  ki={msg.roll_angle_ki}  kd={msg.roll_angle_kd}\n"
            f"  pitch_angle kp={msg.pitch_angle_kp}  ki={msg.pitch_angle_ki}  kd={msg.pitch_angle_kd}\n"
            f"  yaw_rate    kp={msg.yaw_rate_kp}  ki={msg.yaw_rate_ki}  kd={msg.yaw_rate_kd}\n"
            f"  altitude    kp={msg.altitude_kp}  ki={msg.altitude_ki}  kd={msg.altitude_kd}\n"
            f"  alt_vel     kp={msg.alt_vel_kp}  ki={msg.alt_vel_ki}  kd={msg.alt_vel_kd}"
        )

    def _simple_done(self, label: str):
        def cb(future):
            try:
                resp = future.result()
                with self._lock:
                    self.last_result = f"{label}: {resp.message}"
            except Exception as e:
                with self._lock:
                    self.last_result = f"{label} ERR: {e}"
        return cb

    def send_arm(self, arm: bool):
        from mcu_msgs.srv import DroneArm
        req = DroneArm.Request()
        req.arm = arm
        future = self.arm_cli.call_async(req)
        future.add_done_callback(self._simple_done("ARM" if arm else "DISARM"))

    def send_takeoff(self, altitude: float = 1.0):
        from mcu_msgs.srv import DroneTakeoff
        req = DroneTakeoff.Request()
        req.target_altitude = altitude
        future = self.takeoff_cli.call_async(req)
        future.add_done_callback(self._simple_done("TAKEOFF"))

    def send_land(self):
        from mcu_msgs.srv import DroneLand
        req = DroneLand.Request()
        future = self.land_cli.call_async(req)
        future.add_done_callback(self._simple_done("LAND"))

    def send_save(self):
        req = DroneTare.Request()
        future = self.save_cli.call_async(req)
        future.add_done_callback(self._simple_done("SAVE"))

    def send_ready(self):
        req = DroneTare.Request()
        future = self.ready_cli.call_async(req)
        future.add_done_callback(self._simple_done("READY"))


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------
_node: PidTunerGuiNode = None


@app.route("/")
def index():
    defaults_json = json.dumps(DEFAULTS)
    html = HTML.replace("DEFAULTS_PLACEHOLDER", defaults_json)
    return render_template_string(html)


@app.route("/state")
def state():
    if _node is None:
        return jsonify({"roll": 0, "pitch": 0, "yaw": 0, "alt": 0, "state": 0, "config": None})
    return jsonify(_node.get_telemetry())


@app.route("/update_param", methods=["POST"])
def update_param():
    d = request.get_json()
    name  = d.get("param_name", "")
    value = d.get("value", 0.0)
    if not name or name not in DEFAULTS:
        return jsonify({"ok": False, "error": f"Unknown param: {name}"})
    if _node is None:
        return jsonify({"ok": False, "error": "ROS node not ready"})
    _node.send_param(name, float(value))
    return jsonify({"ok": True, "message": f"sent {name}={value}"})


@app.route("/cmd", methods=["POST"])
def cmd():
    d = request.get_json()
    c = d.get("cmd", "")
    if _node is None:
        return jsonify({"ok": False, "message": "ROS node not ready"})
    if   c == "arm":     _node.send_arm(True)
    elif c == "disarm":  _node.send_arm(False)
    elif c == "ready":   _node.send_ready()
    elif c == "land":    _node.send_land()
    elif c == "save":    _node.send_save()
    elif c == "takeoff":
        alt = float(d.get("altitude", 1.0))
        _node.send_takeoff(alt)
    else:
        return jsonify({"ok": False, "message": f"Unknown command: {c}"})
    return jsonify({"ok": True, "message": f"{c} sent"})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = PidTunerGuiNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()

    print("\n  Drone PID Tuner GUI  ->  http://localhost:5001/\n")
    try:
        app.run(host="0.0.0.0", port=5001, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()