import threading
import os
import yaml
import rclpy
from rclpy.node import Node
from secbot_msgs.msg import ColorTunning
from flask import Flask, render_template_string, request, jsonify
from ament_index_python.packages import get_package_share_directory

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ROS2 HSV Color Tuner</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', sans-serif;
      background: #1a1a2e; color: #eee;
      display: flex; flex-direction: column; align-items: center;
      justify-content: center; min-height: 100vh; gap: 24px; padding: 32px 0;
    }

    .row { display: flex; gap: 24px; align-items: flex-start; flex-wrap: wrap; justify-content: center; }

    .card {
      background: #16213e; border-radius: 16px; padding: 32px 36px;
      width: 340px; box-shadow: 0 8px 32px rgba(0,0,0,0.4);
    }
    .card.wide { width: 706px; max-width: 95vw; }

    h1 { font-size: 1.1rem; font-weight: 600; margin-bottom: 20px; text-align: center; color: #a0c4ff; }

    .color-preview {
      width: 100%; height: 70px; border-radius: 10px; margin-bottom: 16px;
      border: 2px solid #0f3460; transition: background-color 0.15s ease;
    }
    .hsv-label { text-align: center; font-size: 0.9rem; font-weight: 600; letter-spacing: 2px; margin-bottom: 16px; }

    .slider-row { margin-bottom: 14px; }
    .slider-row label { display: flex; justify-content: space-between; font-size: 0.75rem;
      font-weight: 600; margin-bottom: 5px; letter-spacing: 1px; }
    .slider-row label span { color: #aaa; font-weight: 400; }

    input[type=range] { -webkit-appearance: none; width: 100%; height: 6px;
      border-radius: 3px; outline: none; cursor: pointer; }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%;
      background: white; box-shadow: 0 0 4px rgba(0,0,0,0.5); cursor: pointer;
    }

    .h-slider {
      background: linear-gradient(to right,
        hsl(0,100%,50%), hsl(30,100%,50%), hsl(60,100%,50%), hsl(90,100%,50%),
        hsl(120,100%,50%), hsl(150,100%,50%), hsl(180,100%,50%), hsl(210,100%,50%),
        hsl(240,100%,50%), hsl(270,100%,50%), hsl(300,100%,50%), hsl(330,100%,50%),
        hsl(360,100%,50%));
    }
    .s-slider { background: linear-gradient(to right, #888, #ff2222); }
    .v-slider { background: linear-gradient(to right, #000, #ffffff); }

    .preset-bar {
      display: flex; gap: 8px; justify-content: center; flex-wrap: wrap; margin-bottom: 16px;
    }
    .preset-btn {
      padding: 6px 14px; border-radius: 8px; border: 2px solid #334; cursor: pointer;
      font-size: 0.75rem; font-weight: 700; letter-spacing: 1px; text-transform: uppercase;
      background: #1a1a2e; color: #ccc; transition: all 0.15s;
    }
    .preset-btn:hover { border-color: #a0c4ff; color: #fff; }
    .preset-btn.active { border-color: #44cc88; color: #44cc88; }

    .action-bar { display: flex; gap: 12px; justify-content: center; margin-top: 16px; }
    .action-btn {
      padding: 10px 24px; border-radius: 10px; border: none; cursor: pointer;
      font-size: 0.8rem; font-weight: 700; letter-spacing: 1px; text-transform: uppercase;
    }
    .save-btn { background: #44cc88; color: #111; }
    .save-btn:hover { background: #33bb77; }
    .copy-btn { background: #3388cc; color: #fff; }
    .copy-btn:hover { background: #2277bb; }

    .global-status {
      width: 706px; max-width: 95vw; background: #16213e; border-radius: 12px;
      padding: 14px 20px; text-align: center; font-size: 0.7rem; color: #555;
      box-shadow: 0 4px 16px rgba(0,0,0,0.3);
    }
    .global-status.ok  { color: #44cc88; }
    .global-status.err { color: #ff6655; }
    .global-status.info { color: #a0c4ff; }

    .yaml-output {
      background: #0d1117; border: 1px solid #334; border-radius: 8px;
      padding: 12px 16px; font-family: monospace; font-size: 0.8rem;
      color: #88cc88; white-space: pre; margin-top: 12px; display: none;
      max-width: 100%; overflow-x: auto;
    }
  </style>
</head>
<body>

  <!-- Color preset selector -->
  <div class="card wide">
    <h1>COLOR PRESET</h1>
    <div class="preset-bar" id="presetBar"></div>
  </div>

  <div class="row">
    <!-- Card 1: HSV LOW -->
    <div class="card">
      <h1>HSV LOW</h1>
      <div class="color-preview" id="preview1"></div>
      <div class="hsv-label" id="hsvLabel1">H=0 S=0 V=0</div>

      <div class="slider-row">
        <label>HUE <span id="h1Val">0</span></label>
        <input type="range" class="h-slider" id="h1Slider" min="0" max="179" value="0">
      </div>
      <div class="slider-row">
        <label>SATURATION <span id="s1Val">0</span></label>
        <input type="range" class="s-slider" id="s1Slider" min="0" max="255" value="0">
      </div>
      <div class="slider-row">
        <label>VALUE <span id="v1Val">0</span></label>
        <input type="range" class="v-slider" id="v1Slider" min="0" max="255" value="0">
      </div>
    </div>

    <!-- Card 2: HSV HIGH -->
    <div class="card">
      <h1>HSV HIGH</h1>
      <div class="color-preview" id="preview2"></div>
      <div class="hsv-label" id="hsvLabel2">H=0 S=0 V=0</div>

      <div class="slider-row">
        <label>HUE <span id="h2Val">0</span></label>
        <input type="range" class="h-slider" id="h2Slider" min="0" max="179" value="0">
      </div>
      <div class="slider-row">
        <label>SATURATION <span id="s2Val">0</span></label>
        <input type="range" class="s-slider" id="s2Slider" min="0" max="255" value="0">
      </div>
      <div class="slider-row">
        <label>VALUE <span id="v2Val">0</span></label>
        <input type="range" class="v-slider" id="v2Slider" min="0" max="255" value="0">
      </div>
    </div>
  </div>

  <!-- Actions -->
  <div class="card wide">
    <h1>ACTIONS</h1>
    <div class="action-bar">
      <button class="action-btn save-btn" onclick="saveToYaml()">Save to colors.yaml</button>
      <button class="action-btn copy-btn" onclick="copyYaml()">Copy YAML</button>
    </div>
    <div class="yaml-output" id="yamlOutput"></div>
  </div>

  <!-- Status -->
  <div class="global-status" id="globalStatus">Waiting...</div>

  <script>
    const S = {};
    ['h1','s1','v1','h2','s2','v2'].forEach(id => {
      S[id] = document.getElementById(id + 'Slider');
    });

    let publishDebounce;
    let activePreset = null;
    const PRESETS = PRESET_DATA_PLACEHOLDER;

    // Build preset buttons
    const presetBar = document.getElementById('presetBar');
    Object.keys(PRESETS).forEach(name => {
      const btn = document.createElement('button');
      btn.className = 'preset-btn';
      btn.textContent = name;
      btn.dataset.name = name;
      btn.onclick = () => loadPreset(name);
      presetBar.appendChild(btn);
    });

    function loadPreset(name) {
      const p = PRESETS[name];
      if (!p) return;
      activePreset = name;

      S.h1.value = p.color_low[0]; S.s1.value = p.color_low[1]; S.v1.value = p.color_low[2];
      S.h2.value = p.color_high[0]; S.s2.value = p.color_high[1]; S.v2.value = p.color_high[2];

      document.querySelectorAll('.preset-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.name === name);
      });

      updateUI();
      schedulePublish();
    }

    // OpenCV HSV (H: 0-179) to CSS hsl
    function hsvToHsl(h, s, v) {
      // h: 0-179 (OpenCV), s: 0-255, v: 0-255
      const hDeg = h * 2;  // 0-358
      const sF = s / 255;
      const vF = v / 255;
      const l = vF * (1 - sF / 2);
      const sl = (l === 0 || l === 1) ? 0 : (vF - l) / Math.min(l, 1 - l);
      return `hsl(${hDeg}, ${Math.round(sl*100)}%, ${Math.round(l*100)}%)`;
    }

    function updateUI() {
      const h1 = parseInt(S.h1.value), s1 = parseInt(S.s1.value), v1 = parseInt(S.v1.value);
      const h2 = parseInt(S.h2.value), s2 = parseInt(S.s2.value), v2 = parseInt(S.v2.value);

      document.getElementById('h1Val').textContent = h1;
      document.getElementById('s1Val').textContent = s1;
      document.getElementById('v1Val').textContent = v1;
      document.getElementById('h2Val').textContent = h2;
      document.getElementById('s2Val').textContent = s2;
      document.getElementById('v2Val').textContent = v2;

      document.getElementById('hsvLabel1').textContent = `H=${h1} S=${s1} V=${v1}`;
      document.getElementById('hsvLabel2').textContent = `H=${h2} S=${s2} V=${v2}`;

      document.getElementById('preview1').style.backgroundColor = hsvToHsl(h1, s1, v1);
      document.getElementById('preview2').style.backgroundColor = hsvToHsl(h2, s2, v2);
    }

    function schedulePublish() {
      clearTimeout(publishDebounce);
      publishDebounce = setTimeout(publishAll, 60);
    }

    async function publishAll() {
      const h1 = parseInt(S.h1.value), s1 = parseInt(S.s1.value), v1 = parseInt(S.v1.value);
      const h2 = parseInt(S.h2.value), s2 = parseInt(S.s2.value), v2 = parseInt(S.v2.value);

      // Midpoint HSV for confidence scoring
      const hMid = Math.round((h1 + h2) / 2);
      const sMid = Math.round((s1 + s2) / 2);
      const vMid = Math.round((v1 + v2) / 2);

      const payload = {
        low_h: h1, low_s: s1, low_v: v1,
        high_h: h2, high_s: s2, high_v: v2,
        hue: hMid, saturation: sMid, value: vMid,
      };

      const statusEl = document.getElementById('globalStatus');
      try {
        const res = await fetch('/update', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(payload)
        });
        const d = await res.json();
        statusEl.textContent =
          `LOW  H=${d.low_h} S=${d.low_s} V=${d.low_v}  |  ` +
          `HIGH  H=${d.high_h} S=${d.high_s} V=${d.high_v}  |  ` +
          `MID  H=${d.hue} S=${d.saturation} V=${d.value}`;
        statusEl.className = 'global-status ok';
      } catch(e) {
        statusEl.textContent = 'Error publishing to ROS2';
        statusEl.className = 'global-status err';
      }
    }

    function getYamlSnippet() {
      const h1 = parseInt(S.h1.value), s1 = parseInt(S.s1.value), v1 = parseInt(S.v1.value);
      const h2 = parseInt(S.h2.value), s2 = parseInt(S.s2.value), v2 = parseInt(S.v2.value);
      const hMid = Math.round((h1 + h2) / 2);
      const sMid = Math.round((s1 + s2) / 2);
      const vMid = Math.round((v1 + v2) / 2);
      const name = activePreset || 'custom';
      return `  ${name}:\\n` +
             `    color_low:  [${h1}, ${s1}, ${v1}]\\n` +
             `    color_high: [${h2}, ${s2}, ${v2}]\\n` +
             `    hsv_values: [${hMid}, ${sMid}, ${vMid}]`;
    }

    async function saveToYaml() {
      const name = activePreset;
      if (!name) {
        document.getElementById('globalStatus').textContent = 'Select a color preset first!';
        document.getElementById('globalStatus').className = 'global-status err';
        return;
      }
      const h1 = parseInt(S.h1.value), s1 = parseInt(S.s1.value), v1 = parseInt(S.v1.value);
      const h2 = parseInt(S.h2.value), s2 = parseInt(S.s2.value), v2 = parseInt(S.v2.value);
      const hMid = Math.round((h1 + h2) / 2);
      const sMid = Math.round((s1 + s2) / 2);
      const vMid = Math.round((v1 + v2) / 2);

      try {
        const res = await fetch('/save', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            name: name,
            color_low: [h1, s1, v1],
            color_high: [h2, s2, v2],
            hsv_values: [hMid, sMid, vMid],
          })
        });
        const d = await res.json();
        const statusEl = document.getElementById('globalStatus');
        if (d.ok) {
          statusEl.textContent = `Saved '${name}' to colors.yaml!`;
          statusEl.className = 'global-status ok';
        } else {
          statusEl.textContent = `Save failed: ${d.error}`;
          statusEl.className = 'global-status err';
        }
      } catch(e) {
        document.getElementById('globalStatus').textContent = 'Save error: ' + e;
        document.getElementById('globalStatus').className = 'global-status err';
      }
    }

    function copyYaml() {
      const h1 = parseInt(S.h1.value), s1 = parseInt(S.s1.value), v1 = parseInt(S.v1.value);
      const h2 = parseInt(S.h2.value), s2 = parseInt(S.s2.value), v2 = parseInt(S.v2.value);
      const hMid = Math.round((h1 + h2) / 2);
      const sMid = Math.round((s1 + s2) / 2);
      const vMid = Math.round((v1 + v2) / 2);
      const name = activePreset || 'custom';

      const yamlText =
        `  ${name}:\n` +
        `    color_low:  [${h1}, ${s1}, ${v1}]\n` +
        `    color_high: [${h2}, ${s2}, ${v2}]\n` +
        `    hsv_values: [${hMid}, ${sMid}, ${vMid}]`;

      const yamlEl = document.getElementById('yamlOutput');
      yamlEl.textContent = yamlText;
      yamlEl.style.display = 'block';

      navigator.clipboard.writeText(yamlText).then(() => {
        const statusEl = document.getElementById('globalStatus');
        statusEl.textContent = 'YAML copied to clipboard!';
        statusEl.className = 'global-status info';
      });
    }

    // Wire up slider events
    ['h1','s1','v1','h2','s2','v2'].forEach(k => {
      S[k].addEventListener('input', () => { updateUI(); schedulePublish(); });
    });

    // Load first preset on startup
    const firstPreset = Object.keys(PRESETS)[0];
    if (firstPreset) loadPreset(firstPreset);
  </script>
</body>
</html>
"""

_node = None

class GUITunner(Node):
    def __init__(self):
        super().__init__('rgb_slider_gui')
        self.pub = self.create_publisher(ColorTunning, '/color_tunning', 10)
        self.get_logger().info('HSV Tuner node started — publishing on /color_tunning')

        # Load colors.yaml for presets and saving
        pkg_share = get_package_share_directory('secbot_vision')
        self.colors_path = os.path.join(pkg_share, 'config', 'colors.yaml')
        # Also find the source colors.yaml for saving (share dir may be read-only)
        self.colors_source_path = self._find_source_colors_yaml()
        self.get_logger().info(f'colors.yaml (read): {self.colors_path}')
        self.get_logger().info(f'colors.yaml (write): {self.colors_source_path}')

        with open(self.colors_path, 'r') as f:
            self.colors_cfg = yaml.safe_load(f)

    def _find_source_colors_yaml(self):
        """Find the source colors.yaml (not the installed copy) for saving."""
        # Walk up from the share directory to find the source
        # Typical layout: .../install/secbot_vision/share/secbot_vision/config/colors.yaml
        # Source is at:    .../src/secbot_vision/config/colors.yaml
        pkg_share = get_package_share_directory('secbot_vision')
        # Try common source locations
        candidates = [
            os.path.join(pkg_share, '..', '..', '..', '..', 'src', 'secbot_vision', 'config', 'colors.yaml'),
            os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'colors.yaml'),
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'colors.yaml'),
        ]
        for c in candidates:
            p = os.path.normpath(c)
            if os.path.isfile(p):
                return p
        # Fallback to share path
        return self.colors_path

    def get_presets_json(self):
        """Return colors config as JSON for the web UI."""
        import json
        colors = self.colors_cfg.get('colors', {})
        return json.dumps(colors)

    def publish_hsv(self, low_h, low_s, low_v, high_h, high_s, high_v, hue, saturation, value):
        """Publish HSV values using the ColorTunning message.

        Field mapping (repurposed from RGB names):
          low_red   → H low    low_green  → S low    low_blue  → V low
          high_red  → H high   high_green → S high   high_blue → V high
          hue, saturation, value → midpoint HSV
        """
        msg = ColorTunning()
        msg.low_red        = int(low_h)
        msg.low_green      = int(low_s)
        msg.low_blue       = int(low_v)
        msg.high_red       = int(high_h)
        msg.high_green     = int(high_s)
        msg.high_blue      = int(high_v)
        msg.hue            = int(hue)
        msg.saturation     = int(saturation)
        msg.value          = int(value)
        self.pub.publish(msg)
        self.get_logger().info(
            f'LOW H={low_h} S={low_s} V={low_v} | '
            f'HIGH H={high_h} S={high_s} V={high_v} | '
            f'MID H={hue} S={saturation} V={value}'
        )

    def save_color(self, name, color_low, color_high, hsv_values):
        """Save a color entry back to the source colors.yaml."""
        self.colors_cfg['colors'][name] = {
            'color_low': color_low,
            'color_high': color_high,
            'hsv_values': hsv_values,
        }
        with open(self.colors_source_path, 'w') as f:
            yaml.dump(self.colors_cfg, f, default_flow_style=None, sort_keys=False)
        self.get_logger().info(f'Saved {name} to {self.colors_source_path}')


@app.route('/')
def index():
    presets_json = _node.get_presets_json() if _node else '{}'
    html = HTML.replace('PRESET_DATA_PLACEHOLDER', presets_json)
    return render_template_string(html)

@app.route('/update', methods=['POST'])
def update():
    d = request.get_json()
    low_h  = d.get('low_h', 0)
    low_s  = d.get('low_s', 0)
    low_v  = d.get('low_v', 0)
    high_h = d.get('high_h', 0)
    high_s = d.get('high_s', 0)
    high_v = d.get('high_v', 0)
    hue    = d.get('hue', 0)
    sat    = d.get('saturation', 0)
    val    = d.get('value', 0)
    if _node is not None:
        _node.publish_hsv(low_h, low_s, low_v, high_h, high_s, high_v, hue, sat, val)
    return jsonify({
        'low_h': low_h, 'low_s': low_s, 'low_v': low_v,
        'high_h': high_h, 'high_s': high_s, 'high_v': high_v,
        'hue': hue, 'saturation': sat, 'value': val,
    })

@app.route('/save', methods=['POST'])
def save():
    d = request.get_json()
    name = d.get('name', '')
    color_low = d.get('color_low', [0, 0, 0])
    color_high = d.get('color_high', [0, 0, 0])
    hsv_values = d.get('hsv_values', [0, 0, 0])
    if not name:
        return jsonify({'ok': False, 'error': 'No color name provided'})
    if _node is not None:
        try:
            _node.save_color(name, color_low, color_high, hsv_values)
            return jsonify({'ok': True})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)})
    return jsonify({'ok': False, 'error': 'ROS node not ready'})

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = GUITunner()
    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()
    print('\n  ROS2 HSV Color Tuner  ->  http://localhost:5000/\n')
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
