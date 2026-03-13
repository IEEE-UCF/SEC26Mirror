import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from secbot_msgs.msg import ColorTunning  # adjust 'your_package' to your actual package name
from flask import Flask, render_template_string, request, jsonify

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ROS2 RGB/HSV Slider</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', sans-serif;
      background: #1a1a2e; color: #eee;
      display: flex; justify-content: center; align-items: center; min-height: 100vh;
    }
    .card {
      background: #16213e; border-radius: 16px; padding: 36px 40px;
      width: 380px; box-shadow: 0 8px 32px rgba(0,0,0,0.4);
    }
    h1 { font-size: 1.3rem; font-weight: 600; margin-bottom: 28px; text-align: center; color: #a0c4ff; }
    .color-preview {
      width: 100%; height: 90px; border-radius: 10px; margin-bottom: 28px;
      border: 2px solid #0f3460; transition: background-color 0.15s ease;
    }
    .hex-label { text-align: center; font-size: 1rem; font-weight: 600; letter-spacing: 2px; margin-bottom: 28px; }
    .section-title {
      font-size: 0.7rem; font-weight: 700; letter-spacing: 3px; text-transform: uppercase;
      color: #556; margin-bottom: 14px; margin-top: 6px;
      border-top: 1px solid #223; padding-top: 18px;
    }
    .section-title:first-of-type { border-top: none; padding-top: 0; }
    .slider-row { margin-bottom: 18px; }
    .slider-row label { display: flex; justify-content: space-between; font-size: 0.8rem; font-weight: 600; margin-bottom: 6px; letter-spacing: 1px; }
    .slider-row label span { color: #aaa; font-weight: 400; }
    input[type=range] { -webkit-appearance: none; width: 100%; height: 6px; border-radius: 3px; outline: none; cursor: pointer; }
    #rSlider { background: linear-gradient(to right, #111, #ff4444); }
    #gSlider { background: linear-gradient(to right, #111, #44ff44); }
    #bSlider { background: linear-gradient(to right, #111, #4444ff); }
    #hSlider {
      background: linear-gradient(to right,
        hsl(0,100%,50%), hsl(30,100%,50%), hsl(60,100%,50%), hsl(90,100%,50%),
        hsl(120,100%,50%), hsl(150,100%,50%), hsl(180,100%,50%), hsl(210,100%,50%),
        hsl(240,100%,50%), hsl(270,100%,50%), hsl(300,100%,50%), hsl(330,100%,50%),
        hsl(360,100%,50%));
    }
    #sSlider { background: linear-gradient(to right, #888, #ff2222); }
    #vSlider { background: linear-gradient(to right, #000, #ffffff); }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none; width: 18px; height: 18px; border-radius: 50%;
      background: white; box-shadow: 0 0 4px rgba(0,0,0,0.5); cursor: pointer; transition: transform 0.1s;
    }
    input[type=range]::-webkit-slider-thumb:hover { transform: scale(1.2); }
    .status { margin-top: 22px; font-size: 0.75rem; text-align: center; color: #555; }
    .status.ok  { color: #44cc88; }
    .status.err { color: #ff6655; }
  </style>
</head>
<body>
  <div class="card">
    <h1>ROS2 RGB / HSV Slider</h1>
    <div class="color-preview" id="preview" style="background-color:#0000FF"></div>
    <div class="hex-label" id="hexLabel">#0000FF</div>

    <div class="section-title">RGB</div>
    <div class="slider-row">
      <label>RED <span id="rVal">0</span></label>
      <input type="range" id="rSlider" min="0" max="255" value="0">
    </div>
    <div class="slider-row">
      <label>GREEN <span id="gVal">0</span></label>
      <input type="range" id="gSlider" min="0" max="255" value="0">
    </div>
    <div class="slider-row">
      <label>BLUE <span id="bVal">255</span></label>
      <input type="range" id="bSlider" min="0" max="255" value="255">
    </div>

    <div class="section-title">HSV</div>
    <div class="slider-row">
      <label>HUE <span id="hVal">240°</span></label>
      <input type="range" id="hSlider" min="0" max="360" value="240">
    </div>
    <div class="slider-row">
      <label>SATURATION <span id="sVal">100%</span></label>
      <input type="range" id="sSlider" min="0" max="100" value="100">
    </div>
    <div class="slider-row">
      <label>VALUE <span id="vVal">100%</span></label>
      <input type="range" id="vSlider" min="0" max="100" value="100">
    </div>

    <div class="status" id="status">Waiting for input…</div>
  </div>
  <script>
    const rS = document.getElementById('rSlider');
    const gS = document.getElementById('gSlider');
    const bS = document.getElementById('bSlider');
    const hS = document.getElementById('hSlider');
    const sS = document.getElementById('sSlider');
    const vS = document.getElementById('vSlider');
    const preview  = document.getElementById('preview');
    const hexLabel = document.getElementById('hexLabel');
    const status   = document.getElementById('status');
    let debounce, _lock = false;

    function toHex(n) { return Math.round(n).toString(16).padStart(2,'0').toUpperCase(); }

    function rgbToHsv(r, g, b) {
      r /= 255; g /= 255; b /= 255;
      const max = Math.max(r,g,b), min = Math.min(r,g,b), d = max - min;
      let h = 0, s = max === 0 ? 0 : d / max, v = max;
      if (d !== 0) {
        switch (max) {
          case r: h = ((g - b) / d + (g < b ? 6 : 0)) / 6; break;
          case g: h = ((b - r) / d + 2) / 6; break;
          case b: h = ((r - g) / d + 4) / 6; break;
        }
      }
      return [Math.round(h * 360), Math.round(s * 100), Math.round(v * 100)];
    }

    function hsvToRgb(h, s, v) {
      s /= 100; v /= 100;
      const i = Math.floor(h / 60) % 6;
      const f = h / 60 - Math.floor(h / 60);
      const p = v*(1-s), q = v*(1-f*s), t = v*(1-(1-f)*s);
      let r, g, b;
      switch (i) {
        case 0:[r,g,b]=[v,t,p];break; case 1:[r,g,b]=[q,v,p];break;
        case 2:[r,g,b]=[p,v,t];break; case 3:[r,g,b]=[p,q,v];break;
        case 4:[r,g,b]=[t,p,v];break; case 5:[r,g,b]=[v,p,q];break;
      }
      return [Math.round(r*255), Math.round(g*255), Math.round(b*255)];
    }

    function updateHsvTracks(h, s, v) {
      sS.style.background = `linear-gradient(to right, hsl(0,0%,${v/2}%), hsl(${h},100%,${v/2}%))`;
      vS.style.background = `linear-gradient(to right, #000, hsl(${h},${s}%,50%))`;
    }

    function applyColor(hex) {
      preview.style.backgroundColor = hex;
      hexLabel.textContent = hex;
      clearTimeout(debounce);
      debounce = setTimeout(() => publishColor(hex), 60);
    }

    function onRgbSlide() {
      if (_lock) return;
      const r = parseInt(rS.value), g = parseInt(gS.value), b = parseInt(bS.value);
      document.getElementById('rVal').textContent = r;
      document.getElementById('gVal').textContent = g;
      document.getElementById('bVal').textContent = b;
      const [h, s, v] = rgbToHsv(r, g, b);
      _lock = true; hS.value = h; sS.value = s; vS.value = v; _lock = false;
      document.getElementById('hVal').textContent = h + '°';
      document.getElementById('sVal').textContent = s + '%';
      document.getElementById('vVal').textContent = v + '%';
      updateHsvTracks(h, s, v);
      applyColor('#' + toHex(r) + toHex(g) + toHex(b));
    }

    function onHsvSlide() {
      if (_lock) return;
      const h = parseInt(hS.value), s = parseInt(sS.value), v = parseInt(vS.value);
      document.getElementById('hVal').textContent = h + '°';
      document.getElementById('sVal').textContent = s + '%';
      document.getElementById('vVal').textContent = v + '%';
      updateHsvTracks(h, s, v);
      const [r, g, b] = hsvToRgb(h, s, v);
      _lock = true; rS.value = r; gS.value = g; bS.value = b; _lock = false;
      document.getElementById('rVal').textContent = r;
      document.getElementById('gVal').textContent = g;
      document.getElementById('bVal').textContent = b;
      applyColor('#' + toHex(r) + toHex(g) + toHex(b));
    }

    async function publishColor(hex) {
      const r = parseInt(rS.value), g = parseInt(gS.value), b = parseInt(bS.value);
      const h = parseInt(hS.value), s = parseInt(sS.value), v = parseInt(vS.value);
      try {
        const res = await fetch('/update', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({color: hex, r, g, b, h, s, v})
        });
        const data = await res.json();
        status.textContent = `Published: R${data.r} G${data.g} B${data.b} | H${data.h} S${data.s} V${data.v}`;
        status.className = 'status ok';
      } catch(e) {
        status.textContent = 'Error publishing to ROS2';
        status.className = 'status err';
      }
    }

    [rS, gS, bS].forEach(s => s.addEventListener('input', onRgbSlide));
    [hS, sS, vS].forEach(s => s.addEventListener('input', onHsvSlide));
    onRgbSlide();
  </script>
</body>
</html>
"""

_node = None

class GUITunner(Node):
    def __init__(self):
        super().__init__('rgb_slider_gui')
        self.publisher = self.create_publisher(ColorTunning, '/color_tunning', 10)
        self.get_logger().info('GUITunner node started')

    def publish_color(self, r, g, b, h, s, v):
        msg = ColorTunning()
        msg.red        = int(r)
        msg.green      = int(g)
        msg.blue       = int(b)
        msg.hue        = int(h)
        msg.saturation = int(s)
        msg.value      = int(v)
        self.publisher.publish(msg)
        self.get_logger().info(f'r={r} g={g} b={b} h={h} s={s} v={v}')

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/update', methods=['POST'])
def update():
    data = request.get_json()
    r, g, b = data.get('r', 0), data.get('g', 0), data.get('b', 0)
    h, s, v = data.get('h', 0), data.get('s', 0), data.get('v', 0)
    if _node is not None:
        _node.publish_color(r, g, b, h, s, v)
    return jsonify({'r': r, 'g': g, 'b': b, 'h': h, 's': s, 'v': v})

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = GUITunner()
    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()
    print('\n  ROS2 RGB/HSV Slider  ->  http://localhost:5000/\n')
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()