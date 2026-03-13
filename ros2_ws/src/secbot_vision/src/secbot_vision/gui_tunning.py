import threading
import rclpy
from rclpy.node import Node
from secbot_msgs.msg import ColorTunning
from flask import Flask, render_template_string, request, jsonify

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ROS2 Color Tuner</title>
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

    h1 { font-size: 1.1rem; font-weight: 600; margin-bottom: 20px; text-align: center; color: #a0c4ff; }

    .color-preview {
      width: 100%; height: 70px; border-radius: 10px; margin-bottom: 16px;
      border: 2px solid #0f3460; transition: background-color 0.15s ease;
    }
    .hex-label { text-align: center; font-size: 0.9rem; font-weight: 600; letter-spacing: 2px; margin-bottom: 4px; }
    .bgr-label { text-align: center; font-size: 0.65rem; color: #556; letter-spacing: 1px; margin-bottom: 16px; }

    .section-title {
      font-size: 0.65rem; font-weight: 700; letter-spacing: 3px; text-transform: uppercase;
      color: #556; margin-bottom: 12px; border-top: 1px solid #223; padding-top: 14px;
    }
    .section-title.first { border-top: none; padding-top: 0; }

    .slider-row { margin-bottom: 14px; }
    .slider-row label { display: flex; justify-content: space-between; font-size: 0.75rem;
      font-weight: 600; margin-bottom: 5px; letter-spacing: 1px; }
    .slider-row label span { color: #aaa; font-weight: 400; }

    input[type=range] { -webkit-appearance: none; width: 100%; height: 6px;
      border-radius: 3px; outline: none; cursor: pointer; }

    /* RGB sliders */
    #r1Slider { background: linear-gradient(to right, #111, #ff4444); }
    #g1Slider { background: linear-gradient(to right, #111, #44ff44); }
    #b1Slider { background: linear-gradient(to right, #111, #4444ff); }
    #r2Slider { background: linear-gradient(to right, #111, #ff4444); }
    #g2Slider { background: linear-gradient(to right, #111, #44ff44); }
    #b2Slider { background: linear-gradient(to right, #111, #4444ff); }

    /* HSV sliders — read-only */
    #hSlider {
      background: linear-gradient(to right,
        hsl(0,100%,50%), hsl(30,100%,50%), hsl(60,100%,50%), hsl(90,100%,50%),
        hsl(120,100%,50%), hsl(150,100%,50%), hsl(180,100%,50%), hsl(210,100%,50%),
        hsl(240,100%,50%), hsl(270,100%,50%), hsl(300,100%,50%), hsl(330,100%,50%),
        hsl(360,100%,50%));
      pointer-events: none; opacity: 0.7;
    }
    #sSlider { background: linear-gradient(to right, #888, #ff2222); pointer-events: none; opacity: 0.7; }
    #vSlider { background: linear-gradient(to right, #000, #ffffff); pointer-events: none; opacity: 0.7; }

    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%;
      background: white; box-shadow: 0 0 4px rgba(0,0,0,0.5); cursor: pointer; transition: transform 0.1s;
    }

    .hsv-note {
      text-align: center; font-size: 0.65rem; color: #445; letter-spacing: 1px;
      margin-bottom: 14px; text-transform: uppercase;
    }

    .global-status {
      width: 706px; max-width: 95vw; background: #16213e; border-radius: 12px;
      padding: 14px 20px; text-align: center; font-size: 0.7rem; color: #555;
      box-shadow: 0 4px 16px rgba(0,0,0,0.3);
    }
    .global-status.ok  { color: #44cc88; }
    .global-status.err { color: #ff6655; }
  </style>
</head>
<body>

  <div class="row">

    <!-- ── Card 1: LOW ── -->
    <div class="card">
      <h1>LOW COLOR</h1>
      <!-- preview shows BGR: R→B, G→G, B→R -->
      <div class="color-preview" id="preview1"></div>
      <div class="hex-label" id="hexLabel1">#000000</div>
      <div class="bgr-label" id="bgrLabel1">BGR: B=0 G=0 R=0</div>

      <div class="section-title first">RGB</div>
      <div class="slider-row">
        <label>RED <span id="r1Val">0</span></label>
        <input type="range" id="r1Slider" min="0" max="255" value="0">
      </div>
      <div class="slider-row">
        <label>GREEN <span id="g1Val">0</span></label>
        <input type="range" id="g1Slider" min="0" max="255" value="0">
      </div>
      <div class="slider-row">
        <label>BLUE <span id="b1Val">0</span></label>
        <input type="range" id="b1Slider" min="0" max="255" value="0">
      </div>
    </div>

    <!-- ── Card 2: HIGH ── -->
    <div class="card">
      <h1>HIGH COLOR</h1>
      <div class="color-preview" id="preview2"></div>
      <div class="hex-label" id="hexLabel2">#000000</div>
      <div class="bgr-label" id="bgrLabel2">BGR: B=0 G=0 R=0</div>

      <div class="section-title first">RGB</div>
      <div class="slider-row">
        <label>RED <span id="r2Val">0</span></label>
        <input type="range" id="r2Slider" min="0" max="255" value="0">
      </div>
      <div class="slider-row">
        <label>GREEN <span id="g2Val">0</span></label>
        <input type="range" id="g2Slider" min="0" max="255" value="0">
      </div>
      <div class="slider-row">
        <label>BLUE <span id="b2Val">0</span></label>
        <input type="range" id="b2Slider" min="0" max="255" value="0">
      </div>
    </div>

  </div>

  <!-- ── HSV readout — average of both cards ── -->
  <div class="card" style="width:706px; max-width:95vw;">
    <h1>HSV — Average</h1>
    <div class="hsv-note">📊 Midpoint between LOW and HIGH · not editable</div>

    <div class="slider-row">
      <label>HUE <span id="hVal">0°</span></label>
      <input type="range" id="hSlider" min="0" max="360" value="0" tabindex="-1">
    </div>
    <div class="slider-row">
      <label>SATURATION <span id="sVal">0%</span></label>
      <input type="range" id="sSlider" min="0" max="100" value="0" tabindex="-1">
    </div>
    <div class="slider-row">
      <label>VALUE <span id="vVal">0%</span></label>
      <input type="range" id="vSlider" min="0" max="100" value="0" tabindex="-1">
    </div>
  </div>

  <!-- ── Global publish status ── -->
  <div class="global-status" id="globalStatus">Waiting…</div>

  <script>
    const S = {};
    ['r1','g1','b1','r2','g2','b2','h','s','v'].forEach(id => {
      S[id] = document.getElementById(id + 'Slider');
    });

    let publishDebounce;

    function toHex(n) { return Math.round(n).toString(16).padStart(2, '0').toUpperCase(); }

    // ── RGB → HSV ────────────────────────────────────────────────────────────
    function rgbToHsv(r, g, b) {
      r /= 255; g /= 255; b /= 255;
      const max = Math.max(r, g, b), min = Math.min(r, g, b), d = max - min;
      let h = 0;
      const s = max === 0 ? 0 : d / max;
      const v = max;
      if (d !== 0) {
        switch (max) {
          case r: h = ((g - b) / d + (g < b ? 6 : 0)) / 6; break;
          case g: h = ((b - r) / d + 2) / 6; break;
          case b: h = ((r - g) / d + 4) / 6; break;
        }
      }
      return [Math.round(h * 360), Math.round(s * 100), Math.round(v * 100)];
    }

    // ── Circular average of two hues (shortest arc) ──────────────────────────
    function avgHue(h1, h2) {
      let diff = h2 - h1;
      if (diff > 180)  diff -= 360;
      if (diff < -180) diff += 360;
      let avg = (h1 + diff / 2 + 360) % 360;
      return Math.round(avg);
    }

    // ── Update HSV readout as midpoint between both cards ────────────────────
    function updateHsvReadout() {
      const r1 = parseInt(S.r1.value), g1 = parseInt(S.g1.value), b1 = parseInt(S.b1.value);
      const r2 = parseInt(S.r2.value), g2 = parseInt(S.g2.value), b2 = parseInt(S.b2.value);

      const [h1, s1, v1] = rgbToHsv(r1, g1, b1);
      const [h2, s2, v2] = rgbToHsv(r2, g2, b2);

      const h = avgHue(h1, h2);
      const s = Math.round((s1 + s2) / 2);
      const v = Math.round((v1 + v2) / 2);

      S.h.value = h; S.s.value = s; S.v.value = v;
      document.getElementById('hVal').textContent = h + '°';
      document.getElementById('sVal').textContent = s + '%';
      document.getElementById('vVal').textContent = v + '%';
      S.s.style.background = `linear-gradient(to right, hsl(0,0%,${v/2}%), hsl(${h},100%,${v/2}%))`;
      S.v.style.background = `linear-gradient(to right, #000, hsl(${h},${s}%,50%))`;
    }

    // ── Apply RGB to card — preview shown in BGR (swap R↔B for display) ──────
    function applyRgbToCard(c, r, g, b) {
      S[`r${c}`].value = r; S[`g${c}`].value = g; S[`b${c}`].value = b;
      document.getElementById(`r${c}Val`).textContent = r;
      document.getElementById(`g${c}Val`).textContent = g;
      document.getElementById(`b${c}Val`).textContent = b;

      // Published values use R,G,B order — preview swaps to BGR (B→red channel, R→blue channel)
      const hexBgr = '#' + toHex(b) + toHex(g) + toHex(r);
      document.getElementById(`preview${c}`).style.backgroundColor = hexBgr;

      // Hex label shows the true RGB value for reference
      const hexRgb = '#' + toHex(r) + toHex(g) + toHex(b);
      document.getElementById(`hexLabel${c}`).textContent = `RGB ${hexRgb}`;
      document.getElementById(`bgrLabel${c}`).textContent = `BGR: B=${b} G=${g} R=${r}`;
    }

    // ── RGB slider moved ──────────────────────────────────────────────────────
    function onRgbSlide(c) {
      const r = parseInt(S[`r${c}`].value);
      const g = parseInt(S[`g${c}`].value);
      const b = parseInt(S[`b${c}`].value);
      applyRgbToCard(c, r, g, b);
      updateHsvReadout();   // average of both cards
      schedulePublish();
    }

    function schedulePublish() {
      clearTimeout(publishDebounce);
      publishDebounce = setTimeout(publishAll, 60);
    }

    async function publishAll() {
      const payload = {
        low_r:  parseInt(S.r1.value), low_g:  parseInt(S.g1.value), low_b:  parseInt(S.b1.value),
        high_r: parseInt(S.r2.value), high_g: parseInt(S.g2.value), high_b: parseInt(S.b2.value),
        hue:    parseInt(S.h.value),  saturation: parseInt(S.s.value), value: parseInt(S.v.value),
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
          `LOW  R=${d.low_r} G=${d.low_g} B=${d.low_b}   |   ` +
          `HIGH  R=${d.high_r} G=${d.high_g} B=${d.high_b}   |   ` +
          `HSV  H=${d.hue} S=${d.saturation} V=${d.value}`;
        statusEl.className = 'global-status ok';
      } catch(e) {
        statusEl.textContent = 'Error publishing to ROS2';
        statusEl.className = 'global-status err';
      }
    }

    // ── Wire up ───────────────────────────────────────────────────────────────
    ['r1','g1','b1'].forEach(k => S[k].addEventListener('input', () => onRgbSlide(1)));
    ['r2','g2','b2'].forEach(k => S[k].addEventListener('input', () => onRgbSlide(2)));

    // Init
    onRgbSlide(1);
    onRgbSlide(2);
  </script>
</body>
</html>
"""

_node = None

class GUITunner(Node):
    def __init__(self):
        super().__init__('rgb_slider_gui')
        self.pub = self.create_publisher(ColorTunning, '/color_tunning', 10)
        self.get_logger().info('GUITunner node started — publishing on /color_tunning')

    def publish_color(self, low_r, low_g, low_b, high_r, high_g, high_b, hue, saturation, value):
        msg = ColorTunning()
        msg.low_red        = int(low_r)
        msg.low_green      = int(low_g)
        msg.low_blue       = int(low_b)
        msg.high_red       = int(high_r)
        msg.high_green     = int(high_g)
        msg.high_blue      = int(high_b)
        msg.hue            = int(hue)
        msg.saturation     = int(saturation)
        msg.value          = int(value)
        self.pub.publish(msg)
        self.get_logger().info(
            f'LOW r={low_r} g={low_g} b={low_b} | '
            f'HIGH r={high_r} g={high_g} b={high_b} | '
            f'HSV h={hue} s={saturation} v={value}'
        )

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/update', methods=['POST'])
def update():
    d = request.get_json()
    low_r,  low_g,  low_b  = d.get('low_r',  0), d.get('low_g',  0), d.get('low_b',  0)
    high_r, high_g, high_b = d.get('high_r', 0), d.get('high_g', 0), d.get('high_b', 0)
    hue, saturation, value  = d.get('hue', 0), d.get('saturation', 0), d.get('value', 0)
    if _node is not None:
        _node.publish_color(low_r, low_g, low_b, high_r, high_g, high_b, hue, saturation, value)
    return jsonify({
        'low_r': low_r, 'low_g': low_g, 'low_b': low_b,
        'high_r': high_r, 'high_g': high_g, 'high_b': high_b,
        'hue': hue, 'saturation': saturation, 'value': value,
    })

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = GUITunner()
    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()
    print('\n  ROS2 Color Tuner  ->  http://localhost:5000/\n')
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()