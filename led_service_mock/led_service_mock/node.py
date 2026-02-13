import os
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from emorobcare_led_service.srv import ControlLEDs, GetLEDState, PlayEffect, SetLEDs
from flask import Flask, Response, jsonify, request


INDEX_HTML = """
<!doctype html>
<html>
  <head>
    <meta charset="utf-8"/>
    <title>LED Service Mock</title>
    <style>
      body { font-family: sans-serif; margin: 24px; background: #111827; color: #e5e7eb; }
      .row { display: flex; gap: 12px; align-items: center; margin-bottom: 14px; flex-wrap: wrap; }
      button { padding: 8px 12px; border: 0; border-radius: 8px; cursor: pointer; }
      input, select { padding: 8px; border-radius: 8px; border: 1px solid #374151; background: #1f2937; color: #e5e7eb; }
      #leds { display: flex; gap: 8px; margin-top: 12px; }
      .led { width: 44px; height: 44px; border-radius: 999px; border: 2px solid #374151; }
      #status { margin-top: 10px; color: #93c5fd; min-height: 20px; }
    </style>
  </head>
  <body>
    <h2>LED Service Mock</h2>
    <div class="row">
      <label>Color:</label>
      <input id="color" type="color" value="#00ffff"/>
      <label>Fade:</label>
      <input id="fade" type="number" value="0.2" min="0" step="0.1"/>
      <button onclick="setColor()">Set All LEDs</button>
      <button onclick="turnOff()">Turn Off</button>
      <button onclick="refreshState()">Refresh</button>
    </div>
    <div class="row">
      <label>Effect:</label>
      <select id="effect">
        <option value="rainbow">rainbow</option>
        <option value="heartbeat">heartbeat</option>
        <option value="cascade">cascade</option>
        <option value="blink">blink</option>
        <option value="knight_rider">knight_rider</option>
        <option value="color_cycle">color_cycle</option>
      </select>
      <label>Duration:</label>
      <input id="duration" type="number" value="2.0" min="0.1" step="0.1"/>
      <button onclick="playEffect()">Play Effect</button>
    </div>
    <div id="status"></div>
    <div id="leds"></div>
    <script>
      function setStatus(text) {
        document.getElementById("status").textContent = text || "";
      }
      async function post(path, payload) {
        const res = await fetch(path, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload || {})
        });
        return await res.json();
      }
      async function setColor() {
        const color = document.getElementById("color").value;
        const fade = Number(document.getElementById("fade").value || 0);
        const data = await post("/api/set_color", { color, fade_time: fade });
        setStatus(data.message || (data.success ? "OK" : "Error"));
        await refreshState();
      }
      async function playEffect() {
        const effect = document.getElementById("effect").value;
        const duration = Number(document.getElementById("duration").value || 2);
        const data = await post("/api/effect", { effect_name: effect, duration });
        setStatus(data.message || (data.success ? "OK" : "Error"));
        await refreshState();
      }
      async function turnOff() {
        const fade = Number(document.getElementById("fade").value || 0);
        const data = await post("/api/off", { fade_time: fade });
        setStatus(data.message || (data.success ? "OK" : "Error"));
        await refreshState();
      }
      async function refreshState() {
        const res = await fetch("/api/state");
        const data = await res.json();
        const root = document.getElementById("leds");
        root.innerHTML = "";
        (data.colors || []).forEach((led) => {
          const dot = document.createElement("div");
          dot.className = "led";
          dot.style.background = led.hex;
          root.appendChild(dot);
        });
        if (data.message) setStatus(data.message);
      }
      refreshState();
      setInterval(refreshState, 1500);
    </script>
  </body>
</html>
"""


def _hex_to_rgb(value: str) -> Tuple[int, int, int]:
    raw = str(value or "").strip().lstrip("#")
    if len(raw) != 6:
        raise ValueError("color must be #RRGGBB")
    return int(raw[0:2], 16), int(raw[2:4], 16), int(raw[4:6], 16)


def _to_hex(red: int, green: int, blue: int) -> str:
    return f"#{int(red):02x}{int(green):02x}{int(blue):02x}"


class LEDServiceBridge(Node):
    def __init__(self) -> None:
        super().__init__("led_service_mock_ui")
        self._set_leds = self.create_client(SetLEDs, "/set_leds")
        self._play_effect = self.create_client(PlayEffect, "/play_effect")
        self._get_state = self.create_client(GetLEDState, "/get_led_state")
        self._control = self.create_client(ControlLEDs, "/control_leds")
        self._call_lock = threading.Lock()

    def _call(self, client, request, timeout_sec: float = 3.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, "Service unavailable", None
        with self._call_lock:
            future = client.call_async(request)
            deadline = time.time() + timeout_sec
            while not future.done() and time.time() < deadline:
                time.sleep(0.01)
            if not future.done():
                return False, "Service timeout", None
            exc = future.exception()
            if exc:
                return False, str(exc), None
            return True, "", future.result()

    def set_all_color(self, red: int, green: int, blue: int, fade_time: float):
        req = SetLEDs.Request()
        req.led_id = -1
        req.r_values = [int(red)]
        req.g_values = [int(green)]
        req.b_values = [int(blue)]
        req.fade_time = float(max(0.0, fade_time))
        req.wait_response = False
        return self._call(self._set_leds, req)

    def play_effect(self, effect_name: str, duration: float):
        req = PlayEffect.Request()
        req.effect_name = str(effect_name or "rainbow")
        req.duration = float(max(0.1, duration))
        req.speed = 0.1
        req.cycles = 2
        req.fade_time = 0.0
        req.direction = "forward"
        req.density = 0.3
        req.progress = 0.5
        return self._call(self._play_effect, req, timeout_sec=max(5.0, req.duration + 1.0))

    def turn_off(self, fade_time: float):
        req = ControlLEDs.Request()
        req.command = "turn_off_all"
        req.fade_time = float(max(0.0, fade_time))
        return self._call(self._control, req)

    def get_state(self):
        req = GetLEDState.Request()
        req.get_colors = True
        return self._call(self._get_state, req)


def create_app(bridge: LEDServiceBridge) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return Response(INDEX_HTML, mimetype="text/html")

    @app.get("/healthz")
    def healthz():
        return jsonify({"ok": True})

    @app.get("/api/state")
    def api_state():
        ok, message, result = bridge.get_state()
        if not ok or result is None:
            return jsonify({"success": False, "message": message, "colors": []}), 503
        colors = []
        for red, green, blue in zip(result.current_r, result.current_g, result.current_b):
            colors.append(
                {
                    "r": int(red),
                    "g": int(green),
                    "b": int(blue),
                    "hex": _to_hex(red, green, blue),
                }
            )
        return jsonify(
            {
                "success": bool(result.success),
                "message": str(result.message),
                "num_leds": int(result.num_leds),
                "is_fading": bool(result.is_fading),
                "colors": colors,
            }
        )

    @app.post("/api/set_color")
    def api_set_color():
        payload = request.get_json(silent=True) or {}
        try:
            red, green, blue = _hex_to_rgb(payload.get("color", "#00ffff"))
            fade_time = float(payload.get("fade_time", 0.2))
        except Exception as exc:
            return jsonify({"success": False, "message": str(exc)}), 400
        ok, message, result = bridge.set_all_color(red, green, blue, fade_time)
        if not ok or result is None:
            return jsonify({"success": False, "message": message}), 503
        return jsonify({"success": bool(result.success), "message": str(result.message)})

    @app.post("/api/effect")
    def api_effect():
        payload = request.get_json(silent=True) or {}
        effect_name = str(payload.get("effect_name", "rainbow"))
        duration = float(payload.get("duration", 2.0))
        ok, message, result = bridge.play_effect(effect_name, duration)
        if not ok or result is None:
            return jsonify({"success": False, "message": message}), 503
        return jsonify({"success": bool(result.success), "message": str(result.message)})

    @app.post("/api/off")
    def api_off():
        payload = request.get_json(silent=True) or {}
        fade_time = float(payload.get("fade_time", 0.0))
        ok, message, result = bridge.turn_off(fade_time)
        if not ok or result is None:
            return jsonify({"success": False, "message": message}), 503
        return jsonify({"success": bool(result.success), "message": str(result.message)})

    return app


def main(args=None) -> None:
    host = os.environ.get("LED_MOCK_UI_HOST", "0.0.0.0")
    port = int(os.environ.get("LED_MOCK_UI_PORT", "8095"))

    rclpy.init(args=args)
    bridge = LEDServiceBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(bridge)
    try:
        app.run(host=host, port=port, debug=False, use_reloader=False)
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
