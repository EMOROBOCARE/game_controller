#!/usr/bin/env python3
"""ROS2 TTS mock for local development.

Features:
- Provides /say (audio_tts_msgs/action/TTS)
- Web UI that speaks received text in the browser via Web Speech API
  so audio plays through laptop speakers.
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any, Deque, Dict, List

import rclpy
from flask import Flask, Response, jsonify
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from audio_tts_msgs.action import TTS


INDEX_HTML = """
<!doctype html>
<html>
  <head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1"/>
    <title>TTS Mock</title>
    <style>
      :root {
        --bg: #0d1b2a;
        --panel: #1b263b;
        --text: #e0e1dd;
        --accent: #ffb703;
        --muted: #93a8c3;
      }
      body {
        margin: 0;
        font-family: "Trebuchet MS", "Segoe UI", sans-serif;
        color: var(--text);
        background:
          radial-gradient(900px 500px at 10% -5%, #26465366, transparent 60%),
          radial-gradient(700px 450px at 95% 10%, #e76f5133, transparent 60%),
          var(--bg);
      }
      .wrap { max-width: 980px; margin: 24px auto; padding: 0 16px; }
      .panel {
        background: linear-gradient(160deg, #1b263b 0%, #223252 100%);
        border: 1px solid #38507e;
        border-radius: 14px;
        padding: 14px;
      }
      button {
        background: var(--accent);
        color: #1c1c1c;
        border: 0;
        border-radius: 10px;
        font-weight: 700;
        padding: 10px 12px;
        cursor: pointer;
      }
      #status { color: var(--muted); margin-top: 8px; min-height: 20px; }
      .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
      .event {
        border: 1px solid #324a75;
        border-radius: 10px;
        padding: 10px;
        margin-top: 10px;
        background: #10203a;
      }
      .meta { color: var(--muted); font-size: 12px; margin-bottom: 6px; }
      .text { font-size: 18px; }
      .tag {
        display: inline-block;
        margin-right: 8px;
        font-size: 12px;
        background: #2a3b5a;
        border-radius: 99px;
        padding: 2px 8px;
      }
    </style>
  </head>
  <body>
    <div class="wrap">
      <h2>TTS Mock Speaker</h2>
      <div class="panel">
        <div class="row">
          <button id="enableBtn">Enable Audio</button>
          <button id="testBtn">Test Voice</button>
          <span id="status">Waiting for /say requests...</span>
        </div>
      </div>
      <div id="events"></div>
    </div>
    <script>
      let audioEnabled = false;
      const eventsRoot = document.getElementById("events");
      const statusEl = document.getElementById("status");

      function setStatus(text) {
        statusEl.textContent = text;
      }

      function chooseLang(lang) {
        const raw = String(lang || "es").toLowerCase();
        if (raw.startsWith("es")) return "es-ES";
        if (raw.startsWith("en")) return "en-US";
        return "es-ES";
      }

      function speak(evt) {
        if (!("speechSynthesis" in window)) {
          setStatus("Speech synthesis API unavailable in this browser.");
          return;
        }
        const u = new SpeechSynthesisUtterance(evt.text || "");
        u.lang = chooseLang(evt.language);
        u.rate = Number(evt.rate || 1.0);
        setStatus("Speaking: " + (evt.text || ""));
        window.speechSynthesis.cancel();
        window.speechSynthesis.speak(u);
      }

      function renderEvent(evt) {
        const node = document.createElement("div");
        node.className = "event";
        node.innerHTML = `
          <div class="meta">
            <span class="tag">#${evt.id}</span>
            <span class="tag">${evt.language || "es"}</span>
            <span class="tag">${evt.emotion || "neutral"}</span>
            <span>${evt.timestamp || ""}</span>
          </div>
          <div class="text">${evt.text || ""}</div>
        `;
        eventsRoot.prepend(node);
      }

      document.getElementById("enableBtn").addEventListener("click", () => {
        audioEnabled = true;
        window.speechSynthesis.cancel();
        const warmup = new SpeechSynthesisUtterance("Audio enabled");
        warmup.volume = 0.01;
        window.speechSynthesis.speak(warmup);
        setStatus("Audio enabled. Waiting for /say requests...");
      });

      document.getElementById("testBtn").addEventListener("click", () => {
        audioEnabled = true;
        speak({ text: "Prueba de voz lista", language: "es", rate: 1.0 });
      });

      fetch("/api/history")
        .then((r) => r.json())
        .then((data) => {
          (data.history || []).forEach((evt) => renderEvent(evt));
        })
        .catch(() => {});

      const stream = new EventSource("/events");
      stream.onmessage = (ev) => {
        const msg = JSON.parse(ev.data);
        renderEvent(msg);
        if (audioEnabled) {
          speak(msg);
        } else {
          setStatus("New TTS request received. Click 'Enable Audio' once.");
        }
      };
      stream.onerror = () => setStatus("Event stream disconnected. Retrying...");
    </script>
  </body>
</html>
"""


class TtsMockNode(Node):
    def __init__(self) -> None:
        super().__init__("tts_mock_node")
        self._history_limit = int(os.environ.get("TTS_MOCK_HISTORY_LIMIT", "120"))
        self._history: Deque[Dict[str, Any]] = deque(maxlen=max(1, self._history_limit))
        self._next_id = 1
        self._lock = threading.Lock()
        self._listeners: List[queue.Queue] = []
        self._action_server = ActionServer(
            self,
            TTS,
            "/say",
            execute_callback=self._execute,
        )
        self.get_logger().info("TTS mock ready on /say")

    def _broadcast(self, payload: Dict[str, Any]) -> None:
        stale: List[queue.Queue] = []
        for q in self._listeners:
            try:
                q.put_nowait(payload)
            except Exception:
                stale.append(q)
        if stale:
            self._listeners = [q for q in self._listeners if q not in stale]

    def _append_event(self, payload: Dict[str, Any]) -> None:
        with self._lock:
            self._history.append(payload)
        self._broadcast(payload)

    def snapshot_history(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._history)

    def register_listener(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=50)
        self._listeners.append(q)
        return q

    def unregister_listener(self, q: queue.Queue) -> None:
        self._listeners = [it for it in self._listeners if it is not q]

    def _execute(self, goal_handle):
        req = goal_handle.request
        text = str(req.text or "").strip()
        language = str(req.language or "es")
        emotion = str(req.emotion or "neutral")
        rate = float(req.rate or 1.0)
        temperature = float(req.temperature or 0.5)

        with self._lock:
            event_id = self._next_id
            self._next_id += 1
        payload = {
            "id": event_id,
            "timestamp": datetime.now(tz=timezone.utc).isoformat(),
            "text": text,
            "language": language,
            "emotion": emotion,
            "rate": rate,
            "temperature": temperature,
        }
        self._append_event(payload)
        self.get_logger().info(
            f"/say [{event_id}] ({language}, {emotion}, rate={rate:.2f}): {text}"
        )

        # Keep compatibility across TTS action variants:
        # - legacy variants expose feedback.status/result.success/message
        # - real /src variant exposes feedback.audio/result.text
        feedback = TTS.Feedback()
        if hasattr(feedback, "status"):
            feedback.status = "speaking"
        goal_handle.publish_feedback(feedback)

        # Simulate speaking duration enough for UI/flow visibility.
        duration = min(5.0, max(0.25, 0.045 * max(1, len(text)) / max(0.4, rate)))
        time.sleep(duration)

        result = TTS.Result()
        if hasattr(result, "success"):
            result.success = True
        if hasattr(result, "message"):
            result.message = f"spoken:{event_id}"
        if hasattr(result, "text"):
            result.text = text
        goal_handle.succeed()
        return result


def create_app(node: TtsMockNode) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return Response(INDEX_HTML, mimetype="text/html")

    @app.get("/healthz")
    def healthz():
        return jsonify({"ok": True})

    @app.get("/api/history")
    def api_history():
        return jsonify({"history": node.snapshot_history()})

    @app.get("/events")
    def events():
        listener = node.register_listener()

        def _stream():
            try:
                while True:
                    try:
                        item = listener.get(timeout=15)
                        yield f"data: {json.dumps(item, ensure_ascii=False)}\n\n"
                    except queue.Empty:
                        yield ": keep-alive\n\n"
            finally:
                node.unregister_listener(listener)

        return Response(_stream(), mimetype="text/event-stream")

    return app


def main(args=None) -> None:
    host = os.environ.get("TTS_MOCK_UI_HOST", "0.0.0.0")
    port = int(os.environ.get("TTS_MOCK_UI_PORT", "8096"))

    rclpy.init(args=args)
    node = TtsMockNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(node)
    try:
        app.run(host=host, port=port, debug=False, use_reloader=False)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
