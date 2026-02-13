#!/usr/bin/env python3
"""Mock /expressive_say action server for headless integration tests.

This server implements `audio_tts_msgs/action/Communication` and returns a
successful result after a short, configurable delay. It allows the integration
suite to validate that QUESTION_PRESENT advances only after TTS completion.
"""

from __future__ import annotations

import json
import os
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from audio_tts_msgs.action import Communication


class MockExpressiveSayServer(Node):
    def __init__(self) -> None:
        super().__init__("mock_expressive_say_server")

        self._delay_sec = float(os.environ.get("MOCK_EXPRESSIVE_SAY_DELAY_SEC", "0.15"))
        self._log_path = os.environ.get("MOCK_EXPRESSIVE_SAY_LOG_PATH", "").strip()
        if self._log_path:
            try:
                with open(self._log_path, "w", encoding="utf-8"):
                    pass
            except Exception as exc:
                self.get_logger().warning(f"Failed to initialize expressive_say log file: {exc}")
                self._log_path = ""

        self._server = ActionServer(
            self,
            Communication,
            "/expressive_say",
            execute_callback=self._execute,
        )
        self.get_logger().info(
            f"Mock /expressive_say ready (delay={self._delay_sec}s)"
        )

    def _execute(self, goal_handle: Any) -> Communication.Result:
        req = goal_handle.request
        text = getattr(req, "text", "")
        language = getattr(req, "language", "")
        self.get_logger().info(f"/expressive_say goal: {str(text)[:120]}")
        if self._log_path:
            record = {
                "ts": time.monotonic(),
                "text": str(text),
                "language": str(language),
            }
            try:
                with open(self._log_path, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            except Exception as exc:
                self.get_logger().warning(f"Failed writing expressive_say log: {exc}")

        feedback = Communication.Feedback()
        feedback.status = "speaking"
        goal_handle.publish_feedback(feedback)

        time.sleep(self._delay_sec)

        goal_handle.succeed()
        result = Communication.Result()
        result.success = True
        result.message = "ok"
        return result


def main() -> None:
    rclpy.init()
    node = MockExpressiveSayServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
