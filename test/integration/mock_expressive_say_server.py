#!/usr/bin/env python3
"""Mock /expressive_say action server for headless integration tests.

This server implements `audio_tts_msgs/action/Communication` and returns a
successful result after a short, configurable delay. It allows the integration
suite to validate that QUESTION_PRESENT advances only after TTS completion.
"""

from __future__ import annotations

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
        self.get_logger().info(f"/expressive_say goal: {str(text)[:120]}")

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
