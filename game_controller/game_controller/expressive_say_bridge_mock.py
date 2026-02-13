#!/usr/bin/env python3
"""Mock communication_hub expressive bridge.

Provides /expressive_say (Communication action) and forwards requests to /say
(TTS action) so game_controller can keep using the production action contract.
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from audio_tts_msgs.action import Communication, TTS


class ExpressiveSayBridgeMock(Node):
    def __init__(self) -> None:
        super().__init__("expressive_say_bridge_mock")
        self._tts_client = ActionClient(self, TTS, "/say")
        self._action_server = ActionServer(
            self,
            Communication,
            "/expressive_say",
            execute_callback=self._execute_callback,
        )
        self.declare_parameter("default_emotion", "neutral")
        self.declare_parameter("default_rate", 1.0)
        self.declare_parameter("default_temperature", 0.5)
        self.declare_parameter("say_wait_timeout_sec", 2.0)
        self.get_logger().info(
            "expressive_say_bridge_mock ready (/expressive_say -> /say)"
        )

    def _send_tts_goal(
        self,
        text: str,
        language: str,
    ) -> Optional[TTS.Result]:
        wait_timeout = float(self.get_parameter("say_wait_timeout_sec").value or 2.0)
        if not self._tts_client.wait_for_server(timeout_sec=max(0.0, wait_timeout)):
            self.get_logger().warning("/say action server not available")
            return None

        goal = TTS.Goal()
        goal.text = str(text or "")
        goal.language = str(language or "es")
        goal.emotion = str(self.get_parameter("default_emotion").value or "neutral")
        goal.rate = float(self.get_parameter("default_rate").value or 1.0)
        goal.temperature = float(
            self.get_parameter("default_temperature").value or 0.5
        )

        goal_future = self._tts_client.send_goal_async(goal)
        deadline = time.time() + max(0.1, wait_timeout)
        while not goal_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not goal_future.done():
            self.get_logger().warning("Timed out waiting for /say goal acceptance")
            return None

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("/say goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        deadline = time.time() + 20.0
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().warning("Timed out waiting for /say result")
            return None

        result_response = result_future.result()
        return getattr(result_response, "result", None)

    def _execute_callback(self, goal_handle):
        text = str(goal_handle.request.text or "")
        language = str(goal_handle.request.language or "es")
        self.get_logger().info(
            f"/expressive_say received: text='{text[:120]}', language={language}"
        )
        tts_result = self._send_tts_goal(text, language)

        result = Communication.Result()
        success = False
        message = "Failed forwarding request to /say"
        if tts_result is None:
            success = False
        else:
            success = bool(getattr(tts_result, "success", True))
            message = str(getattr(tts_result, "message", "ok"))
            if hasattr(result, "text") and hasattr(tts_result, "text"):
                result.text = str(getattr(tts_result, "text", ""))

        if hasattr(result, "success"):
            result.success = success
        if hasattr(result, "message"):
            result.message = message

        goal_handle.succeed() if success else goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExpressiveSayBridgeMock()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
