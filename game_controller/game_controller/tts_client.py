"""Expressive TTS client wrapper.

Uses `audio_tts_msgs/action/Communication` on `/expressive_say` when available.
This module is optional: when `audio_tts_msgs` is not present, callers can
fallback to time-based auto-advance.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

try:  # pragma: no cover - depends on ROS environment
    from rclpy.action import ActionClient
except Exception:  # pragma: no cover - depends on ROS environment
    ActionClient = None  # type: ignore[assignment]

try:  # pragma: no cover - depends on ROS environment
    from audio_tts_msgs.action import Communication
except Exception:  # pragma: no cover - depends on ROS environment
    Communication = None  # type: ignore[assignment]


DoneCallback = Callable[[bool], None]


class ExpressiveTtsClient:
    """Thin wrapper over a ROS2 ActionClient for /expressive_say."""

    def __init__(
        self,
        node: Any,
        action_name: str = "/expressive_say",
        server_wait_timeout_sec: float = 0.1,
        callback_group: Optional[Any] = None,
        logger: Optional[Any] = None,
    ) -> None:
        self._node = node
        self._action_name = str(action_name or "/expressive_say")
        self._server_wait_timeout_sec = float(server_wait_timeout_sec or 0.1)
        self._logger = logger
        self._client = None

        if ActionClient is None or Communication is None:
            return

        self._client = ActionClient(
            node, Communication, self._action_name, callback_group=callback_group
        )

    def available(self) -> bool:
        if self._client is None:
            return False
        try:
            return bool(self._client.wait_for_server(timeout_sec=0.0))
        except Exception:
            return False

    def speak(self, text: str, language: str, done: DoneCallback) -> bool:
        """Send a Communication goal and call `done(success)` on result."""
        if self._client is None or Communication is None:
            return False

        try:
            if not self._client.wait_for_server(timeout_sec=self._server_wait_timeout_sec):
                return False
        except Exception:
            return False

        goal = Communication.Goal()
        goal.text = str(text or "")
        goal.language = str(language or "")

        try:
            goal_future = self._client.send_goal_async(goal)
        except Exception:
            return False

        def _on_goal(fut: Any) -> None:  # pragma: no cover - ROS callback
            try:
                goal_handle = fut.result()
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"TTS goal failed: {exc}")
                done(False)
                return

            if not getattr(goal_handle, "accepted", False):
                if self._logger:
                    self._logger.warning("TTS goal rejected")
                done(False)
                return

            try:
                result_future = goal_handle.get_result_async()
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"TTS get_result_async failed: {exc}")
                done(False)
                return

            def _on_result(res_fut: Any) -> None:  # pragma: no cover - ROS callback
                try:
                    response = res_fut.result()
                    result = getattr(response, "result", None)
                    if result is not None and hasattr(result, "success"):
                        done(bool(result.success))
                    else:
                        done(True)
                except Exception as exc:
                    if self._logger:
                        self._logger.error(f"TTS result failed: {exc}")
                    done(False)

            result_future.add_done_callback(_on_result)

        goal_future.add_done_callback(_on_goal)
        return True

