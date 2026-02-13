#!/usr/bin/env python3
"""Bridge `/ui/input` String payloads to `/intents` RAW_USER_INPUT messages."""

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hri_actions_msgs.msg import Intent


_ACTION_TO_LABEL = {
    "pause": "PAUSE",
    "resume": "RESUME",
    "restart": "RESTART",
    "reset": "RESET",
    "exit": "EXIT",
    "stop": "STOP",
    "back": "BACK",
    "skip_phase": "SKIP_PHASE",
    "skip": "SKIP",
}


class UIIntentBridge(Node):
    def __init__(self) -> None:
        super().__init__("communication_hub_ui_intent_bridge")
        self._intent_pub = self.create_publisher(Intent, "/intents", 10)
        self._ui_sub = self.create_subscription(
            String,
            "/ui/input",
            self._on_ui_input,
            10,
        )
        self.get_logger().info("[BRIDGE] UI→Intent bridge ready (/ui/input -> /intents)")

    def _extract_payload(self, raw: str) -> str:
        raw = str(raw or "").strip()
        if not raw:
            return ""
        try:
            parsed = json.loads(raw)
        except Exception:
            return raw
        if not isinstance(parsed, dict):
            return raw

        payload: Dict[str, Any] = dict(parsed)
        action = str(payload.get("action") or "").strip().lower()
        if action in _ACTION_TO_LABEL:
            return json.dumps({"label": _ACTION_TO_LABEL[action]}, ensure_ascii=False)

        label = payload.get("label")
        if label is not None and str(label).strip():
            return json.dumps({"label": str(label).strip()}, ensure_ascii=False)

        return json.dumps(payload, ensure_ascii=False)

    def _on_ui_input(self, msg: String) -> None:
        payload_text = self._extract_payload(msg.data)
        if not payload_text:
            return

        intent_msg = Intent()
        if hasattr(intent_msg, "intent"):
            intent_msg.intent = Intent.RAW_USER_INPUT
        intent_msg.data = json.dumps({"input": payload_text}, ensure_ascii=False)
        if hasattr(intent_msg, "source"):
            intent_msg.source = Intent.REMOTE_SUPERVISOR
        intent_msg.modality = Intent.MODALITY_TOUCHSCREEN
        if hasattr(intent_msg, "priority"):
            intent_msg.priority = 128
        if hasattr(intent_msg, "confidence"):
            intent_msg.confidence = 1.0
        self._intent_pub.publish(intent_msg)
        self.get_logger().info(
            f"[BRIDGE] Published /intents RAW_USER_INPUT: {payload_text}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UIIntentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
