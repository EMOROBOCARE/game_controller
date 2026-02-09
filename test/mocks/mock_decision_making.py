"""Mock Decision Making FSM for isolated testing.

This mock simulates the decision_making node by:
1. Subscribing to /decision/events
2. Publishing state updates to /decision/state
3. Simulating basic FSM state transitions
"""

import json
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockDecisionMaking(Node):
    """Mock FSM that simulates decision_making behavior."""

    def __init__(self):
        super().__init__("mock_decision_making")

        # State tracking
        self._transaction_counter = 0
        self._session_id: Optional[int] = None
        self._current_game_state = "IDLE"
        self._current_system_state = "IDLE"

        # Publishers
        self._state_pub = self.create_publisher(
            String,
            "/decision/state",
            10,
        )

        # Subscribers
        self._events_sub = self.create_subscription(
            String,
            "/decision/events",
            self._on_event,
            10,
        )

        # Publish initial IDLE state after short delay
        self.create_timer(0.5, self._publish_initial_state)

        self.get_logger().info("Mock Decision Making initialized")

    def _publish_initial_state(self) -> None:
        """Publish initial IDLE state."""
        if not hasattr(self, "_initial_published"):
            self._publish_state("IDLE", "IDLE")
            self._initial_published = True

    def _on_event(self, msg: String) -> None:
        """Handle events from game_controller."""
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Failed to parse event: {msg.data}")
            return

        event_type = event.get("type")
        payload = event.get("payload", {})

        self.get_logger().info(f"Received event: {event_type}")

        if event_type == "GAME_INIT":
            self._handle_game_init(payload)
        elif event_type == "USER_INTENT":
            self._handle_user_intent(payload)
        elif event_type == "ON_COMPLETE":
            self._handle_on_complete(payload)
        elif event_type == "GAME_CONTROL":
            self._handle_game_control(payload)

    def _handle_game_init(self, payload: Dict[str, Any]) -> None:
        """Handle GAME_INIT event."""
        self._session_id = payload.get("sessionId", 1)
        self._transaction_counter = 1

        # Simulate GAME_START state
        self._publish_state(
            "GAME_START",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "gameInfo": payload.get("gameInfo", {}),
            }
        )

        # Auto-transition to PHASE_INTRO after short delay
        def go_to_phase_intro():
            threading.Event().wait(0.3)
            if self._current_game_state == "GAME_START":
                self._simulate_phase_intro(payload)

        threading.Thread(target=go_to_phase_intro, daemon=True).start()

    def _simulate_phase_intro(self, game_payload: Dict[str, Any]) -> None:
        """Simulate PHASE_INTRO state."""
        self._transaction_counter += 1

        rounds = game_payload.get("rounds", [])
        if rounds:
            first_round = rounds[0]
            phase = first_round.get("phase", "P1")

            self._publish_state(
                "PHASE_INTRO",
                "ACTIVE",
                payload={
                    "sessionId": self._session_id,
                    "phase": phase,
                    "roundId": 0,
                }
            )

    def _handle_on_complete(self, payload: Dict[str, Any]) -> None:
        """Handle ON_COMPLETE event."""
        transaction_id = payload.get("transactionId")

        # Verify transaction ID matches
        if transaction_id != self._transaction_counter:
            self.get_logger().warn(
                f"Transaction ID mismatch: expected {self._transaction_counter}, "
                f"got {transaction_id}"
            )
            return

        # Simulate state progression based on current state
        if self._current_game_state == "PHASE_INTRO":
            self._simulate_wait_input()
        elif self._current_game_state == "CORRECT":
            self._simulate_phase_complete()
        elif self._current_game_state in ("FAIL_L1", "FAIL_L2"):
            self._simulate_wait_input()

    def _simulate_wait_input(self) -> None:
        """Simulate WAIT_INPUT state."""
        self._transaction_counter += 1

        self._publish_state(
            "WAIT_INPUT",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "question": {
                    "questionId": 1,
                    "questionType": "color",
                    "text": "¿De qué color es esto?",
                    "options": [
                        {"id": "1", "label": "rojo", "correct": True},
                        {"id": "2", "label": "azul", "correct": False},
                    ],
                },
                "roundId": 1,
                "phase": "P1",
            }
        )

    def _simulate_phase_complete(self) -> None:
        """Simulate PHASE_COMPLETE state."""
        self._transaction_counter += 1

        self._publish_state(
            "PHASE_COMPLETE",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "phase": "P1",
            }
        )

    def _handle_user_intent(self, payload: Dict[str, Any]) -> None:
        """Handle USER_INTENT event."""
        transaction_id = payload.get("transactionId")

        if transaction_id != self._transaction_counter:
            self.get_logger().warn(
                f"Transaction ID mismatch in USER_INTENT: expected "
                f"{self._transaction_counter}, got {transaction_id}"
            )
            return

        # Check if answer is correct (simplified)
        answer = payload.get("answer", {})
        value = answer.get("value") or answer.get("label")

        # Simulate correct/incorrect
        if value and "rojo" in str(value).lower():
            self._simulate_correct()
        else:
            self._simulate_fail_l1()

    def _simulate_correct(self) -> None:
        """Simulate CORRECT state."""
        self._transaction_counter += 1

        self._publish_state(
            "CORRECT",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
            }
        )

    def _simulate_fail_l1(self) -> None:
        """Simulate FAIL_L1 state."""
        self._transaction_counter += 1

        self._publish_state(
            "FAIL_L1",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
            }
        )

    def _handle_game_control(self, payload: Dict[str, Any]) -> None:
        """Handle GAME_CONTROL event (PAUSE, RESUME, EXIT)."""
        action = payload.get("action", "").upper()

        if action == "PAUSE":
            self._current_system_state = "PAUSED"
            self._transaction_counter += 1
            self._publish_state(self._current_game_state, "PAUSED")
        elif action == "RESUME":
            self._current_system_state = "ACTIVE"
            self._transaction_counter += 1
            self._publish_state(self._current_game_state, "ACTIVE")
        elif action == "EXIT":
            self._current_game_state = "IDLE"
            self._current_system_state = "IDLE"
            self._transaction_counter += 1
            self._publish_state("IDLE", "IDLE")

    def _publish_state(
        self,
        game_state: str,
        system_state: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Publish a state update."""
        self._current_game_state = game_state
        self._current_system_state = system_state

        state_data = {
            "state": system_state,
            "gameState": game_state,
            "transactionId": self._transaction_counter,
            "sessionId": self._session_id,
            "payload": payload or {},
        }

        msg = String()
        msg.data = json.dumps(state_data)
        self._state_pub.publish(msg)

        self.get_logger().info(
            f"Published state: {game_state} (tx={self._transaction_counter})"
        )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MockDecisionMaking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
