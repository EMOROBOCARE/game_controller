"""Integration tests with mock services.

These tests run the game_controller node against mock decision_making and
generic_ui services, allowing isolated testing without external dependencies.
"""

import json
import time
from typing import Any, Dict, List, Optional

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int16
from hri_actions_msgs.msg import Intent


class TestHelper(Node):
    """Helper node for testing - listens to topics and publishes test inputs."""

    def __init__(self):
        super().__init__("test_helper")

        # Storage for received messages
        self.decision_events: List[Dict[str, Any]] = []
        self.decision_states: List[Dict[str, Any]] = []
        self.current_user_msgs: List[int] = []

        # Subscribers
        self._events_sub = self.create_subscription(
            String,
            "/decision/events",
            self._on_event,
            10,
        )

        self._state_sub = self.create_subscription(
            String,
            "/decision/state",
            self._on_state,
            10,
        )

        self._current_user_sub = self.create_subscription(
            Int16,
            "/game/current_user",
            self._on_current_user,
            10,
        )

        # Publishers
        self._game_selector_pub = self.create_publisher(
            String,
            "/game/game_selector",
            10,
        )

        self._user_selector_pub = self.create_publisher(
            String,
            "/game/user_selector",
            10,
        )

        self._intent_pub = self.create_publisher(
            Intent,
            "/intents",
            10,
        )

    def _on_event(self, msg: String) -> None:
        """Store received events."""
        try:
            event = json.loads(msg.data)
            self.decision_events.append(event)
            self.get_logger().info(f"Received event: {event.get('type')}")
        except json.JSONDecodeError:
            pass

    def _on_state(self, msg: String) -> None:
        """Store received states."""
        try:
            state = json.loads(msg.data)
            self.decision_states.append(state)
            self.get_logger().info(f"Received state: {state.get('gameState')}")
        except json.JSONDecodeError:
            pass

    def _on_current_user(self, msg: Int16) -> None:
        """Store current user messages."""
        self.current_user_msgs.append(msg.data)

    def publish_game_selection(self, game_slug: str) -> None:
        """Publish game selection."""
        msg = String()
        msg.data = game_slug
        self._game_selector_pub.publish(msg)
        self.get_logger().info(f"Published game selection: {game_slug}")

    def publish_user_selection(self, user_id: int) -> None:
        """Publish user selection."""
        msg = String()
        msg.data = str(user_id)
        self._user_selector_pub.publish(msg)
        self.get_logger().info(f"Published user selection: {user_id}")

    def publish_intent(
        self,
        label: str,
        modality: str = Intent.MODALITY_TOUCHSCREEN,
    ) -> None:
        """Publish an intent."""
        msg = Intent()
        msg.modality = modality
        msg.data = json.dumps({"label": label})
        self._intent_pub.publish(msg)
        self.get_logger().info(f"Published intent: {label}")

    def wait_for_event_type(
        self,
        event_type: str,
        timeout: float = 5.0,
    ) -> Optional[Dict[str, Any]]:
        """Wait for a specific event type."""
        start = time.time()
        while time.time() - start < timeout:
            for event in self.decision_events:
                if event.get("type") == event_type:
                    return event
            time.sleep(0.1)
        return None

    def wait_for_state(
        self,
        game_state: str,
        timeout: float = 5.0,
    ) -> Optional[Dict[str, Any]]:
        """Wait for a specific game state."""
        start = time.time()
        while time.time() - start < timeout:
            for state in self.decision_states:
                if state.get("gameState") == game_state:
                    return state
            time.sleep(0.1)
        return None

    def wait_for_system_state(
        self,
        system_state: str,
        timeout: float = 5.0,
    ) -> Optional[Dict[str, Any]]:
        """Wait for a specific system state."""
        start = time.time()
        while time.time() - start < timeout:
            for state in self.decision_states:
                if state.get("state") == system_state:
                    return state
            time.sleep(0.1)
        return None

    def clear_messages(self) -> None:
        """Clear all stored messages."""
        self.decision_events.clear()
        self.decision_states.clear()
        self.current_user_msgs.clear()


@pytest.fixture(scope="module")
def ros_context():
    """Initialize ROS context for tests."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def test_helper(ros_context):
    """Create test helper node."""
    helper = TestHelper()
    executor = MultiThreadedExecutor()
    executor.add_node(helper)

    # Spin in background thread
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for node to initialize
    time.sleep(1.0)

    yield helper

    # Ensure each test leaves the shared session stack in IDLE.
    if rclpy.ok():
        try:
            helper.publish_intent("EXIT")
            helper.wait_for_system_state("IDLE", timeout=3.0)
        except Exception:
            pass

    # Cleanup
    helper.clear_messages()
    try:
        executor.shutdown()
    except Exception:
        pass
    try:
        helper.destroy_node()
    except Exception:
        pass


class TestGameControllerIsolated:
    """Test game_controller with mock services."""

    def test_game_initialization(self, test_helper):
        """Test that game initialization publishes GAME_INIT event."""
        # Clear any previous messages
        test_helper.clear_messages()

        # Select user
        test_helper.publish_user_selection(1)
        time.sleep(0.2)

        # Select game
        test_helper.publish_game_selection("colores")
        time.sleep(0.5)

        # Wait for GAME_INIT event
        event = test_helper.wait_for_event_type("GAME_INIT", timeout=3.0)

        assert event is not None, "GAME_INIT event not received"
        assert event["type"] == "GAME_INIT"

        payload = event.get("payload", {})
        assert "sessionId" in payload
        assert "slug" in payload
        assert "rounds" in payload

    def test_current_user_published(self, test_helper):
        """Test that current user is published on selection."""
        test_helper.clear_messages()

        # Select user
        test_helper.publish_user_selection(42)
        time.sleep(0.3)

        # Check current_user was published
        assert len(test_helper.current_user_msgs) > 0
        assert test_helper.current_user_msgs[-1] == 42

    def test_user_intent_forwarding(self, test_helper):
        """Test that user intents are translated and forwarded."""
        test_helper.clear_messages()

        # Start a game first
        test_helper.publish_user_selection(1)
        time.sleep(0.2)
        test_helper.publish_game_selection("colores")
        time.sleep(0.5)

        # Wait for WAIT_INPUT state (simulated by mock)
        state = test_helper.wait_for_state("WAIT_INPUT", timeout=6.0)
        assert state is not None, "WAIT_INPUT state not received"

        # Clear events to isolate USER_INTENT
        test_helper.decision_events.clear()

        # Publish an intent
        test_helper.publish_intent("rojo")
        time.sleep(0.3)

        # Wait for USER_INTENT event
        event = test_helper.wait_for_event_type("USER_INTENT", timeout=2.0)

        assert event is not None, "USER_INTENT event not received"
        assert event["type"] == "USER_INTENT"

        payload = event.get("payload", {})
        assert "transactionId" in payload
        assert "value" in payload

    def test_auto_advance_on_complete(self, test_helper):
        """Test that ON_COMPLETE is sent for auto-advancing states."""
        test_helper.clear_messages()

        # Start a game
        test_helper.publish_user_selection(1)
        test_helper.publish_game_selection("colores")
        # Wait until the game reaches WAIT_INPUT after auto-advances
        state = test_helper.wait_for_state("WAIT_INPUT", timeout=6.0)
        assert state is not None

        on_complete_events = [
            event for event in test_helper.decision_events
            if event.get("type") == "ON_COMPLETE"
        ]
        assert len(on_complete_events) >= 1, "Expected at least one ON_COMPLETE event"

    def test_transaction_id_tracking(self, test_helper):
        """Test that transaction IDs are properly tracked and used."""
        test_helper.clear_messages()

        # Start game
        test_helper.publish_user_selection(1)
        test_helper.publish_game_selection("colores")
        time.sleep(0.5)

        # Get first state
        assert len(test_helper.decision_states) > 0
        first_state = test_helper.decision_states[0]
        first_tx_id = first_state.get("transactionId")

        assert first_tx_id is not None

        # Wait for more states
        time.sleep(1.0)

        # Check that transaction IDs are increasing
        tx_ids = [s.get("transactionId") for s in test_helper.decision_states]
        tx_ids = [tid for tid in tx_ids if tid is not None]

        assert len(tx_ids) > 0
        # Transaction IDs should be sequential
        for i in range(1, len(tx_ids)):
            assert tx_ids[i] >= tx_ids[i - 1]

    def test_game_selection_with_json(self, test_helper):
        """Test game selection with JSON payload."""
        test_helper.clear_messages()

        # Select user first
        test_helper.publish_user_selection(1)
        time.sleep(0.2)

        # Select game with JSON
        game_msg = String()
        game_msg.data = json.dumps({
            "slug": "colores",
            "phases": ["P1", "P2"],
            "difficulty": "intermediate",
        })

        test_helper._game_selector_pub.publish(game_msg)
        time.sleep(0.5)

        # Wait for GAME_INIT
        event = test_helper.wait_for_event_type("GAME_INIT", timeout=3.0)

        assert event is not None
        payload = event.get("payload", {})

        # Check that rounds match the requested phases
        rounds = payload.get("rounds", [])
        phases_in_rounds = [r.get("phase") for r in rounds]

        # Should have rounds for P1 and P2
        assert "P1" in phases_in_rounds
        assert "P2" in phases_in_rounds

    def test_intent_during_non_answerable_state_ignored(self, test_helper):
        """Test that intents are not forwarded during non-answerable states."""
        test_helper.clear_messages()

        # Clear events
        test_helper.decision_events.clear()

        # Try to send an answer while IDLE
        test_helper.publish_intent("rojo")
        time.sleep(0.3)

        # Should NOT have received USER_INTENT
        user_intents = [e for e in test_helper.decision_events if e.get("type") == "USER_INTENT"]

        # During PHASE_INTRO, answers should not be forwarded
        # (Only GAME_CONTROL commands would be forwarded)
        assert len(user_intents) == 0

    def test_speech_intent_modality(self, test_helper):
        """Test that speech intents are properly handled."""
        test_helper.clear_messages()

        # Start game and get to WAIT_INPUT
        test_helper.publish_user_selection(1)
        test_helper.publish_game_selection("colores")
        time.sleep(0.5)

        state = test_helper.wait_for_state("WAIT_INPUT", timeout=3.0)
        assert state is not None

        test_helper.decision_events.clear()

        # Send speech intent
        test_helper.publish_intent("rojo", modality=Intent.MODALITY_SPEECH)
        time.sleep(0.3)

        # Should receive USER_INTENT with modality info
        event = test_helper.wait_for_event_type("USER_INTENT", timeout=2.0)

        assert event is not None
        payload = event.get("payload", {})
        # The modality should be tracked on USER_INTENT payload.
        modality = payload.get("modality", "")
        assert modality == "speech"


class TestGameControllerConfiguration:
    """Test configuration and parameter handling."""

    def test_multiple_game_sessions(self, test_helper):
        """Test that multiple game sessions have different session IDs."""
        test_helper.clear_messages()

        # Session 1
        test_helper.publish_user_selection(1)
        test_helper.publish_game_selection("colores")
        time.sleep(0.5)

        event1 = test_helper.wait_for_event_type("GAME_INIT", timeout=3.0)
        assert event1 is not None
        session1 = event1["payload"]["sessionId"]

        # End session 1 and start session 2.
        test_helper.publish_intent("EXIT")
        idle = test_helper.wait_for_system_state("IDLE", timeout=3.0)
        assert idle is not None

        test_helper.decision_events.clear()
        test_helper.publish_user_selection(1)
        time.sleep(0.2)
        test_helper.publish_game_selection("colores")
        time.sleep(0.5)

        event2 = test_helper.wait_for_event_type("GAME_INIT", timeout=3.0)
        assert event2 is not None
        session2 = event2["payload"]["sessionId"]

        # Session IDs should be different
        assert session2 != session1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
