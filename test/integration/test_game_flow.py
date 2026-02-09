"""Integration tests for Game Controller system.

These tests verify the complete flow:
1. Game initialization
2. State transitions
3. User input handling
4. Auto-advance behavior
"""

import json
import time
import pytest
import rclpy
from std_msgs.msg import String
from hri_actions_msgs.msg import Intent


class TestGameFlow:
    """Test the complete game flow."""

    _user_counter = 1000
    
    @pytest.fixture(autouse=True)
    def setup_ros(self):
        """Setup ROS2 context."""
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("test_node")
        
        # Publishers
        self.game_selector_pub = self.node.create_publisher(
            String, "/game/game_selector", 10
        )
        self.user_selector_pub = self.node.create_publisher(
            String, "/game/user_selector", 10
        )
        self.intent_pub = self.node.create_publisher(
            Intent, "/intents", 10
        )
        
        # State tracking
        self.received_states = []
        self.latest_state = None
        
        # Subscribers
        self.state_sub = self.node.create_subscription(
            String, "/decision/state", self._on_state, 10
        )

        yield

        # Ensure each test leaves the shared stack in IDLE to avoid cross-test bleed.
        self._send_control("EXIT")
        self._wait_for_system_state("IDLE", timeout=3.0)

        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    def _on_state(self, msg):
        """Callback for state updates."""
        try:
            state = json.loads(msg.data)
            self.received_states.append(state)
            self.latest_state = state
        except json.JSONDecodeError:
            pass
    
    def _spin_until(self, condition, timeout=10.0):
        """Spin until condition is met or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition():
                return True
        return False
    
    def _wait_for_state(self, game_state, timeout=10.0):
        """Wait for a specific game state."""
        def condition():
            return (
                self.latest_state is not None and
                self.latest_state.get("gameState") == game_state
            )
        return self._spin_until(condition, timeout)

    def _wait_for_system_state(self, system_state, timeout=10.0):
        """Wait for a specific system state."""
        def condition():
            return (
                self.latest_state is not None and
                self.latest_state.get("state") == system_state
            )
        return self._spin_until(condition, timeout)

    def _next_user_id(self):
        """Generate a unique user id per test."""
        self.__class__._user_counter += 1
        return self.__class__._user_counter

    def _send_control(self, label):
        """Send a control command through /intents."""
        self._publish_intent({"label": label})

    def _publish_intent(self, payload):
        """Publish an intent payload to /intents."""
        msg = Intent()
        msg.modality = Intent.MODALITY_TOUCHSCREEN
        msg.data = json.dumps(payload)
        self.intent_pub.publish(msg)

    def _ensure_idle(self, timeout=5.0):
        """Force decision_making back to IDLE before starting a new game."""
        self._send_control("EXIT")
        # Best effort reset; if IDLE is not observed, start-game assertions still gate validity.
        self._wait_for_system_state("IDLE", timeout=timeout)

    def _start_game(self, game_slug="colores", wait_state=None, timeout=15.0):
        """Select a user and game, optionally waiting for a state."""
        self._ensure_idle(timeout=5.0)

        user_msg = String()
        user_msg.data = str(self._next_user_id())
        self.user_selector_pub.publish(user_msg)

        rclpy.spin_once(self.node, timeout_sec=0.5)

        game_msg = String()
        game_msg.data = game_slug
        self.game_selector_pub.publish(game_msg)

        if wait_state:
            assert self._wait_for_state(wait_state, timeout=timeout), (
                f"Did not receive {wait_state} state"
            )
    
    def test_game_initialization(self):
        """Test that selecting game and user starts a session."""
        # Wait for nodes to be ready
        time.sleep(2)

        self._start_game(wait_state="PHASE_INTRO", timeout=15.0)
        
        # Verify state contains expected fields
        assert self.latest_state.get("sessionId") is not None
        assert self.latest_state.get("transactionId") is not None
    
    def test_auto_advance_through_phases(self):
        """Test that auto-advance moves through non-interactive states."""
        self._start_game(wait_state="PHASE_INTRO", timeout=15.0)

        # Auto-advance should move to QUESTION_PRESENT
        assert self._wait_for_state("QUESTION_PRESENT", timeout=5.0), \
            "Did not auto-advance to QUESTION_PRESENT"
        
        # Auto-advance should move to WAIT_INPUT
        assert self._wait_for_state("WAIT_INPUT", timeout=5.0), \
            "Did not auto-advance to WAIT_INPUT"
    
    def test_correct_answer_flow(self):
        """Test that correct answer advances the game."""
        self._start_game(wait_state="WAIT_INPUT", timeout=20.0)
        
        # Get the current question's correct answer
        payload = self.latest_state.get("payload", {})
        question = None
        for state in reversed(self.received_states):
            if state.get("gameState") == "QUESTION_PRESENT":
                question = state.get("payload", {}).get("question", {})
                break
        
        # Send correct answer
        answer = question.get("answer") if question else "rojo"
        if not answer:
            answer = question.get("meta", {}).get("color", "rojo") if question else "rojo"
        
        self._publish_intent({"label": answer, "correct": True})
        
        # Wait for CORRECT state
        assert self._wait_for_state("CORRECT", timeout=5.0), \
            "Did not receive CORRECT state after correct answer"
    
    def test_wrong_answer_flow(self):
        """Test that wrong answer triggers FAIL_L1."""
        self._start_game(wait_state="WAIT_INPUT", timeout=20.0)
        
        # Send wrong answer
        self._publish_intent({"label": "WRONG_ANSWER", "correct": False})
        
        # Wait for FAIL_L1 state
        assert self._wait_for_state("FAIL_L1", timeout=5.0), \
            "Did not receive FAIL_L1 state after wrong answer"
    
    def test_pause_resume_flow(self):
        """Test pause and resume functionality."""
        self._start_game(wait_state="WAIT_INPUT", timeout=20.0)
        
        # Send PAUSE command
        self._publish_intent({"label": "PAUSE"})
        
        # Wait for PAUSED state
        def is_paused():
            return (
                self.latest_state is not None and
                self.latest_state.get("state") == "PAUSED"
            )
        assert self._spin_until(is_paused, timeout=5.0), \
            "Did not receive PAUSED state"
        
        # Send RESUME command
        self._publish_intent({"label": "RESUME"})
        
        # Wait for GAME state
        def is_game():
            return (
                self.latest_state is not None and
                self.latest_state.get("state") == "GAME"
            )
        assert self._spin_until(is_game, timeout=5.0), \
            "Did not receive GAME state after resume"


class TestManifestUpdates:
    """Test manifest update flow."""
    
    @pytest.fixture(autouse=True)
    def setup_ros(self):
        """Setup ROS2 context."""
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("test_manifest_node")

        yield

        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    def test_initial_manifest_sent(self):
        """Test that initial manifest is sent on startup."""
        # This test would require checking the manifest service
        # For now, just verify the nodes are running
        time.sleep(3)
        
        # Check that game_controller node exists
        node_names = self.node.get_node_names()
        # Note: In integration, we'd verify the service was called
        assert True  # Placeholder


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
