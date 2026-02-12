"""
Pytest-based E2E tests for game_controller.

These tests run against a docker-compose stack.
Start the stack before running: docker compose up -d
Wait for health checks: docker compose ps (all should be healthy)

Run with: pytest -v test_game_controller_e2e.py
"""

import json
import subprocess
import time
from typing import Any, Dict, Optional

import pytest

try:
    from .e2e_utils import DockerE2EContext
except ImportError:
    from e2e_utils import DockerE2EContext


class ROS2Helper:
    """Helper for ROS2 interactions via docker exec."""

    CONTEXT = DockerE2EContext()
    DECISION_SERVICE = CONTEXT.decision_service
    GAME_CONTROLLER_SERVICE = CONTEXT.game_controller_service

    @classmethod
    def _require_container(cls, service: str) -> str:
        container = cls.CONTEXT.resolve_service_container(service)
        if not container:
            raise RuntimeError(f"Could not resolve container for service '{service}'")
        return container

    @classmethod
    def preflight_reason(cls) -> Optional[str]:
        return cls.CONTEXT.preflight_reason(
            [cls.DECISION_SERVICE, cls.GAME_CONTROLLER_SERVICE],
            require_compose=False,
        )
    
    @classmethod
    def exec_cmd(cls, container: str, cmd: str, timeout: int = 30) -> str:
        """Execute a command in a container."""
        full_cmd = [
            "docker", "exec", container,
            "bash", "-c",
            f"source /opt/ros/humble/setup.bash && {cmd}"
        ]
        try:
            result = subprocess.run(full_cmd, capture_output=True, text=True, timeout=timeout)
            return result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            return ""

    @classmethod
    def exec_decision(cls, cmd: str, timeout: int = 30) -> str:
        return cls.exec_cmd(cls._require_container(cls.DECISION_SERVICE), cmd, timeout=timeout)

    @classmethod
    def exec_game_controller(cls, cmd: str, timeout: int = 30) -> str:
        return cls.exec_cmd(
            cls._require_container(cls.GAME_CONTROLLER_SERVICE),
            cmd,
            timeout=timeout,
        )
    
    @classmethod
    def pub(cls, topic: str, msg_type: str, data: str) -> None:
        """Publish a ROS2 message."""
        # Publish from game_controller container because it includes hri_actions_msgs for /intents tests.
        cmd = f"source /ros2_ws/install/setup.bash && ros2 topic pub --once {topic} {msg_type} '{data}'"
        cls.exec_game_controller(cmd)

    @classmethod
    def publish_raw_user_intent(cls, payload: Dict[str, Any], modality: str = "__modality_touchscreen__") -> None:
        """Publish an Intent with JSON payload encoded in msg.data."""
        payload_json = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        cmd = (
            "source /ros2_ws/install/setup.bash && "
            "python3 - <<'PY'\n"
            "import time\n"
            "import rclpy\n"
            "from hri_actions_msgs.msg import Intent\n"
            f"payload_json = {payload_json!r}\n"
            f"modality = {modality!r}\n"
            "rclpy.init()\n"
            "node = rclpy.create_node('e2e_intent_publisher')\n"
            "pub = node.create_publisher(Intent, '/intents', 10)\n"
            "msg = Intent()\n"
            "msg.intent = '__raw_user_input__'\n"
            "msg.data = payload_json\n"
            "msg.modality = modality\n"
            "for _ in range(3):\n"
            "    pub.publish(msg)\n"
            "    rclpy.spin_once(node, timeout_sec=0.05)\n"
            "    time.sleep(0.05)\n"
            "node.destroy_node()\n"
            "rclpy.shutdown()\n"
            "PY"
        )
        cls.exec_game_controller(cmd)
    
    @classmethod
    def get_state(cls) -> Optional[Dict[str, Any]]:
        """Get current decision state."""
        cmd = "source /ws/install/setup.bash && timeout 5 ros2 topic echo /decision/state --once"
        output = cls.exec_decision(cmd, timeout=10)
        
        for line in output.split('\n'):
            if line.startswith('data:'):
                data_str = line.replace('data:', '').strip()
                if data_str.startswith("'") and data_str.endswith("'"):
                    data_str = data_str[1:-1]
                try:
                    return json.loads(data_str)
                except json.JSONDecodeError:
                    return None
        return None
    
    @classmethod
    def wait_for_state(cls, state: str, timeout: int = 30) -> Optional[Dict[str, Any]]:
        """Wait for a specific game state."""
        start = time.time()
        while time.time() - start < timeout:
            current = cls.get_state()
            if current and current.get("gameState") == state:
                return current
            time.sleep(0.2)
        return None

    @classmethod
    def wait_for_system_state(cls, state: str, timeout: int = 10) -> Optional[Dict[str, Any]]:
        """Wait for a specific system state."""
        start = time.time()
        while time.time() - start < timeout:
            current = cls.get_state()
            if current and current.get("state") == state:
                return current
            time.sleep(0.2)
        return None
    
    @classmethod
    def select_user(cls, user_id: int) -> None:
        """Select a user."""
        cls.pub("/game/user_selector", "std_msgs/String", f'data: "{user_id}"')
    
    @classmethod
    def select_game(cls, game_slug: str) -> None:
        """Select a game."""
        cls.pub("/game/game_selector", "std_msgs/String", f'data: "{game_slug}"')
    
    @classmethod
    def submit_answer(cls, label: str, correct: bool) -> None:
        """Submit a user answer."""
        cls.publish_raw_user_intent({"label": label, "correct": correct})

    @classmethod
    def send_control(cls, command: str) -> None:
        """Send a control command through intents."""
        cls.publish_raw_user_intent({"label": command})

    @classmethod
    def start_fresh_game(cls, user_id: int, timeout: int = 15) -> Optional[Dict[str, Any]]:
        """Reset to IDLE and start a fresh game session."""
        cls.send_control("EXIT")
        cls.wait_for_system_state("IDLE", timeout=5)
        cls.select_user(user_id)
        time.sleep(0.5)
        cls.select_game("colores")
        return cls.wait_for_state("WAIT_INPUT", timeout=timeout)


@pytest.fixture(scope="module")
def user_counter():
    """Provide unique user IDs for each test."""
    class Counter:
        def __init__(self):
            self.value = 200
        
        def next(self) -> int:
            self.value += 1
            return self.value
    
    return Counter()


@pytest.fixture(scope="module", autouse=True)
def e2e_context_ready():
    """Skip module when docker/compose context is unavailable."""
    reason = ROS2Helper.preflight_reason()
    if reason:
        pytest.skip(reason)


@pytest.fixture(autouse=True)
def reset_session_state(e2e_context_ready):
    """Best-effort reset to IDLE before each test."""
    ROS2Helper.send_control("EXIT")
    ROS2Helper.wait_for_system_state("IDLE", timeout=5)


class TestServiceAvailability:
    """Test that services are running."""
    
    def test_decision_making_running(self):
        """decision_making node should be running."""
        output = ROS2Helper.exec_decision("source /ws/install/setup.bash && ros2 node list")
        assert "decision_making" in output
    
    def test_game_controller_running(self):
        """game_controller node should be running."""
        output = ROS2Helper.exec_game_controller("source /ros2_ws/install/setup.bash && ros2 node list")
        assert "game_controller" in output
    
    def test_required_topics_exist(self):
        """All required topics should exist."""
        output = ROS2Helper.exec_decision("source /ws/install/setup.bash && ros2 topic list")
        
        required = [
            "/decision/state",
            "/decision/events",
            "/intents",
            "/game/game_selector",
            "/game/user_selector"
        ]
        for topic in required:
            assert topic in output, f"Missing topic: {topic}"


class TestGameInitialization:
    """Test game initialization flow."""
    
    def test_game_starts_on_selection(self, user_counter):
        """Game should start when user and game are selected."""
        user_id = user_counter.next()
        state = ROS2Helper.start_fresh_game(user_id, timeout=15)
        assert state is not None, "Game should reach WAIT_INPUT state"
        assert state.get("sessionId") is not None, "Session ID should be set"


class TestAnswerFlow:
    """Test answer handling flow."""
    
    def test_correct_answer_advances_round(self, user_counter):
        """Correct answer should advance to next round."""
        user_id = user_counter.next()
        state = ROS2Helper.start_fresh_game(user_id, timeout=15)
        assert state is not None
        initial_tx = state.get("transactionId", 0)
        
        ROS2Helper.submit_answer("verde", True)
        time.sleep(2)  # Wait for CORRECT auto-advance
        
        state = ROS2Helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Should advance to next round"
        assert state.get("transactionId", 0) > initial_tx
    
    def test_incorrect_answer_allows_retry(self, user_counter):
        """Incorrect answer should allow retry on same round."""
        user_id = user_counter.next()
        state = ROS2Helper.start_fresh_game(user_id, timeout=15)
        assert state is not None
        initial_tx = state.get("transactionId", 0)
        
        ROS2Helper.submit_answer("azul", False)
        time.sleep(3)  # Wait for FAIL_L1 auto-advance
        
        state = ROS2Helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None
        assert state.get("transactionId", 0) > initial_tx


class TestAutoAdvance:
    """Test auto-advance timing."""
    
    def test_phase_intro_auto_advances(self, user_counter):
        """PHASE_INTRO should auto-advance within expected time."""
        user_id = user_counter.next()

        ROS2Helper.send_control("EXIT")
        ROS2Helper.wait_for_system_state("IDLE", timeout=5)
        ROS2Helper.select_user(user_id)
        time.sleep(0.5)

        start = time.time()
        ROS2Helper.select_game("colores")
        
        # Should reach WAIT_INPUT within ~5 seconds
        # PHASE_INTRO (2s) + ROUND_SETUP (0.05s) + QUESTION_PRESENT (0.05s)
        state = ROS2Helper.wait_for_state("WAIT_INPUT", timeout=10)
        elapsed = time.time() - start
        
        assert state is not None
        assert 2.0 <= elapsed <= 8.0, f"Auto-advance took {elapsed}s"


class TestFullGameFlow:
    """Full game flow integration tests."""
    
    def test_failure_then_success_flow(self, user_counter):
        """Test failure -> retry -> success flow."""
        user_id = user_counter.next()
        state = ROS2Helper.start_fresh_game(user_id, timeout=15)
        assert state is not None
        initial_tx = state.get("transactionId", 0)
        
        # First try - fail
        ROS2Helper.submit_answer("azul", False)
        time.sleep(3)
        
        state = ROS2Helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None
        
        # Second try - succeed
        ROS2Helper.submit_answer("verde", True)
        time.sleep(2)
        
        state = ROS2Helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None
        assert state.get("transactionId", 0) > initial_tx + 3


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
