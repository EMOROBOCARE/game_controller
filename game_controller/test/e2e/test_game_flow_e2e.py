"""E2E tests for game_controller.

These tests run against a full ROS2 stack including decision_making,
game_controller, and generic_ui backend.

Tests are organized by scenario:
1. Game Initialization - selecting user/game and starting
2. Question Flow - correct/incorrect answer handling
3. State Transitions - auto-advance timing
4. UI Updates - manifest patching
5. Control Commands - pause/resume/exit
"""

import json
import subprocess
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import pytest


@dataclass
class ROS2Message:
    """Represents a captured ROS2 message."""
    topic: str
    data: Any
    timestamp: float


class DockerComposeTestRunner:
    """Helper class to run docker-compose commands and interact with ROS2."""
    
    def __init__(self, compose_file: str = "docker-compose.yml"):
        self.compose_file = compose_file
        self._captured_messages: List[ROS2Message] = []
        
    def docker_exec(self, container: str, cmd: str, timeout: int = 30) -> str:
        """Execute a command in a container."""
        full_cmd = [
            "docker", "exec", container,
            "bash", "-c",
            f"source /opt/ros/humble/setup.bash && {cmd}"
        ]
        result = subprocess.run(
            full_cmd,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.stdout + result.stderr
    
    def ros2_topic_pub(self, topic: str, msg_type: str, data: str, container: str = "game_controller-decision_making-1") -> None:
        """Publish a message to a ROS2 topic."""
        cmd = f"source /ws/install/setup.bash && ros2 topic pub --once {topic} {msg_type} '{data}'"
        self.docker_exec(container, cmd)

    def publish_raw_user_intent(
        self,
        label: str,
        correct: Optional[bool] = None,
        modality: str = "__modality_touchscreen__",
    ) -> None:
        """Publish a RAW_USER_INPUT intent with embedded ui/input payload."""
        payload: Dict[str, Any] = {"label": label}
        if correct is not None:
            payload["correct"] = correct
        intent_data = json.dumps({"input": json.dumps(payload)})
        data = (
            "{intent: '__raw_user_input__', "
            f"data: '{intent_data}', modality: '{modality}'}}"
        )
        self.ros2_topic_pub("/intents", "hri_actions_msgs/msg/Intent", data)
    
    def ros2_topic_echo_once(self, topic: str, container: str = "game_controller-decision_making-1", timeout: int = 10) -> Optional[Dict[str, Any]]:
        """Echo a single message from a topic."""
        cmd = f"source /ws/install/setup.bash && timeout {timeout} ros2 topic echo {topic} --once"
        output = self.docker_exec(container, cmd, timeout=timeout + 5)
        
        # Parse the YAML-like output
        for line in output.split('\n'):
            if line.startswith('data:'):
                data_str = line.replace('data:', '').strip()
                if data_str.startswith("'") and data_str.endswith("'"):
                    data_str = data_str[1:-1]
                try:
                    return json.loads(data_str)
                except json.JSONDecodeError:
                    return {"raw": data_str}
        return None
    
    def get_logs(self, service: str, lines: int = 50) -> str:
        """Get logs from a service."""
        result = subprocess.run(
            ["docker", "compose", "-f", self.compose_file, "logs", "--tail", str(lines), service],
            capture_output=True,
            text=True
        )
        return result.stdout
    
    def wait_for_state(self, expected_state: str, timeout: int = 30, container: str = "game_controller-decision_making-1") -> Optional[Dict[str, Any]]:
        """Wait for a specific game state."""
        start = time.time()
        while time.time() - start < timeout:
            state = self.ros2_topic_echo_once("/decision/state", container)
            if state and state.get("gameState") == expected_state:
                return state
            time.sleep(0.5)
        return None

    def wait_for_system_state(self, expected_state: str, timeout: int = 10, container: str = "game_controller-decision_making-1") -> Optional[Dict[str, Any]]:
        """Wait for a specific system state."""
        start = time.time()
        while time.time() - start < timeout:
            state = self.ros2_topic_echo_once("/decision/state", container)
            if state and state.get("state") == expected_state:
                return state
            time.sleep(0.5)
        return None

    def ensure_idle(self, timeout: int = 5) -> None:
        """Best-effort reset to IDLE before each test."""
        self.publish_raw_user_intent("EXIT")
        self.wait_for_system_state("IDLE", timeout=timeout)


@pytest.fixture(scope="module")
def runner():
    """Create a test runner with docker-compose up/down lifecycle."""
    runner = DockerComposeTestRunner()
    
    # Start services
    subprocess.run(
        ["docker", "compose", "-f", "docker-compose.yml", "up", "-d"],
        check=True
    )
    
    # Wait for services to be healthy
    time.sleep(20)  # Allow time for healthchecks
    
    yield runner
    
    # Cleanup
    subprocess.run(
        ["docker", "compose", "-f", "docker-compose.yml", "down"],
        check=True
    )


@pytest.fixture(autouse=True)
def reset_session(runner: DockerComposeTestRunner):
    """Ensure each test starts from IDLE to avoid cross-test bleed."""
    runner.ensure_idle()


class TestGameInitialization:
    """Tests for game initialization flow."""
    
    def test_user_selection(self, runner: DockerComposeTestRunner):
        """Test that user selection is received by game_controller."""
        runner.ros2_topic_pub(
            "/game/user_selector",
            "std_msgs/String",
            'data: "1"'
        )
        
        # Check logs for user selection
        logs = runner.get_logs("game_controller")
        assert "User selected: 1" in logs
    
    def test_game_selection_starts_game(self, runner: DockerComposeTestRunner):
        """Test that selecting a game starts the game session."""
        # Ensure user is selected
        runner.ros2_topic_pub(
            "/game/user_selector",
            "std_msgs/String",
            'data: "2"'
        )
        time.sleep(0.5)
        
        # Select game
        runner.ros2_topic_pub(
            "/game/game_selector",
            "std_msgs/String",
            'data: "colores"'
        )
        
        # Wait for PHASE_INTRO state
        state = runner.wait_for_state("PHASE_INTRO", timeout=5)
        assert state is not None, "Game should enter PHASE_INTRO state"
    
    def test_game_init_event_published(self, runner: DockerComposeTestRunner):
        """Test that GAME_INIT event is published with correct structure."""
        logs = runner.get_logs("game_controller")
        assert "Published event: GAME_INIT" in logs
        assert "Starting game: colores" in logs


class TestStateTransitions:
    """Tests for game state transitions."""
    
    def test_auto_advance_from_phase_intro(self, runner: DockerComposeTestRunner):
        """Test auto-advance from PHASE_INTRO to ROUND_SETUP."""
        # Start fresh game
        runner.ros2_topic_pub("/game/user_selector", "std_msgs/String", 'data: "3"')
        time.sleep(0.5)
        runner.ros2_topic_pub("/game/game_selector", "std_msgs/String", 'data: "colores"')
        
        # Wait for WAIT_INPUT (after auto-advance through intro -> round_setup -> question_present)
        state = runner.wait_for_state("WAIT_INPUT", timeout=10)
        assert state is not None, "Game should auto-advance to WAIT_INPUT"
    
    def test_correct_answer_transitions(self, runner: DockerComposeTestRunner):
        """Test that correct answer transitions through CORRECT state."""
        # Submit correct answer
        runner.publish_raw_user_intent("verde", correct=True)
        
        # Wait for CORRECT state
        state = runner.wait_for_state("CORRECT", timeout=5)
        assert state is not None, "Should transition to CORRECT state"
        
        # Then auto-advance back to WAIT_INPUT (next round)
        state = runner.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Should auto-advance to next round"
    
    def test_incorrect_answer_transitions_fail_l1(self, runner: DockerComposeTestRunner):
        """Test that incorrect answer transitions to FAIL_L1."""
        # Submit incorrect answer
        runner.publish_raw_user_intent("azul", correct=False)
        
        # Wait for FAIL_L1 state
        state = runner.wait_for_state("FAIL_L1", timeout=5)
        assert state is not None, "Should transition to FAIL_L1 state"
    
    def test_second_incorrect_transitions_fail_l2(self, runner: DockerComposeTestRunner):
        """Test that second incorrect answer transitions to FAIL_L2."""
        # Wait for FAIL_L1 to auto-advance back to WAIT_INPUT
        state = runner.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Should return to WAIT_INPUT after FAIL_L1"
        
        # Submit another incorrect answer
        runner.publish_raw_user_intent("rojo", correct=False)
        
        # Wait for FAIL_L2 state
        state = runner.wait_for_state("FAIL_L2", timeout=5)
        assert state is not None, "Should transition to FAIL_L2 state on second failure"


class TestUIManifestUpdates:
    """Tests for UI manifest updates."""
    
    def test_initial_manifest_sent(self, runner: DockerComposeTestRunner):
        """Test that initial manifest is sent on startup."""
        logs = runner.get_logs("game_controller", lines=100)
        assert "Initial manifest sent" in logs
    
    def test_manifest_patched_on_state_change(self, runner: DockerComposeTestRunner):
        """Test that manifest is patched on state changes."""
        # Start a fresh game
        runner.ros2_topic_pub("/game/user_selector", "std_msgs/String", 'data: "4"')
        time.sleep(0.5)
        runner.ros2_topic_pub("/game/game_selector", "std_msgs/String", 'data: "colores"')
        
        # Wait for game to start
        runner.wait_for_state("WAIT_INPUT", timeout=10)
        
        # Check logs for manifest patches
        logs = runner.get_logs("game_controller", lines=50)
        assert "Sent manifest patch request" in logs
        assert "Manifest update successful" in logs


class TestControlCommands:
    """Tests for game control commands (pause, resume, exit)."""
    
    def test_pause_command(self, runner: DockerComposeTestRunner):
        """Test that PAUSE command is forwarded."""
        runner.publish_raw_user_intent("PAUSE")
        
        logs = runner.get_logs("game_controller")
        assert "Published event: GAME_CONTROL" in logs or "PAUSE" in logs
    
    def test_exit_command(self, runner: DockerComposeTestRunner):
        """Test that EXIT command is forwarded."""
        runner.publish_raw_user_intent("EXIT")
        
        logs = runner.get_logs("game_controller")
        # Should contain either GAME_CONTROL event or EXIT in logs
        assert any(x in logs for x in ["GAME_CONTROL", "EXIT"])


class TestEndToEndGameFlow:
    """Full end-to-end game flow tests."""
    
    def test_complete_round_flow(self, runner: DockerComposeTestRunner):
        """Test a complete round from start to correct answer."""
        # Start fresh game
        runner.ros2_topic_pub("/game/user_selector", "std_msgs/String", 'data: "5"')
        time.sleep(0.5)
        runner.ros2_topic_pub("/game/game_selector", "std_msgs/String", 'data: "colores"')
        
        # Wait for WAIT_INPUT
        state = runner.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None, "Game should reach WAIT_INPUT"
        
        initial_tx = state.get("transactionId", 0)
        
        # Submit correct answer
        runner.publish_raw_user_intent("verde", correct=True)
        
        # Wait for next WAIT_INPUT (next round)
        time.sleep(2)  # Wait for CORRECT state auto-advance
        state = runner.wait_for_state("WAIT_INPUT", timeout=10)
        assert state is not None, "Should advance to next round"
        
        # Transaction ID should have increased
        new_tx = state.get("transactionId", 0)
        assert new_tx > initial_tx, f"Transaction ID should increase: {initial_tx} -> {new_tx}"
    
    def test_failure_retry_flow(self, runner: DockerComposeTestRunner):
        """Test the failure-retry flow (FAIL_L1 -> retry -> correct)."""
        # Start fresh game
        runner.ros2_topic_pub("/game/user_selector", "std_msgs/String", 'data: "6"')
        time.sleep(0.5)
        runner.ros2_topic_pub("/game/game_selector", "std_msgs/String", 'data: "colores"')
        
        # Wait for WAIT_INPUT
        state = runner.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None
        
        round_id = state.get("payload", {}).get("roundId")
        
        # Submit incorrect answer
        runner.publish_raw_user_intent("azul", correct=False)
        
        # Wait for FAIL_L1
        state = runner.wait_for_state("FAIL_L1", timeout=5)
        assert state is not None, "Should be in FAIL_L1"
        
        # Wait for auto-advance back to WAIT_INPUT (same round)
        state = runner.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None
        assert state.get("payload", {}).get("roundId") == round_id, "Should still be same round"
        
        # Submit correct answer
        runner.publish_raw_user_intent("verde", correct=True)
        
        # Wait for CORRECT
        state = runner.wait_for_state("CORRECT", timeout=5)
        assert state is not None, "Should transition to CORRECT"


class TestServiceAvailability:
    """Tests for service availability and connectivity."""
    
    def test_decision_making_node_running(self, runner: DockerComposeTestRunner):
        """Test that decision_making node is running."""
        output = runner.docker_exec(
            "game_controller-decision_making-1",
            "source /ws/install/setup.bash && ros2 node list"
        )
        assert "decision_making" in output
    
    def test_game_controller_node_running(self, runner: DockerComposeTestRunner):
        """Test that game_controller node is running."""
        output = runner.docker_exec(
            "game_controller-game_controller-1",
            "source /ros2_ws/install/setup.bash && ros2 node list"
        )
        assert "game_controller" in output
    
    def test_update_manifest_service_available(self, runner: DockerComposeTestRunner):
        """Test that UpdateManifest service is available."""
        output = runner.docker_exec(
            "game_controller-decision_making-1",
            "source /ws/install/setup.bash && ros2 service list"
        )
        assert "/generic_ui/update_manifest" in output
    
    def test_required_topics_exist(self, runner: DockerComposeTestRunner):
        """Test that all required topics exist."""
        output = runner.docker_exec(
            "game_controller-decision_making-1",
            "source /ws/install/setup.bash && ros2 topic list"
        )
        
        required_topics = [
            "/decision/state",
            "/decision/events",
            "/intents",
            "/game/game_selector",
            "/game/user_selector",
            "/game/current_user"
        ]
        
        for topic in required_topics:
            assert topic in output, f"Topic {topic} should exist"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
