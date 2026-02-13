"""E2E tests for game_controller.

These tests run against a full ROS2 stack including decision_making,
game_controller, and generic_ui backend.

Tests are organized by scenario:
1. Game Initialization - selecting user/game and starting
2. Question Flow - correct/incorrect answer handling
3. State Transitions - auto-advance timing
4. UI Updates - manifest patching
5. Control Commands - pause/resume/exit

Environment:
- `E2E_MANAGE_STACK=1` to have this module run compose up/down itself.
- `E2E_COMPOSE_FILE` to override compose file path.
- `E2E_COMPOSE_PROJECT` to override compose project name.
"""

import json
import os
import subprocess
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import pytest

try:
    from .e2e_utils import DockerE2EContext
except ImportError:
    from e2e_utils import DockerE2EContext


@dataclass
class ROS2Message:
    """Represents a captured ROS2 message."""
    topic: str
    data: Any
    timestamp: float


class DockerComposeTestRunner:
    """Helper class to run docker commands and interact with ROS2."""

    def __init__(
        self,
        compose_file: Optional[str] = None,
        compose_project: Optional[str] = None,
        manage_stack: Optional[bool] = None,
    ):
        self.compose_file = compose_file or os.getenv("E2E_COMPOSE_FILE", "docker-compose.yml")
        self.compose_project = compose_project or os.getenv("E2E_COMPOSE_PROJECT", "game_controller")
        if manage_stack is None:
            self.manage_stack = os.getenv("E2E_MANAGE_STACK", "").lower() in {"1", "true", "yes", "on"}
        else:
            self.manage_stack = manage_stack
        self.context = DockerE2EContext(
            compose_file=self.compose_file,
            compose_project=self.compose_project,
        )
        self.decision_service = self.context.decision_service
        self.game_controller_service = self.context.game_controller_service
        self.backend_service = self.context.backend_service
        self._captured_messages: List[ROS2Message] = []

    def preflight_reason(self) -> Optional[str]:
        """Return a skip reason when docker prerequisites are unavailable."""
        required_services: List[str] = []
        if not self.manage_stack:
            required_services = [self.decision_service, self.game_controller_service]
        return self.context.preflight_reason(
            required_services=required_services,
            require_compose=self.manage_stack,
        )

    def _compose(self, args: List[str], check: bool = False) -> subprocess.CompletedProcess:
        """Run a compose command using whichever compose implementation is available."""
        return self.context.compose(args, check=check)

    def _find_service_container(self, service: str) -> Optional[str]:
        """Resolve a compose service to a running container name."""
        return self.context.resolve_service_container(service)

    def stack_up(self) -> None:
        """Start compose stack when test runner owns lifecycle."""
        self._compose(["up", "-d"], check=True)

    def stack_down(self) -> None:
        """Stop compose stack when test runner owns lifecycle."""
        self._compose(["down"], check=True)

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

    def docker_exec_service(self, service: str, cmd: str, timeout: int = 30) -> str:
        """Execute a command in a service container resolved at runtime."""
        container = self._find_service_container(service)
        if not container:
            raise RuntimeError(f"Could not resolve container for service '{service}'")
        return self.docker_exec(container, cmd, timeout=timeout)

    def ros2_topic_pub(
        self,
        topic: str,
        msg_type: str,
        data: str,
        service: Optional[str] = None,
    ) -> None:
        """Publish a message to a ROS2 topic."""
        cmd = f"source /ros2_ws/install/setup.bash && ros2 topic pub --once {topic} {msg_type} '{data}'"
        self.docker_exec_service(service or self.game_controller_service, cmd)

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
        self.docker_exec_service(self.game_controller_service, cmd)

    def ros2_topic_echo_once(
        self,
        topic: str,
        service: Optional[str] = None,
        timeout: int = 10,
    ) -> Optional[Dict[str, Any]]:
        """Echo a single message from a topic."""
        cmd = f"source /ws/install/setup.bash && timeout {timeout} ros2 topic echo {topic} --once"
        output = self.docker_exec_service(
            service or self.decision_service,
            cmd,
            timeout=timeout + 5,
        )
        
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
        if self.context.compose_available():
            args = ["logs"]
            if lines and lines > 0:
                args.extend(["--tail", str(lines)])
            args.append(service)
            result = self._compose(args, check=False)
            if result.returncode == 0 and (result.stdout or result.stderr):
                return result.stdout + result.stderr

        container = self._find_service_container(service)
        if not container:
            return ""
        cmd = ["docker", "logs"]
        if lines and lines > 0:
            cmd.extend(["--tail", str(lines)])
        cmd.append(container)
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
        )
        return result.stdout + result.stderr

    def wait_for_log_contains(self, service: str, needle: str, timeout: int = 10, lines: int = 500) -> bool:
        """Poll service logs until a target string appears."""
        start = time.time()
        while time.time() - start < timeout:
            logs = self.get_logs(service, lines=lines)
            if needle in logs:
                return True
            time.sleep(0.25)
        return False
    
    def wait_for_state(
        self,
        expected_state: str,
        timeout: int = 30,
        service: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        """Wait for a specific game state."""
        start = time.time()
        while time.time() - start < timeout:
            state = self.ros2_topic_echo_once("/decision/state", service=service)
            if state and state.get("gameState") == expected_state:
                return state
            time.sleep(0.5)
        return None

    def wait_for_system_state(
        self,
        expected_state: str,
        timeout: int = 10,
        service: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        """Wait for a specific system state."""
        start = time.time()
        while time.time() - start < timeout:
            state = self.ros2_topic_echo_once("/decision/state", service=service)
            if state and state.get("state") == expected_state:
                return state
            time.sleep(0.5)
        return None

    def ensure_idle(self, timeout: int = 5) -> None:
        """Best-effort reset to IDLE before each test."""
        self.publish_raw_user_intent("EXIT")
        self.wait_for_system_state("IDLE", timeout=timeout)

    def start_fresh_game(self, user_id: int, game_slug: str = "colores", timeout: int = 15) -> Optional[Dict[str, Any]]:
        """Reset to IDLE, select user+game, and wait until input is expected."""
        self.ensure_idle(timeout=5)
        self.ros2_topic_pub("/game/user_selector", "std_msgs/String", f'data: "{user_id}"')
        time.sleep(0.5)
        self.ros2_topic_pub("/game/game_selector", "std_msgs/String", f'data: "{game_slug}"')
        return self.wait_for_state("WAIT_INPUT", timeout=timeout)


@pytest.fixture(scope="module")
def runner():
    """Create a test runner with optional compose up/down lifecycle."""
    runner = DockerComposeTestRunner()
    reason = runner.preflight_reason()

    if reason and not runner.manage_stack:
        pytest.skip(reason)

    stack_started = False

    if runner.manage_stack:
        if reason:
            pytest.skip(reason)
        runner.stack_up()
        stack_started = True
        startup_wait = int(os.getenv("E2E_STACK_STARTUP_WAIT_SEC", "20"))
        time.sleep(startup_wait)

        # After stack startup, ensure required services are actually reachable.
        reason = runner.context.preflight_reason(
            [runner.decision_service, runner.game_controller_service],
            require_compose=False,
        )
        if reason:
            if stack_started:
                runner.stack_down()
            pytest.skip(reason)

    yield runner

    if stack_started:
        runner.stack_down()


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
        state = runner.start_fresh_game(user_id=2, timeout=15)
        assert state is not None, "Game should start and reach WAIT_INPUT"
        assert runner.wait_for_log_contains(
            "game_controller",
            "Starting game: colores",
            timeout=5,
            lines=800,
        )
    
    def test_game_init_event_published(self, runner: DockerComposeTestRunner):
        """Test that GAME_INIT event is published with correct structure."""
        state = runner.start_fresh_game(user_id=22, timeout=15)
        assert state is not None
        assert runner.wait_for_log_contains(
            "game_controller",
            "Published /decision/events: type=GAME_INIT",
            timeout=5,
            lines=800,
        )
        assert runner.wait_for_log_contains("game_controller", "Starting game: colores", timeout=5, lines=800)


class TestStateTransitions:
    """Tests for game state transitions."""
    
    def test_auto_advance_from_phase_intro(self, runner: DockerComposeTestRunner):
        """Test auto-advance from PHASE_INTRO to ROUND_SETUP."""
        state = runner.start_fresh_game(user_id=3, timeout=15)
        assert state is not None, "Game should auto-advance to WAIT_INPUT"
    
    def test_correct_answer_transitions(self, runner: DockerComposeTestRunner):
        """Test that correct answer transitions through CORRECT state."""
        state = runner.start_fresh_game(user_id=31, timeout=15)
        assert state is not None

        runner.publish_raw_user_intent("verde", correct=True)

        assert runner.wait_for_log_contains("game_controller", "game=CORRECT", timeout=8, lines=1000)
        state = runner.wait_for_state("WAIT_INPUT", timeout=10)
        assert state is not None, "Should auto-advance to next round"
    
    def test_incorrect_answer_transitions_fail_l1(self, runner: DockerComposeTestRunner):
        """Test that incorrect answer transitions to FAIL_L1."""
        state = runner.start_fresh_game(user_id=32, timeout=15)
        assert state is not None

        runner.publish_raw_user_intent("azul", correct=False)

        assert runner.wait_for_log_contains("game_controller", "game=FAIL_L1", timeout=8, lines=1000)
    
    def test_second_incorrect_transitions_fail_l2(self, runner: DockerComposeTestRunner):
        """Test that second incorrect answer transitions to FAIL_L2."""
        state = runner.start_fresh_game(user_id=33, timeout=15)
        assert state is not None

        runner.publish_raw_user_intent("azul", correct=False)
        assert runner.wait_for_log_contains("game_controller", "game=FAIL_L1", timeout=8, lines=1000)
        state = runner.wait_for_state("WAIT_INPUT", timeout=10)
        assert state is not None, "Should return to WAIT_INPUT after FAIL_L1"

        runner.publish_raw_user_intent("rojo", correct=False)
        assert runner.wait_for_log_contains("game_controller", "game=FAIL_L2", timeout=8, lines=1000)


class TestUIManifestUpdates:
    """Tests for UI manifest updates."""
    
    def test_initial_manifest_sent(self, runner: DockerComposeTestRunner):
        """Test that initial manifest is sent on startup."""
        logs = runner.get_logs("game_controller", lines=0)
        assert any(
            token in logs for token in ["Initial manifest sent", "Sent manifest set request"]
        ), "Expected initial manifest lifecycle logs"
    
    def test_manifest_patched_on_state_change(self, runner: DockerComposeTestRunner):
        """Test that manifest is patched on state changes."""
        state = runner.start_fresh_game(user_id=4, timeout=15)
        assert state is not None
        assert runner.wait_for_log_contains("game_controller", "Sent manifest patch request", timeout=5, lines=1000)
        # Accept either controller-side success logs (legacy) or backend-side
        # patch confirmation (current gateway behavior).
        assert (
            runner.wait_for_log_contains("game_controller", "Manifest update successful", timeout=5, lines=1000)
            or runner.wait_for_log_contains("backend", "Manifest patched", timeout=5, lines=1000)
        )


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
        state = runner.start_fresh_game(user_id=5, timeout=15)
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
        state = runner.start_fresh_game(user_id=6, timeout=15)
        assert state is not None
        
        round_id = state.get("payload", {}).get("roundId")
        
        # Submit incorrect answer
        runner.publish_raw_user_intent("azul", correct=False)
        assert runner.wait_for_log_contains("game_controller", "game=FAIL_L1", timeout=8, lines=1200)
        
        # Wait for auto-advance back to WAIT_INPUT (same round)
        state = runner.wait_for_state("WAIT_INPUT", timeout=10)
        assert state is not None
        assert state.get("payload", {}).get("roundId") == round_id, "Should still be same round"
        
        # Submit correct answer
        runner.publish_raw_user_intent("verde", correct=True)
        assert runner.wait_for_log_contains("game_controller", "game=CORRECT", timeout=8, lines=1200)


class TestServiceAvailability:
    """Tests for service availability and connectivity."""
    
    def test_decision_making_node_running(self, runner: DockerComposeTestRunner):
        """Test that decision_making node is running."""
        output = runner.docker_exec_service(
            runner.decision_service,
            "source /ws/install/setup.bash && ros2 node list"
        )
        assert "decision_making" in output
    
    def test_game_controller_node_running(self, runner: DockerComposeTestRunner):
        """Test that game_controller node is running."""
        output = runner.docker_exec_service(
            runner.game_controller_service,
            "source /ros2_ws/install/setup.bash && ros2 node list"
        )
        assert "game_controller" in output
    
    def test_update_manifest_service_available(self, runner: DockerComposeTestRunner):
        """Test that UpdateManifest service is available."""
        output = runner.docker_exec_service(
            runner.decision_service,
            "source /ws/install/setup.bash && ros2 service list"
        )
        assert "/generic_ui/update_manifest" in output
    
    def test_required_topics_exist(self, runner: DockerComposeTestRunner):
        """Test that all required topics exist."""
        output = runner.docker_exec_service(
            runner.decision_service,
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
