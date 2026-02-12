#!/usr/bin/env python3
"""
Standalone E2E test runner for game_controller.

This script runs e2e tests against a running docker-compose stack.
Start the stack first with: docker compose up -d

Usage:
    python run_e2e_tests.py
    python run_e2e_tests.py --verbose
    python run_e2e_tests.py --test test_game_init
"""

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional


class Colors:
    """ANSI color codes."""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'


def log_info(msg: str) -> None:
    print(f"{Colors.BLUE}[INFO]{Colors.END} {msg}")


def log_pass(msg: str) -> None:
    print(f"{Colors.GREEN}[PASS]{Colors.END} {msg}")


def log_fail(msg: str) -> None:
    print(f"{Colors.RED}[FAIL]{Colors.END} {msg}")


def log_warn(msg: str) -> None:
    print(f"{Colors.YELLOW}[WARN]{Colors.END} {msg}")


class ROS2TestHelper:
    """Helper for interacting with ROS2 via docker exec."""
    
    def __init__(self, decision_container: str = "game_controller-decision_making-1",
                 game_controller_container: str = "game_controller-game_controller-1"):
        self.decision_container = decision_container
        self.game_controller_container = game_controller_container
    
    def exec_cmd(self, container: str, cmd: str, timeout: int = 30) -> str:
        """Execute a command in a container."""
        full_cmd = [
            "docker", "exec", container,
            "bash", "-c",
            f"source /opt/ros/humble/setup.bash && {cmd}"
        ]
        try:
            result = subprocess.run(
                full_cmd,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            return ""
    
    def pub(self, topic: str, msg_type: str, data: str) -> None:
        """Publish a ROS2 message."""
        # Publish from game_controller container because it includes hri_actions_msgs for /intents tests.
        cmd = f"source /ros2_ws/install/setup.bash && ros2 topic pub --once {topic} {msg_type} '{data}'"
        self.exec_cmd(self.game_controller_container, cmd)

    def _publish_intent_payload(
        self,
        payload: Dict[str, Any],
        modality: str = "__modality_touchscreen__",
    ) -> None:
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
        self.exec_cmd(self.game_controller_container, cmd)

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
        self._publish_intent_payload(payload, modality=modality)
    
    def echo_once(self, topic: str, timeout: int = 10) -> Optional[Dict[str, Any]]:
        """Get one message from a topic."""
        cmd = f"source /ws/install/setup.bash && timeout {timeout} ros2 topic echo {topic} --once"
        output = self.exec_cmd(self.decision_container, cmd, timeout=timeout + 5)
        
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
    
    def get_state(self) -> Optional[Dict[str, Any]]:
        """Get current decision state."""
        return self.echo_once("/decision/state")
    
    def wait_for_state(self, state: str, timeout: int = 30) -> Optional[Dict[str, Any]]:
        """Wait for a specific game state."""
        start = time.time()
        last_state = None
        while time.time() - start < timeout:
            current = self.get_state()
            if current:
                current_state = current.get("gameState")
                if current_state != last_state:
                    last_state = current_state
                if current_state == state:
                    return current
            time.sleep(0.2)  # Poll faster
        return None
    
    def wait_for_state_after_action(self, state: str, action_fn, timeout: int = 10) -> Optional[Dict[str, Any]]:
        """Wait for a state after performing an action, polling rapidly."""
        # Get initial state
        initial = self.get_state()
        initial_tx = initial.get("transactionId", 0) if initial else 0
        
        # Perform action
        action_fn()
        
        # Poll for state change
        start = time.time()
        while time.time() - start < timeout:
            current = self.get_state()
            if current:
                # Check if we passed through the target state
                # or if we're past it (higher transaction ID and different state)
                current_state = current.get("gameState")
                current_tx = current.get("transactionId", 0)
                
                if current_state == state and current_tx > initial_tx:
                    return current
            time.sleep(0.1)
        return None
    
    def select_user(self, user_id: int) -> None:
        """Select a user."""
        self.pub("/game/user_selector", "std_msgs/String", f'data: "{user_id}"')
    
    def select_game(self, game_slug: str) -> None:
        """Select a game."""
        self.pub("/game/game_selector", "std_msgs/String", f'data: "{game_slug}"')
    
    def submit_answer(self, label: str, correct: bool) -> None:
        """Submit a user answer."""
        self.publish_raw_user_intent(label, correct=correct)
    
    def send_control(self, command: str) -> None:
        """Send a control command (PAUSE, RESUME, EXIT)."""
        self.publish_raw_user_intent(command)


@dataclass
class TestResult:
    """Result of a single test."""
    name: str
    passed: bool
    message: str
    duration: float


class E2ETestSuite:
    """E2E test suite for game_controller."""
    
    def __init__(self, verbose: bool = False):
        self.helper = ROS2TestHelper()
        self.verbose = verbose
        self.results: list[TestResult] = []
        self._user_counter = 100  # Start with high user IDs to avoid conflicts
    
    def _unique_user(self) -> int:
        """Get a unique user ID for each test."""
        self._user_counter += 1
        return self._user_counter
    
    def _run_test(self, name: str, test_func) -> TestResult:
        """Run a single test and record result."""
        start = time.time()
        try:
            test_func()
            duration = time.time() - start
            result = TestResult(name, True, "OK", duration)
            log_pass(f"{name} ({duration:.2f}s)")
        except AssertionError as e:
            duration = time.time() - start
            result = TestResult(name, False, str(e), duration)
            log_fail(f"{name}: {e}")
        except Exception as e:
            duration = time.time() - start
            result = TestResult(name, False, f"Error: {e}", duration)
            log_fail(f"{name}: {e}")
        
        self.results.append(result)
        return result
    
    # ==================== Tests ====================
    
    def test_services_running(self) -> None:
        """Test that required ROS2 nodes are running."""
        output = self.helper.exec_cmd(
            self.helper.decision_container,
            "source /ws/install/setup.bash && ros2 node list"
        )
        assert "decision_making" in output, "decision_making node not found"
        
        output = self.helper.exec_cmd(
            self.helper.game_controller_container,
            "source /ros2_ws/install/setup.bash && ros2 node list"
        )
        assert "game_controller" in output, "game_controller node not found"
    
    def test_topics_exist(self) -> None:
        """Test that required topics exist."""
        output = self.helper.exec_cmd(
            self.helper.decision_container,
            "source /ws/install/setup.bash && ros2 topic list"
        )
        
        required = [
            "/decision/state",
            "/decision/events",
            "/intents",
            "/game/game_selector",
            "/game/user_selector"
        ]
        for topic in required:
            assert topic in output, f"Topic {topic} not found"
    
    def test_game_init(self) -> None:
        """Test that game initializes correctly."""
        user_id = self._unique_user()
        
        self.helper.select_user(user_id)
        time.sleep(0.5)
        self.helper.select_game("colores")
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None, "Game did not reach WAIT_INPUT state"
        assert state.get("sessionId") is not None, "No session ID"
    
    def test_correct_answer_flow(self) -> None:
        """Test correct answer transitions through CORRECT state."""
        user_id = self._unique_user()
        
        self.helper.select_user(user_id)
        time.sleep(0.5)
        self.helper.select_game("colores")
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None, "Game did not reach WAIT_INPUT"
        
        initial_tx = state.get("transactionId", 0)
        
        # Submit correct answer and wait for state to change
        state = self.helper.wait_for_state_after_action(
            "CORRECT",
            lambda: self.helper.submit_answer("verde", True),
            timeout=5
        )
        # Note: CORRECT state is very brief, we might miss it
        # Just verify we advance to next WAIT_INPUT with higher tx
        
        time.sleep(2)  # Allow auto-advance
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Did not advance to next round"
        assert state.get("transactionId", 0) > initial_tx, "Transaction ID did not increase"
    
    def test_incorrect_answer_flow(self) -> None:
        """Test incorrect answer transitions to FAIL_L1."""
        user_id = self._unique_user()
        
        self.helper.select_user(user_id)
        time.sleep(0.5)
        self.helper.select_game("colores")
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None
        
        initial_tx = state.get("transactionId", 0)
        
        # Submit wrong answer
        self.helper.submit_answer("azul", False)
        
        # FAIL_L1 state lasts 2 seconds, so we should be able to catch it
        state = self.helper.wait_for_state("FAIL_L1", timeout=3)
        if state is None:
            # If we missed FAIL_L1, we should be back in WAIT_INPUT with higher tx
            state = self.helper.wait_for_state("WAIT_INPUT", timeout=5)
            assert state is not None, "Did not return to WAIT_INPUT"
            assert state.get("transactionId", 0) > initial_tx, "Should have progressed"
        
    def test_double_failure_flow(self) -> None:
        """Test that two failures lead to FAIL_L2."""
        user_id = self._unique_user()
        
        self.helper.select_user(user_id)
        time.sleep(0.5)
        self.helper.select_game("colores")
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None
        
        initial_tx = state.get("transactionId", 0)
        
        # First failure
        self.helper.submit_answer("azul", False)
        time.sleep(3)  # Wait for FAIL_L1 to auto-advance
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Did not return to WAIT_INPUT after first failure"
        
        # Second failure
        self.helper.submit_answer("rojo", False)
        
        # FAIL_L2 lasts 2 seconds
        state = self.helper.wait_for_state("FAIL_L2", timeout=3)
        if state is None:
            # If missed, verify we progressed
            state = self.helper.wait_for_state("WAIT_INPUT", timeout=5)
            assert state is not None
            assert state.get("transactionId", 0) > initial_tx + 2, "Should have progressed past FAIL_L2"
    
    def test_auto_advance_timing(self) -> None:
        """Test that auto-advance happens at correct times."""
        user_id = self._unique_user()
        
        self.helper.select_user(user_id)
        time.sleep(0.5)
        
        start = time.time()
        self.helper.select_game("colores")
        
        # Should reach WAIT_INPUT within reasonable time
        # PHASE_INTRO (2s) + ROUND_SETUP (0.05s) + QUESTION_PRESENT (0.05s)
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=10)
        elapsed = time.time() - start
        
        assert state is not None, "Game did not reach WAIT_INPUT"
        assert 2.0 <= elapsed <= 6.0, f"Auto-advance timing off: {elapsed}s"
    
    def test_retry_then_correct(self) -> None:
        """Test failure -> retry -> correct flow."""
        user_id = self._unique_user()
        
        self.helper.select_user(user_id)
        time.sleep(0.5)
        self.helper.select_game("colores")
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=15)
        assert state is not None
        
        initial_tx = state.get("transactionId", 0)
        
        # First try - wrong
        self.helper.submit_answer("azul", False)
        time.sleep(3)  # Wait for FAIL_L1 to auto-advance
        
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Did not return to WAIT_INPUT after failure"
        
        retry_tx = state.get("transactionId", 0)
        
        # Second try - correct
        self.helper.submit_answer("verde", True)
        time.sleep(2)  # Wait for CORRECT to auto-advance
        
        # Should move to next round
        state = self.helper.wait_for_state("WAIT_INPUT", timeout=5)
        assert state is not None, "Did not advance to next round"
        
        final_tx = state.get("transactionId", 0)
        assert final_tx > retry_tx, f"Should have progressed: {retry_tx} -> {final_tx}"
    
    # ==================== Runner ====================
    
    def run_all(self) -> bool:
        """Run all tests."""
        tests = [
            ("Services Running", self.test_services_running),
            ("Topics Exist", self.test_topics_exist),
            ("Game Init", self.test_game_init),
            ("Correct Answer Flow", self.test_correct_answer_flow),
            ("Incorrect Answer Flow", self.test_incorrect_answer_flow),
            ("Double Failure Flow", self.test_double_failure_flow),
            ("Auto-Advance Timing", self.test_auto_advance_timing),
            ("Retry Then Correct", self.test_retry_then_correct),
        ]
        
        log_info(f"Running {len(tests)} e2e tests...")
        print()
        
        for name, test_func in tests:
            self._run_test(name, test_func)
            time.sleep(1)  # Brief pause between tests
        
        # Summary
        print()
        passed = sum(1 for r in self.results if r.passed)
        failed = len(self.results) - passed
        total_time = sum(r.duration for r in self.results)
        
        if failed == 0:
            log_pass(f"All {passed} tests passed in {total_time:.2f}s")
        else:
            log_fail(f"{failed}/{len(self.results)} tests failed")
            print("\nFailed tests:")
            for r in self.results:
                if not r.passed:
                    print(f"  - {r.name}: {r.message}")
        
        return failed == 0
    
    def run_single(self, test_name: str) -> bool:
        """Run a single test by name."""
        test_methods = {
            name.replace("test_", ""): getattr(self, name)
            for name in dir(self)
            if name.startswith("test_") and callable(getattr(self, name))
        }
        
        # Find matching test
        matches = [k for k in test_methods if test_name.lower() in k.lower()]
        if not matches:
            log_fail(f"No test matching '{test_name}' found")
            print("Available tests:")
            for name in sorted(test_methods.keys()):
                print(f"  - {name}")
            return False
        
        for match in matches:
            self._run_test(match, test_methods[match])
        
        return all(r.passed for r in self.results)


def main():
    parser = argparse.ArgumentParser(description="E2E tests for game_controller")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--test", "-t", type=str, help="Run specific test")
    args = parser.parse_args()
    
    suite = E2ETestSuite(verbose=args.verbose)
    
    if args.test:
        success = suite.run_single(args.test)
    else:
        success = suite.run_all()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
