"""Mock Decision Making FSM for isolated testing.

This mock simulates the decision_making node by:
1. Subscribing to /decision/events
2. Publishing state updates to /decision/state
3. Simulating a phase-aware FSM for colores integration flows
"""

import json
import threading
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
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

        # Game plan from GAME_INIT
        self._phase_sequence: List[str] = []
        self._rounds_by_phase: Dict[str, List[Dict[str, Any]]] = {}
        self._phase_index = 0
        self._question_index = 0
        self._attempt_count = 0

        # Publishers
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = QoSReliabilityPolicy.RELIABLE
        state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(
            String,
            "/decision/state",
            state_qos,
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
        """Publish initial IDLE state once."""
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
        self._session_id = int(payload.get("sessionId") or 1)
        self._transaction_counter = 0
        self._phase_sequence, self._rounds_by_phase = self._build_round_plan(payload)
        self._phase_index = 0
        self._question_index = 0
        self._attempt_count = 0

        self._publish_next_state(
            "GAME_START",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "gameInfo": payload.get("gameInfo", {}),
            },
        )

        self._run_after(0.3, self._start_phase_intro)

    def _build_round_plan(self, payload: Dict[str, Any]) -> tuple[List[str], Dict[str, List[Dict[str, Any]]]]:
        """Build ordered phases and rounds from GAME_INIT payload."""
        raw_rounds = payload.get("rounds")
        rounds: List[Dict[str, Any]] = raw_rounds if isinstance(raw_rounds, list) else []

        if not rounds:
            rounds = [self._fallback_round(phase="P1", round_id=1)]

        phase_sequence: List[str] = []
        rounds_by_phase: Dict[str, List[Dict[str, Any]]] = {}

        for index, round_cfg in enumerate(rounds, start=1):
            if not isinstance(round_cfg, dict):
                continue

            phase = str(round_cfg.get("phase") or "P1")
            if phase not in phase_sequence:
                phase_sequence.append(phase)

            normalized_round = dict(round_cfg)
            normalized_round.setdefault("roundId", index)
            normalized_round["question"] = self._normalize_question(
                normalized_round.get("question"),
                int(normalized_round["roundId"]),
            )
            rounds_by_phase.setdefault(phase, []).append(normalized_round)

        if not phase_sequence:
            fallback = self._fallback_round(phase="P1", round_id=1)
            phase_sequence = ["P1"]
            rounds_by_phase = {"P1": [fallback]}

        return phase_sequence, rounds_by_phase

    def _normalize_question(self, raw_question: Any, round_id: int) -> Dict[str, Any]:
        """Normalize question payload shape used by game_controller manifests."""
        question = dict(raw_question) if isinstance(raw_question, dict) else {}
        question.setdefault("questionId", round_id)
        question.setdefault("questionType", "selection")

        prompt = question.get("prompt") or question.get("text")
        question["prompt"] = str(prompt or "Selecciona la respuesta correcta.")

        options = question.get("options")
        if not isinstance(options, list) or not options:
            answer = str(question.get("answer") or "rojo")
            distractor = "azul" if answer.lower() != "azul" else "rojo"
            options = [
                {"id": answer, "label": answer, "correct": True},
                {"id": distractor, "label": distractor, "correct": False},
            ]
            question["options"] = options
        return question

    def _fallback_round(self, phase: str, round_id: int) -> Dict[str, Any]:
        """Create fallback round when GAME_INIT omits rounds."""
        return {
            "phase": phase,
            "roundId": round_id,
            "question": {
                "questionId": round_id,
                "questionType": "selection",
                "prompt": "¿De qué color es esto?",
                "answer": "rojo",
                "options": [
                    {"id": "rojo", "label": "rojo", "correct": True},
                    {"id": "azul", "label": "azul", "correct": False},
                ],
            },
        }

    def _run_after(self, delay_sec: float, callback) -> None:
        """Run callback in background after a small delay."""

        def _runner() -> None:
            threading.Event().wait(delay_sec)
            callback()

        threading.Thread(target=_runner, daemon=True).start()

    def _current_phase(self) -> Optional[str]:
        """Get current phase code from plan."""
        if self._phase_index < len(self._phase_sequence):
            return self._phase_sequence[self._phase_index]
        return None

    def _current_round(self) -> Optional[Dict[str, Any]]:
        """Get current round configuration from plan."""
        phase = self._current_phase()
        if not phase:
            return None
        rounds = self._rounds_by_phase.get(phase, [])
        if self._question_index < len(rounds):
            return rounds[self._question_index]
        return None

    def _has_next_round_in_phase(self) -> bool:
        """Check if current phase has another round after current question index."""
        phase = self._current_phase()
        if not phase:
            return False
        rounds = self._rounds_by_phase.get(phase, [])
        return self._question_index + 1 < len(rounds)

    def _start_phase_intro(self) -> None:
        """Publish PHASE_INTRO for current phase."""
        phase = self._current_phase()
        round_cfg = self._current_round()
        if not phase or not round_cfg:
            self._publish_next_state("IDLE", "IDLE")
            return

        self._publish_next_state(
            "PHASE_INTRO",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "phase": phase,
                "roundId": int(round_cfg.get("roundId", 1)),
            },
        )

    def _publish_question_present(self) -> None:
        """Publish QUESTION_PRESENT for current round."""
        phase = self._current_phase()
        round_cfg = self._current_round()
        if not phase or not round_cfg:
            self._publish_phase_complete()
            return

        question = round_cfg.get("question", {})
        self._attempt_count = 0
        self._publish_next_state(
            "QUESTION_PRESENT",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "phase": phase,
                "roundId": int(round_cfg.get("roundId", 1)),
                "questionId": int(question.get("questionId", self._question_index + 1)),
                "question": question,
            },
        )

    def _publish_wait_input(self) -> None:
        """Publish WAIT_INPUT for current round."""
        phase = self._current_phase()
        round_cfg = self._current_round()
        if not phase or not round_cfg:
            self._publish_next_state("IDLE", "IDLE")
            return

        question = round_cfg.get("question", {})
        self._publish_next_state(
            "WAIT_INPUT",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "phase": phase,
                "roundId": int(round_cfg.get("roundId", 1)),
                "questionId": int(question.get("questionId", self._question_index + 1)),
                "question": question,
            },
        )

    def _publish_phase_complete(self) -> None:
        """Publish PHASE_COMPLETE for current phase."""
        phase = self._current_phase() or "P1"
        self._publish_next_state(
            "PHASE_COMPLETE",
            "ACTIVE",
            payload={
                "sessionId": self._session_id,
                "phase": phase,
            },
        )

    def _advance_phase_or_finish(self) -> None:
        """Move to next phase or return to IDLE at end of plan."""
        if self._phase_index + 1 < len(self._phase_sequence):
            self._phase_index += 1
            self._question_index = 0
            self._attempt_count = 0
            self._start_phase_intro()
            return
        self._publish_next_state("IDLE", "IDLE")

    def _handle_on_complete(self, payload: Dict[str, Any]) -> None:
        """Handle ON_COMPLETE event."""
        transaction_id = payload.get("transactionId")
        if transaction_id != self._transaction_counter:
            self.get_logger().warn(
                f"Transaction ID mismatch: expected {self._transaction_counter}, "
                f"got {transaction_id}"
            )
            return

        current = self._current_game_state
        if current == "GAME_START":
            self._start_phase_intro()
        elif current == "PHASE_INTRO":
            self._publish_question_present()
        elif current == "QUESTION_PRESENT":
            self._publish_wait_input()
        elif current in ("FAIL_L1", "FAIL_L2"):
            self._publish_wait_input()
        elif current == "CORRECT":
            if self._has_next_round_in_phase():
                self._question_index += 1
                self._publish_question_present()
            else:
                self._publish_phase_complete()
        elif current == "PHASE_COMPLETE":
            self._advance_phase_or_finish()

    def _handle_user_intent(self, payload: Dict[str, Any]) -> None:
        """Handle USER_INTENT event."""
        transaction_id = payload.get("transactionId")
        if transaction_id != self._transaction_counter:
            self.get_logger().warn(
                f"Transaction ID mismatch in USER_INTENT: expected "
                f"{self._transaction_counter}, got {transaction_id}"
            )
            return

        if self._is_correct_intent(payload):
            self._attempt_count = 0
            self._publish_next_state(
                "CORRECT",
                "ACTIVE",
                payload={"sessionId": self._session_id, "phase": self._current_phase()},
            )
            return

        self._attempt_count += 1
        fail_state = "FAIL_L2" if self._attempt_count >= 2 else "FAIL_L1"
        self._publish_next_state(
            fail_state,
            "ACTIVE",
            payload={"sessionId": self._session_id, "phase": self._current_phase()},
        )

    def _is_correct_intent(self, payload: Dict[str, Any]) -> bool:
        """Infer correctness from payload and current round definition."""
        explicit_correct = payload.get("correct")
        if explicit_correct is not None:
            return bool(explicit_correct)

        value = payload.get("value")
        if value is None:
            answer = payload.get("answer")
            if isinstance(answer, dict):
                value = answer.get("value") or answer.get("label")
            else:
                value = answer
        if value is None:
            return False
        user_value = self._normalize_text(str(value))
        if not user_value:
            return False

        round_cfg = self._current_round()
        question = round_cfg.get("question", {}) if round_cfg else {}
        candidates: List[str] = []

        answer = question.get("answer")
        if answer:
            candidates.append(self._normalize_text(str(answer)))

        for key in ("acceptedAnswers", "accepted_answers"):
            accepted = question.get(key)
            if isinstance(accepted, list):
                candidates.extend(self._normalize_text(str(item)) for item in accepted if item)

        options = question.get("options")
        if isinstance(options, list):
            for option in options:
                if isinstance(option, dict) and option.get("correct"):
                    for field in ("label", "id", "value", "text"):
                        val = option.get(field)
                        if val:
                            candidates.append(self._normalize_text(str(val)))

        candidates = [candidate for candidate in candidates if candidate]
        if not candidates:
            return False

        if user_value in candidates:
            return True
        return any(candidate in user_value for candidate in candidates if len(candidate) > 1)

    def _normalize_text(self, value: str) -> str:
        """Normalize answer text for matching."""
        return value.strip().lower()

    def _handle_game_control(self, payload: Dict[str, Any]) -> None:
        """Handle GAME_CONTROL event (PAUSE, RESUME, EXIT)."""
        action = str(payload.get("command") or payload.get("action") or "").upper()

        if action == "PAUSE":
            self._publish_next_state(self._current_game_state, "PAUSED")
        elif action == "RESUME":
            self._publish_next_state(self._current_game_state, "ACTIVE")
        elif action == "EXIT":
            self._phase_sequence = []
            self._rounds_by_phase = {}
            self._phase_index = 0
            self._question_index = 0
            self._attempt_count = 0
            self._publish_next_state("IDLE", "IDLE")

    def _publish_next_state(
        self,
        game_state: str,
        system_state: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Increment transaction id and publish state."""
        self._transaction_counter += 1
        self._publish_state(game_state, system_state, payload)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
