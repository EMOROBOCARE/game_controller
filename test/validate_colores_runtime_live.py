#!/usr/bin/env python3
"""Live runtime validator for Colores behavior contract (P1-P6).

Run this inside a running game_controller stack container:

  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  python3 /ros2_ws/src/game_controller/test/validate_colores_runtime_live.py
"""

from __future__ import annotations

import json
import time
import unicodedata
from collections import defaultdict
from typing import Any, Dict, Optional, Tuple

import rclpy
from generic_ui_interfaces.srv import GetManifest
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

TARGET_PHASES = ["P1", "P2", "P3", "P4", "P5", "P6"]


def _norm(text: Any) -> str:
    value = str(text or "").strip().lower()
    value = unicodedata.normalize("NFD", value)
    return "".join(ch for ch in value if unicodedata.category(ch) != "Mn")


def _extract_answers(question: Dict[str, Any]) -> Tuple[Optional[str], Optional[str]]:
    options = question.get("options") if isinstance(question.get("options"), list) else []
    correct = None
    wrong = None

    for opt in options:
        if not isinstance(opt, dict):
            continue
        oid = opt.get("id") or opt.get("label")
        if oid is None:
            continue
        if bool(opt.get("correct")) and correct is None:
            correct = str(oid)
        elif not bool(opt.get("correct")) and wrong is None:
            wrong = str(oid)

    if not correct:
        answer = question.get("answer")
        if answer is not None and str(answer).strip():
            correct = str(answer)

    if not wrong and options and correct is not None:
        for opt in options:
            if not isinstance(opt, dict):
                continue
            oid = opt.get("id") or opt.get("label")
            if oid is None:
                continue
            if str(oid) != str(correct):
                wrong = str(oid)
                break

    return correct, wrong


class ColoresLiveValidator(Node):
    def __init__(self) -> None:
        super().__init__("colores_live_validator")

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = QoSReliabilityPolicy.RELIABLE
        state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.latest_state: Optional[Dict[str, Any]] = None
        self.state_history: list[Dict[str, Any]] = []

        self.latest_game_init: Optional[Dict[str, Any]] = None
        self.round_to_phase: Dict[int, str] = {}
        self.round_to_question: Dict[int, Dict[str, Any]] = {}
        self.question_to_phase: Dict[int, str] = {}
        self.question_to_question: Dict[int, Dict[str, Any]] = {}

        self.create_subscription(String, "/decision/state", self._on_state, state_qos)
        self.create_subscription(String, "/decision/events", self._on_event, 10)

        self.user_pub = self.create_publisher(String, "/game/user_selector", 10)
        self.game_pub = self.create_publisher(String, "/game/game_selector", 10)
        self.ui_input_pub = self.create_publisher(String, "/ui/input", 10)

        self.get_manifest_cli = self.create_client(GetManifest, "/generic_ui/get_manifest")

    def _on_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        self.latest_state = payload
        self.state_history.append(payload)
        if len(self.state_history) > 500:
            self.state_history = self.state_history[-500:]

    def _on_event(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(data, dict):
            return
        if str(data.get("type") or "").upper() != "GAME_INIT":
            return

        payload = data.get("payload") if isinstance(data.get("payload"), dict) else None
        if not payload:
            return
        self.latest_game_init = payload

        rounds = payload.get("rounds") if isinstance(payload.get("rounds"), list) else []
        for rd in rounds:
            if not isinstance(rd, dict):
                continue
            rid = rd.get("id")
            phase = str(rd.get("phase") or "").upper()
            question = rd.get("question") if isinstance(rd.get("question"), dict) else {}
            qid = question.get("questionId")
            if rid is not None and phase:
                self.round_to_phase[int(rid)] = phase
                self.round_to_question[int(rid)] = question
            if qid is not None and phase:
                self.question_to_phase[int(qid)] = phase
                self.question_to_question[int(qid)] = question

    def spin_until(self, predicate, timeout_sec: float) -> bool:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_state(self, predicate, timeout_sec: float, after_tx: Optional[int] = None) -> Dict[str, Any]:
        matched: Dict[str, Any] = {}

        def _pred() -> bool:
            state = self.latest_state
            if not isinstance(state, dict):
                return False
            tx = int(state.get("transactionId") or 0)
            if after_tx is not None and tx <= after_tx:
                return False
            if predicate(state):
                matched["value"] = state
                return True
            return False

        if not self.spin_until(_pred, timeout_sec):
            raise TimeoutError(f"Timeout waiting for state ({timeout_sec}s)")
        return matched["value"]

    def wait_game_init(self, timeout_sec: float = 20.0) -> Dict[str, Any]:
        if not self.spin_until(lambda: isinstance(self.latest_game_init, dict), timeout_sec):
            raise TimeoutError("Timeout waiting GAME_INIT")
        return self.latest_game_init  # type: ignore[return-value]

    def get_game_screen_cfg(self, timeout_sec: float = 4.0) -> Dict[str, Any]:
        if not self.get_manifest_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/generic_ui/get_manifest unavailable")
        req = GetManifest.Request()
        fut = self.get_manifest_cli.call_async(req)
        if not self.spin_until(lambda: fut.done(), timeout_sec):
            raise TimeoutError("GetManifest timeout")
        response = fut.result()
        manifest = json.loads(response.manifest_json)
        for inst in manifest.get("instances", []):
            if isinstance(inst, dict) and inst.get("id") == "game_screen":
                cfg = inst.get("config")
                if isinstance(cfg, dict):
                    return cfg
        raise RuntimeError("game_screen config missing")

    def wait_cfg(self, predicate, timeout_sec: float = 8.0) -> Dict[str, Any]:
        end = time.monotonic() + timeout_sec
        last_error: Optional[Exception] = None
        while time.monotonic() < end:
            try:
                cfg = self.get_game_screen_cfg(timeout_sec=2.0)
                if predicate(cfg):
                    return cfg
            except Exception as exc:
                last_error = exc
            time.sleep(0.15)
        if last_error is not None:
            raise TimeoutError(f"Timeout waiting manifest cfg: {last_error}")
        raise TimeoutError("Timeout waiting manifest cfg")

    def publish_user(self, user_id: int = 88, name: str = "Validator") -> None:
        payload = {"userName": name, "user": {"id": str(user_id), "name": name}}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.user_pub.publish(msg)

    def publish_game(self, phases: list[str]) -> None:
        payload = {
            "game": {"slug": "colores"},
            "level": {"id": 1, "name": "Nivel 1"},
            "difficulty": "basic",
            "phases": phases,
            "roundsPerPhase": 1,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.game_pub.publish(msg)

    def publish_answer(self, value: str, matching: bool = False) -> None:
        if matching:
            payload = {"leftId": value, "rightId": value, "correct": True}
        else:
            payload = {"label": value}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        for _ in range(3):
            self.ui_input_pub.publish(msg)
            time.sleep(0.05)

    def publish_control(self, action: str) -> None:
        msg = String()
        msg.data = json.dumps({"action": action}, ensure_ascii=False)
        for _ in range(2):
            self.ui_input_pub.publish(msg)
            time.sleep(0.05)

    def resolve_phase(self, state_payload: Dict[str, Any]) -> str:
        phase = str(state_payload.get("phase") or "").upper()
        if phase:
            return phase
        rid = state_payload.get("roundId")
        if rid is not None:
            resolved = self.round_to_phase.get(int(rid))
            if resolved:
                return resolved
        qid = state_payload.get("questionId")
        if qid is not None:
            resolved = self.question_to_phase.get(int(qid))
            if resolved:
                return resolved
        return ""

    def resolve_question(self, state_payload: Dict[str, Any]) -> Dict[str, Any]:
        if isinstance(state_payload.get("question"), dict):
            return state_payload["question"]
        rid = state_payload.get("roundId")
        if rid is not None and int(rid) in self.round_to_question:
            return self.round_to_question[int(rid)]
        qid = state_payload.get("questionId")
        if qid is not None and int(qid) in self.question_to_question:
            return self.question_to_question[int(qid)]
        return {}


def validate_wait_input_contract(phase: str, cfg: Dict[str, Any]) -> None:
    answer_type = str(cfg.get("answerType") or "")
    input_disabled = cfg.get("inputDisabled")
    options = cfg.get("options") if isinstance(cfg.get("options"), list) else []
    items = cfg.get("items") if isinstance(cfg.get("items"), list) else []

    if phase == "P1":
        assert answer_type == "match", f"P1 WAIT_INPUT answerType expected match, got {answer_type}"
        assert input_disabled is False, f"P1 WAIT_INPUT inputDisabled expected False, got {input_disabled}"
    elif phase == "P2":
        assert answer_type == "none", f"P2 WAIT_INPUT answerType expected none, got {answer_type}"
        assert input_disabled is True, f"P2 WAIT_INPUT inputDisabled expected True, got {input_disabled}"
    elif phase == "P3":
        assert answer_type == "button", f"P3 WAIT_INPUT answerType expected button, got {answer_type}"
        assert input_disabled is False, f"P3 WAIT_INPUT inputDisabled expected False, got {input_disabled}"
    elif phase == "P4":
        assert answer_type == "button", f"P4 WAIT_INPUT answerType expected button, got {answer_type}"
        assert input_disabled is False, f"P4 WAIT_INPUT inputDisabled expected False, got {input_disabled}"
        labels = {_norm(opt.get("label") or opt.get("id")) for opt in (options or items) if isinstance(opt, dict)}
        assert "si" in labels and "no" in labels, f"P4 WAIT_INPUT expected si/no labels, got {labels}"
    elif phase == "P5":
        assert answer_type == "button", f"P5 WAIT_INPUT answerType expected button, got {answer_type}"
        assert input_disabled is True, f"P5 WAIT_INPUT inputDisabled expected True, got {input_disabled}"
        visible = options or items
        assert visible, "P5 WAIT_INPUT expected visible options/items"
        assert all(bool(opt.get("disabled")) for opt in visible if isinstance(opt, dict)), "P5 WAIT_INPUT expected all options disabled"
    elif phase == "P6":
        assert answer_type == "none", f"P6 WAIT_INPUT answerType expected none, got {answer_type}"
        assert input_disabled is True, f"P6 WAIT_INPUT inputDisabled expected True, got {input_disabled}"
        assert len(options) == 0, f"P6 WAIT_INPUT expected no options, got {len(options)}"
    else:
        raise AssertionError(f"Unexpected phase in WAIT_INPUT: {phase}")


def wait_input_contract_matches(phase: str, cfg: Dict[str, Any]) -> bool:
    try:
        validate_wait_input_contract(phase, cfg)
    except AssertionError:
        return False
    return True


def main() -> None:
    rclpy.init()
    node = ColoresLiveValidator()

    last_tx = None
    steps = 0
    wait_counts: Dict[str, int] = defaultdict(int)
    phase_seen = set()
    phase_complete = set()
    failed_once = set()
    pending_retry_correct = set()
    p5_correct_validated = False
    last_resolved_phase = ""

    try:
        try:
            idle = node.wait_state(lambda s: str(s.get("state") or "").upper() == "IDLE", timeout_sec=3.0)
        except TimeoutError:
            idle = None

        if idle is None:
            print("[live] Session not idle; sending STOP until IDLE")
            for _ in range(6):
                node.publish_control("STOP")
                try:
                    idle = node.wait_state(lambda s: str(s.get("state") or "").upper() == "IDLE", timeout_sec=5.0)
                    break
                except TimeoutError:
                    idle = None
            if idle is None:
                raise TimeoutError("Could not recover IDLE state before validation")

        last_tx = int(idle.get("transactionId") or 0)
        print(f"[live] IDLE tx={last_tx}")

        node.publish_user()
        time.sleep(0.2)
        node.publish_game(TARGET_PHASES)
        print("[live] Published game selection for phases P1..P6")

        node.wait_game_init(timeout_sec=20.0)
        print("[live] GAME_INIT received")

        while steps < 400:
            steps += 1
            state = node.wait_state(
                lambda s: (
                    str(s.get("state") or "").upper() == "IDLE"
                    or str(s.get("gameState") or "").upper() in {"WAIT_INPUT", "FAIL_L1", "CORRECT", "PHASE_COMPLETE"}
                ),
                timeout_sec=35.0,
                after_tx=last_tx,
            )
            tx = int(state.get("transactionId") or 0)
            last_tx = tx
            system_state = str(state.get("state") or "").upper()
            game_state = str(state.get("gameState") or "").upper()
            payload = state.get("payload") if isinstance(state.get("payload"), dict) else {}
            phase = node.resolve_phase(payload)
            if not phase and game_state in {"FAIL_L1", "CORRECT", "PHASE_COMPLETE"}:
                phase = last_resolved_phase
            if phase:
                last_resolved_phase = phase

            print(f"[live] tx={tx} state={system_state}/{game_state} phase={phase or '-'}")

            if system_state == "IDLE":
                break

            if game_state == "PHASE_COMPLETE" and phase:
                phase_complete.add(phase)
                continue

            if game_state == "FAIL_L1" and phase:
                pending_retry_correct.add(phase)
                continue

            if game_state == "CORRECT":
                if phase == "P5":
                    cfg = node.wait_cfg(
                        lambda c: (
                            str(c.get("phase") or "").upper() == "P5"
                            and str((c.get("question") or {}).get("text") or "").strip().startswith("Aquí")
                        ),
                        timeout_sec=8.0,
                    )
                    options = cfg.get("options") if isinstance(cfg.get("options"), list) else []
                    items = cfg.get("items") if isinstance(cfg.get("items"), list) else []
                    visible = options or items
                    assert visible, "P5 CORRECT expected options/items for highlight"
                    assert any(bool(opt.get("highlighted")) for opt in visible if isinstance(opt, dict)), "P5 CORRECT expected highlighted option"
                    assert str(cfg.get("answerType") or "") == "button", f"P5 CORRECT answerType expected button, got {cfg.get('answerType')}"
                    p5_correct_validated = True
                continue

            if game_state != "WAIT_INPUT":
                continue

            if not phase:
                raise AssertionError(f"WAIT_INPUT without resolved phase: {state}")

            phase_seen.add(phase)
            wait_counts[phase] += 1

            cfg = node.wait_cfg(
                lambda c: str(c.get("phase") or "").upper() == phase and wait_input_contract_matches(phase, c),
                timeout_sec=8.0,
            )
            validate_wait_input_contract(phase, cfg)

            question = node.resolve_question(payload)
            correct, wrong = _extract_answers(question)
            if not correct:
                raise AssertionError(f"No correct answer resolved for phase {phase}: {question}")

            if phase in pending_retry_correct:
                node.publish_answer(correct, matching=(phase == "P1"))
                pending_retry_correct.remove(phase)
                print(f"[live] phase={phase} retry -> published CORRECT='{correct}'")
                continue

            if phase in {"P1", "P3", "P4"} and phase not in failed_once and wrong:
                node.publish_answer(wrong, matching=(phase == "P1"))
                failed_once.add(phase)
                print(f"[live] phase={phase} published WRONG='{wrong}'")
                continue

            node.publish_answer(correct, matching=(phase == "P1"))
            print(f"[live] phase={phase} published CORRECT='{correct}'")

        missing = [p for p in TARGET_PHASES if p not in phase_seen]
        assert not missing, f"Missing WAIT_INPUT phases: {missing}"
        assert wait_counts["P4"] >= 2, f"Expected P4 two WAIT_INPUT rounds, got {wait_counts['P4']}"
        assert p5_correct_validated, "P5 CORRECT highlight + 'Aquííí' not validated"

        summary = {
            "status": "PASS",
            "phases_wait_input_seen": sorted(phase_seen),
            "wait_counts": dict(wait_counts),
            "phase_complete_seen": sorted(phase_complete),
            "p5_correct_validated": p5_correct_validated,
        }
        print(json.dumps(summary, ensure_ascii=False, indent=2))

    except Exception as exc:
        print(f"VALIDATION_FAILED: {type(exc).__name__}: {exc}")
        print("STATE_TAIL=")
        print(json.dumps(node.state_history[-20:], ensure_ascii=False, indent=2))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
