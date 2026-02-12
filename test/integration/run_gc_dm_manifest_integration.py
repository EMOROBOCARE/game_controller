#!/usr/bin/env python3
"""Headless integration harness: game_controller ↔ decision_making ↔ generic_ui backend.

Runs inside a Docker container and:
- Validates the initial (menu) manifest emitted by game_controller
- Publishes menu-style selections (user + game + config)
- Drives the session through correct/incorrect answers
- Exercises PAUSE/RESUME/STOP/RESET/SKIP_PHASE flows
- Logs manifest snapshots so you can review them offline
"""

from __future__ import annotations

import json
import os
import time
import traceback
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from game_controller.ui.manifest_builder import rewrite_asset_url

try:
    from generic_ui_interfaces.srv import GetManifest

    HAS_GET_MANIFEST = True
except Exception:  # pragma: no cover - depends on container build
    GetManifest = None  # type: ignore[assignment]
    HAS_GET_MANIFEST = False

try:
    from hri_actions_msgs.msg import Intent

    HAS_INTENT = True
except Exception:  # pragma: no cover - depends on container build
    Intent = None  # type: ignore[assignment]
    HAS_INTENT = False


def _now() -> float:
    return time.monotonic()


def _json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False)


def _safe_json_loads(raw: str) -> Optional[Any]:
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


@dataclass(frozen=True)
class DecisionSnapshot:
    state: str
    game_state: Optional[str]
    transaction_id: int
    payload: Dict[str, Any]
    raw: Dict[str, Any]


@dataclass(frozen=True)
class ManifestSnapshot:
    manifest_hash: str
    manifest: Dict[str, Any]


@dataclass(frozen=True)
class RunConfig:
    name: str
    user_id: int
    user_name: str
    game_slug: str
    phases: List[str]
    difficulty: str = "basic"
    rounds_per_phase: int = 1
    stop_on_phase: Optional[str] = None
    skip_phase_on_phase: Optional[str] = None
    reset_on_phase: Optional[str] = None
    pause_once: bool = False


@dataclass
class RunResult:
    name: str
    phases_requested: List[str]
    phases_seen: Set[str]
    phases_answered_correct: Set[str]
    phases_failed_once: Set[str]
    exited_early: bool


class IntegrationHarness(Node):
    def __init__(self) -> None:
        super().__init__("gc_dm_manifest_integration")

        self._latest_state: Optional[DecisionSnapshot] = None
        self._state_history: List[DecisionSnapshot] = []
        self._latest_event: Optional[Dict[str, Any]] = None
        self._event_history: List[Dict[str, Any]] = []
        self._latest_game_init: Optional[Dict[str, Any]] = None
        self._round_id_to_phase: Dict[int, str] = {}
        self._question_id_to_phase: Dict[int, str] = {}

        # decision_making publishes /decision/state with TRANSIENT_LOCAL durability (latched).
        # Use TRANSIENT_LOCAL here as well so the harness receives the latest state even if it
        # starts after the first publish.
        decision_state_qos = QoSProfile(depth=1)
        decision_state_qos.reliability = QoSReliabilityPolicy.RELIABLE
        decision_state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._state_sub = self.create_subscription(
            String,
            "/decision/state",
            self._on_state,
            decision_state_qos,
        )

        self._events_sub = self.create_subscription(
            String,
            "/decision/events",
            self._on_decision_event,
            10,
        )

        self._user_pub = self.create_publisher(String, "/game/user_selector", 10)
        self._game_pub = self.create_publisher(String, "/game/game_selector", 10)
        self._ui_input_pub = self.create_publisher(String, "/ui/input", 10)

        # Fallback publisher if hri_actions_msgs is unavailable in the container.
        self._decision_events_pub = self.create_publisher(String, "/decision/events", 10)

        self._intent_pub = None
        if HAS_INTENT:
            self._intent_pub = self.create_publisher(Intent, "/intents", 10)

        self._ui_input_sub = self.create_subscription(
            String,
            "/ui/input",
            self._on_ui_input,
            10,
        )

        self._get_manifest_cli = None
        if HAS_GET_MANIFEST:
            self._get_manifest_cli = self.create_client(GetManifest, "/generic_ui/get_manifest")

        self._results_dir = os.environ.get("TEST_RESULTS_DIR", "/test_results")
        if not os.path.isdir(self._results_dir):
            self._results_dir = "."

        self._manifest_log_path = os.path.join(self._results_dir, "manifest_log.jsonl")
        self._state_log_path = os.path.join(self._results_dir, "decision_state_log.jsonl")
        self._event_log_path = os.path.join(self._results_dir, "decision_event_log.jsonl")
        self._report_json_path = os.path.join(self._results_dir, "report.json")
        self._report_md_path = os.path.join(self._results_dir, "report.md")

        # Truncate previous logs to keep each run self-contained.
        for path in (
            self._manifest_log_path,
            self._state_log_path,
            self._event_log_path,
            self._report_json_path,
            self._report_md_path,
        ):
            try:
                with open(path, "w", encoding="utf-8"):
                    pass
            except Exception:
                # Directory may be read-only; logging will surface later.
                pass

        self._steps: List[Dict[str, Any]] = []
        self._run_results: List[RunResult] = []

        self.get_logger().info(f"Manifest log: {self._manifest_log_path}")
        self.get_logger().info(f"State log: {self._state_log_path}")
        self.get_logger().info(f"Event log: {self._event_log_path}")
        self.get_logger().info(f"Report: {self._report_md_path}")

    # -------------------- ROS callbacks --------------------
    def _on_state(self, msg: String) -> None:
        raw = _safe_json_loads(msg.data)
        if not isinstance(raw, dict):
            return
        snapshot = DecisionSnapshot(
            state=str(raw.get("state") or ""),
            game_state=raw.get("gameState"),
            transaction_id=int(raw.get("transactionId") or 0),
            payload=raw.get("payload") if isinstance(raw.get("payload"), dict) else {},
            raw=raw,
        )
        self._latest_state = snapshot
        self._state_history.append(snapshot)
        self._append_jsonl(self._state_log_path, {"ts": _now(), "state": raw})

    def _on_decision_event(self, msg: String) -> None:
        raw = _safe_json_loads(msg.data)
        if not isinstance(raw, dict):
            return
        self._latest_event = raw
        self._event_history.append(raw)
        self._append_jsonl(self._event_log_path, {"ts": _now(), "event": raw})

        etype = str(raw.get("type") or "").upper().strip()
        if etype == "GAME_INIT":
            payload = raw.get("payload")
            if not isinstance(payload, dict):
                return
            self._latest_game_init = payload
            self._index_game_init(payload)

    def _on_ui_input(self, msg: String) -> None:
        """Bridge /ui/input to /intents (RAW_USER_INPUT) to emulate the real UI pipeline."""
        if not HAS_INTENT or self._intent_pub is None:
            return
        intent = Intent()
        intent.intent = Intent.RAW_USER_INPUT
        intent.modality = Intent.MODALITY_TOUCHSCREEN
        intent.source = Intent.REMOTE_SUPERVISOR
        intent.priority = 128
        intent.confidence = 1.0
        # communication_hub convention: Intent.data is JSON with {input: "<raw ui input>"}
        intent.data = _json_dumps({"input": msg.data})
        self._intent_pub.publish(intent)

    # -------------------- logging helpers --------------------
    def _append_jsonl(self, path: str, record: Dict[str, Any]) -> None:
        try:
            with open(path, "a", encoding="utf-8") as f:
                f.write(_json_dumps(record) + "\n")
        except Exception as exc:
            self.get_logger().warning(f"Failed writing {path}: {exc}")

    def log_manifest(self, step: str, state: Optional[DecisionSnapshot]) -> ManifestSnapshot:
        manifest = self.get_manifest()
        record = {
            "ts": _now(),
            "step": step,
            "decision": state.raw if state else None,
            "manifest_hash": manifest.manifest_hash,
            "manifest": manifest.manifest,
        }
        self._append_jsonl(self._manifest_log_path, record)
        return manifest

    # -------------------- wait helpers --------------------
    def spin_until(self, predicate: Callable[[], bool], timeout_sec: float) -> bool:
        end = _now() + timeout_sec
        while _now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_for_state(
        self,
        predicate: Callable[[DecisionSnapshot], bool],
        timeout_sec: float,
    ) -> DecisionSnapshot:
        ok = self.spin_until(
            lambda: self._latest_state is not None and predicate(self._latest_state),
            timeout_sec=timeout_sec,
        )
        _require(ok, f"Timed out waiting for decision state ({timeout_sec}s)")
        _require(self._latest_state is not None, "No decision state received")
        return self._latest_state

    def wait_for_event(
        self,
        predicate: Callable[[Dict[str, Any]], bool],
        timeout_sec: float,
    ) -> Dict[str, Any]:
        """Wait for a matching /decision/events message *after* this call."""
        start_len = len(self._event_history)
        matched: Optional[Dict[str, Any]] = None

        def _has_match() -> bool:
            nonlocal matched
            for ev in self._event_history[start_len:]:
                if predicate(ev):
                    matched = ev
                    return True
            return False

        ok = self.spin_until(_has_match, timeout_sec=timeout_sec)
        _require(ok, f"Timed out waiting for decision event ({timeout_sec}s)")
        _require(isinstance(matched, dict), "Event match is missing/invalid")
        assert isinstance(matched, dict)
        return matched

    # -------------------- manifest helpers --------------------
    def get_manifest(self, timeout_sec: float = 5.0) -> ManifestSnapshot:
        _require(self._get_manifest_cli is not None, "GetManifest service is unavailable in this container")
        assert self._get_manifest_cli is not None
        if not self._get_manifest_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Timed out waiting for /generic_ui/get_manifest service")

        request = GetManifest.Request()
        future = self._get_manifest_cli.call_async(request)
        ok = self.spin_until(lambda: future.done(), timeout_sec=timeout_sec)
        _require(ok, "Timed out waiting for GetManifest response")

        response = future.result()
        manifest = _safe_json_loads(response.manifest_json)
        _require(isinstance(manifest, dict), "GetManifest returned non-dict manifest")
        self.validate_manifest(manifest)
        return ManifestSnapshot(manifest_hash=str(response.manifest_hash), manifest=manifest)

    def wait_for_manifest(
        self,
        predicate: Callable[[Dict[str, Any]], bool],
        timeout_sec: float,
    ) -> ManifestSnapshot:
        last: Optional[ManifestSnapshot] = None
        end = _now() + timeout_sec
        while _now() < end:
            last = self.get_manifest(timeout_sec=min(3.0, timeout_sec))
            if predicate(last.manifest):
                return last
            time.sleep(0.1)
        raise AssertionError(f"Timed out waiting for expected manifest ({timeout_sec}s). Last: {last}")

    def validate_manifest(self, manifest: Dict[str, Any]) -> None:
        _require(isinstance(manifest.get("version"), int), "Manifest missing int 'version'")
        _require(isinstance(manifest.get("componentRegistry"), dict), "Manifest missing dict 'componentRegistry'")
        _require(isinstance(manifest.get("ops"), dict), "Manifest missing dict 'ops'")
        _require(isinstance(manifest.get("layout"), dict), "Manifest missing dict 'layout'")
        _require(isinstance(manifest.get("instances"), list), "Manifest missing list 'instances'")

        instances = manifest.get("instances") or []
        ids: List[str] = []
        for idx, inst in enumerate(instances):
            _require(isinstance(inst, dict), f"Instance {idx} is not an object")
            _require(isinstance(inst.get("id"), str) and inst["id"], f"Instance {idx} missing 'id'")
            _require(isinstance(inst.get("component"), str) and inst["component"], f"Instance {idx} missing 'component'")
            config = inst.get("config")
            _require(config is None or isinstance(config, dict), f"Instance {inst.get('id')} has non-dict 'config'")
            ids.append(inst["id"])

        _require(len(ids) == len(set(ids)), "Instance ids are not unique")

        layout = manifest.get("layout") or {}
        items = layout.get("items") if isinstance(layout.get("items"), list) else []
        for item in items:
            if not isinstance(item, dict):
                continue
            ref = item.get("instanceId")
            if isinstance(ref, str):
                _require(ref in ids, f"Layout references unknown instanceId: {ref}")

    def manifest_has_instances(self, manifest: Dict[str, Any], required_ids: List[str]) -> bool:
        instances = manifest.get("instances") if isinstance(manifest.get("instances"), list) else []
        present = {inst.get("id") for inst in instances if isinstance(inst, dict)}
        return all(req in present for req in required_ids)

    def instance_config(self, manifest: Dict[str, Any], instance_id: str) -> Dict[str, Any]:
        instances = manifest.get("instances") if isinstance(manifest.get("instances"), list) else []
        for inst in instances:
            if not isinstance(inst, dict) or inst.get("id") != instance_id:
                continue
            cfg = inst.get("config")
            return cfg if isinstance(cfg, dict) else {}
        return {}

    # -------------------- publish helpers --------------------
    def publish_user_selection(self, user_id: int, user_name: str = "Pepe") -> None:
        payload = {
            "userName": user_name,
            "user": {"id": str(user_id), "name": user_name},
        }
        msg = String()
        msg.data = _json_dumps(payload)
        self._user_pub.publish(msg)

    def publish_game_selection(
        self,
        slug: str,
        phases: List[str],
        difficulty: str = "basic",
        rounds_per_phase: int = 1,
    ) -> None:
        payload = {
            "game": {
                "slug": slug,
            },
            "level": {"id": 1, "name": "Nivel 1"},
            "phases": phases,
            "difficulty": difficulty,
            "roundsPerPhase": rounds_per_phase,
        }
        msg = String()
        msg.data = _json_dumps(payload)
        self._game_pub.publish(msg)

    def publish_ui_action(self, label: str, correct: Optional[bool] = None) -> None:
        """Publish a user action as /ui/input (preferred) or /decision/events (fallback).

        Preferred path emulates the real stack:
          /ui/input (std_msgs/String) -> (bridge) -> /intents (hri_actions_msgs/Intent) -> game_controller
        """
        input_payload: Dict[str, Any] = {"label": label}
        if correct is not None:
            input_payload["correct"] = correct

        if HAS_INTENT and self._intent_pub is not None:
            msg = String()
            msg.data = _json_dumps(input_payload)
            self._ui_input_pub.publish(msg)
            return

        # Fallback: bypass game_controller input translation by publishing directly to decision_making.
        latest = self._latest_state
        _require(latest is not None, "No decision state available for fallback publish")
        if label.upper() in {"PAUSE", "RESUME", "RESTART", "RESET", "EXIT", "STOP", "BACK", "SKIP_PHASE", "SKIP"}:
            event = {"type": "GAME_CONTROL", "payload": {"command": label.upper()}}
        else:
            if correct is None:
                correct = self._compute_correct_from_history(label)
            # In fallback mode we must provide transactionId + correctness ourselves.
            payload: Dict[str, Any] = {
                "transactionId": latest.transaction_id,
                "value": label,
                "modality": "touch",
            }
            if correct is not None:
                payload["correct"] = bool(correct)
            event = {
                "type": "USER_INTENT",
                "payload": payload,
            }
        msg = String()
        msg.data = _json_dumps(event)
        self._decision_events_pub.publish(msg)

    def _compute_correct_from_history(self, label: str) -> Optional[bool]:
        """Best-effort correctness when publishing directly to /decision/events."""
        for snap in reversed(self._state_history):
            if (snap.game_state or "").upper() != "QUESTION_PRESENT":
                continue
            question = snap.payload.get("question")
            if not isinstance(question, dict):
                continue
            options = question.get("options") if isinstance(question.get("options"), list) else []
            if options:
                for opt in options:
                    if not isinstance(opt, dict) or not opt.get("correct"):
                        continue
                    if label == opt.get("id") or label == opt.get("label"):
                        return True
                return False
            answer = question.get("answer")
            if isinstance(answer, str) and answer:
                return label == answer
        return None

    # -------------------- scenario helpers --------------------
    def _expected_intro_text(self, payload: Dict[str, Any]) -> str:
        intro = str(payload.get("introduction") or "")
        phase = str(payload.get("phase") or "")
        if not intro:
            intro = "Vamos a empezar."
        return f"Fase {phase}: {intro}" if phase else intro

    def _extract_correct_and_wrong(self, question: Dict[str, Any]) -> Tuple[str, str]:
        options = question.get("options") if isinstance(question.get("options"), list) else []
        answer = str(question.get("answer") or "")

        if options:
            correct_opt = next((o for o in options if o.get("correct")), None)
            if correct_opt is not None:
                # Options have explicit correct flag
                wrong_opt = next((o for o in options if not o.get("correct")), None)
                correct_value = str(correct_opt.get("id") or correct_opt.get("label") or answer)
                wrong_value = str((wrong_opt or {}).get("id") or (wrong_opt or {}).get("label") or "__wrong__")
                return (correct_value, wrong_value)

            # Options lack correct flag: use question.answer to identify
            if answer:
                norm_answer = answer.lower().strip()
                wrong_opt = next(
                    (o for o in options if str(o.get("id") or o.get("label") or "").lower().strip() != norm_answer),
                    None,
                )
                wrong_value = str((wrong_opt or {}).get("id") or (wrong_opt or {}).get("label") or "__wrong__")
                return (answer, wrong_value)

        # Voice phases may have no options.
        correct_value = answer
        wrong_value = "__wrong__"
        return (correct_value, wrong_value)

    def _wait_for_user_intent(
        self,
        value: str,
        expected_transaction_id: int,
        timeout_sec: float,
    ) -> Dict[str, Any]:
        ev = self.wait_for_event(
            lambda e: (
                str(e.get("type") or "").upper() == "USER_INTENT"
                and str((e.get("payload") or {}).get("value") or "") == str(value)
            ),
            timeout_sec=timeout_sec,
        )
        payload = ev.get("payload") if isinstance(ev.get("payload"), dict) else {}
        tx = payload.get("transactionId")
        _require(
            int(tx) == int(expected_transaction_id),
            f"USER_INTENT transactionId mismatch (got={tx} expected={expected_transaction_id})",
        )
        _require(isinstance(payload.get("correct"), bool), "USER_INTENT payload missing boolean 'correct'")
        _require(bool(payload.get("modality")), "USER_INTENT payload missing 'modality'")
        return ev

    def _record_step(self, step: str, ok: bool, details: Optional[Dict[str, Any]] = None) -> None:
        entry = {"ts": _now(), "step": step, "ok": ok, "details": details or {}}
        self._steps.append(entry)
        status = "PASS" if ok else "FAIL"
        self.get_logger().info(f"[{status}] {step}")

    def _write_report(self, success: bool, error: Optional[str], started_at: float) -> None:
        duration_s = _now() - started_at
        report = {
            "success": success,
            "error": error,
            "duration_s": duration_s,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "artifacts": {
                "manifest_log_jsonl": self._manifest_log_path,
                "decision_state_log_jsonl": self._state_log_path,
                "decision_event_log_jsonl": self._event_log_path,
                "report_json": self._report_json_path,
                "report_md": self._report_md_path,
            },
            "runs": [
                {
                    "name": r.name,
                    "phases_requested": sorted(set(r.phases_requested)),
                    "phases_seen": sorted(r.phases_seen),
                    "phases_answered_correct": sorted(r.phases_answered_correct),
                    "phases_failed_once": sorted(r.phases_failed_once),
                    "exited_early": r.exited_early,
                }
                for r in self._run_results
            ],
            "steps": self._steps,
            "notes": {
                "ui_actions_path": "/ui/input -> (bridge) -> /intents -> game_controller",
                "ui_docs": "UI_integration.md",
                "gc_docs": "GC_integration.md",
            },
        }

        try:
            with open(self._report_json_path, "w", encoding="utf-8") as f:
                f.write(_json_dumps(report) + "\n")
        except Exception as exc:
            self.get_logger().warning(f"Failed writing report.json: {exc}")

        # Markdown report for quick reading.
        lines: List[str] = []
        lines.append("# game_controller ↔ decision_making integration report")
        lines.append("")
        lines.append(f"- Success: `{success}`")
        lines.append(f"- Duration: `{duration_s:.1f}s`")
        if error:
            lines.append(f"- Error: `{error}`")
        lines.append(f"- ROS_DOMAIN_ID: `{os.environ.get('ROS_DOMAIN_ID', '')}`")
        lines.append("")
        lines.append("## Artifacts")
        lines.append(f"- `{self._manifest_log_path}`")
        lines.append(f"- `{self._state_log_path}`")
        lines.append(f"- `{self._event_log_path}`")
        lines.append("")
        lines.append("## Runs")
        for r in self._run_results:
            lines.append(
                f"- `{r.name}` requested={sorted(set(r.phases_requested))} "
                f"seen={sorted(r.phases_seen)} correct={sorted(r.phases_answered_correct)} "
                f"fail_once={sorted(r.phases_failed_once)} exited_early={r.exited_early}"
            )
        lines.append("")
        lines.append("## Notes")
        lines.append("- This test publishes UI events to `/ui/input` and bridges them to `/intents` inside the runner container (no browser).")
        lines.append("- For UI work, see `UI_integration.md` and `GC_integration.md`.")
        lines.append("")
        lines.append("## Step checklist")
        for s in self._steps:
            status = "PASS" if s.get("ok") else "FAIL"
            lines.append(f"- `{status}` `{s.get('step')}`")
        lines.append("")

        try:
            with open(self._report_md_path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
        except Exception as exc:
            self.get_logger().warning(f"Failed writing report.md: {exc}")

    def _index_game_init(self, payload: Dict[str, Any]) -> None:
        self._round_id_to_phase.clear()
        self._question_id_to_phase.clear()

        rounds = payload.get("rounds")
        if not isinstance(rounds, list):
            return
        for r in rounds:
            if not isinstance(r, dict):
                continue
            phase = str(r.get("phase") or "").strip().upper()
            rid = r.get("id")
            if isinstance(rid, int) and phase:
                self._round_id_to_phase[rid] = phase
            q = r.get("question")
            if isinstance(q, dict):
                qid = q.get("questionId")
                if isinstance(qid, int) and phase:
                    self._question_id_to_phase[qid] = phase

    def _wait_for_game_init(self, timeout_sec: float) -> Dict[str, Any]:
        ok = self.spin_until(lambda: self._latest_game_init is not None, timeout_sec=timeout_sec)
        _require(ok, f"Timed out waiting for GAME_INIT on /decision/events ({timeout_sec}s)")
        _require(isinstance(self._latest_game_init, dict), "GAME_INIT payload missing/invalid")
        assert isinstance(self._latest_game_init, dict)
        _require(isinstance(self._latest_game_init.get("rounds"), list), "GAME_INIT missing rounds[]")
        return self._latest_game_init

    def _phase_for(self, round_id: Any, question_id: Any) -> Optional[str]:
        if isinstance(question_id, int) and question_id in self._question_id_to_phase:
            return self._question_id_to_phase[question_id]
        if isinstance(round_id, int) and round_id in self._round_id_to_phase:
            return self._round_id_to_phase[round_id]
        return None

    def _assert_menu_manifest(self, timeout_sec: float) -> ManifestSnapshot:
        menu = self.wait_for_manifest(
            lambda m: (
                self.manifest_has_instances(m, ["user_panel", "game_screen"])
                and str(self.instance_config(m, "game_screen").get("mode") or "") == "menu"
            ),
            timeout_sec=timeout_sec,
        )
        screen_cfg = self.instance_config(menu.manifest, "game_screen")
        _require(isinstance(screen_cfg.get("games"), list), "Menu manifest missing game_screen.games[]")
        _require(bool(screen_cfg.get("startGameOpId")), "Menu manifest missing game_screen.startGameOpId")
        _require(bool(screen_cfg.get("uiInputOpId")), "Menu manifest missing game_screen.uiInputOpId")
        _require(screen_cfg.get("inputDisabled") is False, "Menu manifest expected game_screen.inputDisabled=false")
        controls = screen_cfg.get("controls") if isinstance(screen_cfg.get("controls"), dict) else {}
        _require(
            all(k in controls for k in ("showPause", "showResume", "showStop", "showReset", "showSkipPhase")),
            "Menu manifest missing game_screen.controls keys",
        )
        _require(
            all(controls.get(k) is False for k in ("showPause", "showResume", "showStop", "showReset", "showSkipPhase")),
            "Menu manifest expected all controls hidden",
        )
        return menu

    def _assert_game_intro_manifest(
        self,
        expected_intro: str,
        timeout_sec: float,
        expected_phase: str = "",
    ) -> ManifestSnapshot:
        game = self.wait_for_manifest(
            lambda m: (
                str(self.instance_config(m, "game_screen").get("mode") or "") == "game"
                and (self.instance_config(m, "game_screen").get("question") or {}).get("text") == expected_intro
                and self.instance_config(m, "game_screen").get("inputDisabled") is True
                and (
                    not expected_phase
                    or str(self.instance_config(m, "game_screen").get("phase") or "").upper()
                    == str(expected_phase).upper()
                )
            ),
            timeout_sec=timeout_sec,
        )
        _require(
            str(self.instance_config(game.manifest, "game_screen").get("mode") or "") == "game",
            "Expected game_screen.mode='game'",
        )
        return game

    def _assert_question_manifest(self, question: Dict[str, Any], timeout_sec: float) -> ManifestSnapshot:
        prompt = str(question.get("prompt") or "")
        _require(bool(prompt.strip()), "QUESTION_PRESENT has empty question.prompt (TTS/UI requires non-empty)")
        image_url = question.get("imageUrl")
        expected_imgs = [rewrite_asset_url(image_url)] if isinstance(image_url, str) and image_url else []
        expected_count = len(question.get("options") or []) if isinstance(question.get("options"), list) else 0
        question_type = str(question.get("questionType") or "").lower()
        allow_synthesized_options = expected_count == 0 and question_type == "speech"

        manifest_q = self.wait_for_manifest(
            lambda m: (
                str(self.instance_config(m, "game_screen").get("mode") or "") == "game"
                and (self.instance_config(m, "game_screen").get("question") or {}).get("text") == prompt
                and (self.instance_config(m, "game_screen").get("question") or {}).get("imgs") == expected_imgs
                and (
                    len(self.instance_config(m, "game_screen").get("options") or []) == expected_count
                    or (
                        allow_synthesized_options
                        and len(self.instance_config(m, "game_screen").get("options") or []) > 0
                    )
                )
            ),
            timeout_sec=timeout_sec,
        )
        _require(
            all(
                isinstance(opt, dict) and all(k in opt for k in ("id", "label"))
                for opt in (self.instance_config(manifest_q.manifest, "game_screen").get("options") or [])
            ),
            "Matching options in manifest are missing required keys",
        )
        return manifest_q

    def _pause_and_resume(self, timeout_sec: float, tag: str) -> None:
        self.publish_ui_action("PAUSE")
        self.wait_for_event(
            lambda e: (
                str(e.get("type") or "").upper() == "GAME_CONTROL"
                and str((e.get("payload") or {}).get("command") or "").upper() == "PAUSE"
            ),
            timeout_sec=timeout_sec,
        )
        paused = self.wait_for_state(lambda s: s.state.upper() == "PAUSED", timeout_sec=timeout_sec)
        paused_manifest = self.wait_for_manifest(
            lambda m: (
                (self.instance_config(m, "game_screen").get("controls") or {}).get("showResume") is True
                and self.instance_config(m, "game_screen").get("inputDisabled") is True
            ),
            timeout_sec=timeout_sec,
        )
        self.log_manifest(f"{tag}_paused", paused)
        paused_controls = self.instance_config(paused_manifest.manifest, "game_screen").get("controls") or {}
        _require(paused_controls.get("showPause") is False, "Expected showPause=False while paused")
        _require(paused_controls.get("showResume") is True, "Expected showResume=True while paused")
        _require(paused_controls.get("showStop") is True, "Expected showStop=True while paused")
        _require(paused_controls.get("showReset") is True, "Expected showReset=True while paused")
        _require(paused_controls.get("showSkipPhase") is True, "Expected showSkipPhase=True while paused")

        self.publish_ui_action("RESUME")
        self.wait_for_event(
            lambda e: (
                str(e.get("type") or "").upper() == "GAME_CONTROL"
                and str((e.get("payload") or {}).get("command") or "").upper() == "RESUME"
            ),
            timeout_sec=timeout_sec,
        )
        resumed = self.wait_for_state(lambda s: s.state.upper() == "GAME", timeout_sec=timeout_sec)
        resumed_manifest = self.wait_for_manifest(
            lambda m: (
                (self.instance_config(m, "game_screen").get("controls") or {}).get("showResume") is False
                and (self.instance_config(m, "game_screen").get("controls") or {}).get("showPause") is True
            ),
            timeout_sec=timeout_sec,
        )
        self.log_manifest(f"{tag}_resumed", resumed)
        resumed_controls = self.instance_config(resumed_manifest.manifest, "game_screen").get("controls") or {}
        _require(resumed_controls.get("showStop") is True, "Expected showStop=True after resume")
        _require(resumed_controls.get("showReset") is True, "Expected showReset=True after resume")
        _require(resumed_controls.get("showSkipPhase") is True, "Expected showSkipPhase=True after resume")

    def _stop_to_menu(self, timeout_sec: float, tag: str) -> DecisionSnapshot:
        self.publish_ui_action("STOP")
        self.wait_for_event(
            lambda e: (
                str(e.get("type") or "").upper() == "GAME_CONTROL"
                and str((e.get("payload") or {}).get("command") or "").upper() in {"STOP", "EXIT"}
            ),
            timeout_sec=timeout_sec,
        )
        idle = self.wait_for_state(lambda s: s.state.upper() == "IDLE", timeout_sec=timeout_sec)
        self._assert_menu_manifest(timeout_sec=timeout_sec)
        self.log_manifest(f"{tag}_menu_after_stop", idle)
        return idle

    def _reset_to_intro(self, timeout_sec: float, tag: str, expected_phase: str) -> DecisionSnapshot:
        self.publish_ui_action("RESET")
        self.wait_for_event(
            lambda e: (
                str(e.get("type") or "").upper() == "GAME_CONTROL"
                and str((e.get("payload") or {}).get("command") or "").upper() in {"RESET", "RESTART"}
            ),
            timeout_sec=timeout_sec,
        )
        intro_state = self.wait_for_state(
            lambda s: (
                s.state.upper() == "GAME"
                and (s.game_state or "").upper() == "PHASE_INTRO"
                and str(s.payload.get("phase") or "").upper() == str(expected_phase or "").upper()
            ),
            timeout_sec=timeout_sec,
        )
        expected_intro = self._expected_intro_text(intro_state.payload)
        self._assert_game_intro_manifest(expected_intro, timeout_sec=timeout_sec, expected_phase=expected_phase)
        self.log_manifest(f"{tag}_reset_intro_{expected_phase}", intro_state)
        return intro_state

    def _skip_to_intro(self, timeout_sec: float, tag: str, expected_phase: str) -> DecisionSnapshot:
        self.publish_ui_action("SKIP_PHASE")
        self.wait_for_event(
            lambda e: (
                str(e.get("type") or "").upper() == "GAME_CONTROL"
                and str((e.get("payload") or {}).get("command") or "").upper() in {"SKIP_PHASE", "SKIP"}
            ),
            timeout_sec=timeout_sec,
        )
        intro_state = self.wait_for_state(
            lambda s: (
                s.state.upper() == "GAME"
                and (s.game_state or "").upper() == "PHASE_INTRO"
                and str(s.payload.get("phase") or "").upper() == str(expected_phase or "").upper()
            ),
            timeout_sec=timeout_sec,
        )
        expected_intro = self._expected_intro_text(intro_state.payload)
        self._assert_game_intro_manifest(expected_intro, timeout_sec=timeout_sec, expected_phase=expected_phase)
        self.log_manifest(f"{tag}_skip_intro_{expected_phase}", intro_state)
        return intro_state

    def _parse_phase_csv(self, raw: str) -> List[str]:
        values = []
        for part in (raw or "").split(","):
            p = part.strip().upper()
            if p:
                values.append(p)
        return values

    def _run_session(self, cfg: RunConfig, timeout_sec: float) -> RunResult:
        self.get_logger().info(
            f"Starting {cfg.name}: game={cfg.game_slug} phases={cfg.phases} "
            f"difficulty={cfg.difficulty} rounds_per_phase={cfg.rounds_per_phase}"
        )
        phase_sequence = [str(p).strip().upper() for p in cfg.phases if str(p).strip()]
        first_phase = phase_sequence[0] if phase_sequence else ""

        # Reset per-run cached mappings.
        self._latest_game_init = None
        self._round_id_to_phase.clear()
        self._question_id_to_phase.clear()

        self.publish_user_selection(cfg.user_id, cfg.user_name)
        self.publish_game_selection(
            slug=cfg.game_slug,
            phases=cfg.phases,
            difficulty=cfg.difficulty,
            rounds_per_phase=cfg.rounds_per_phase,
        )

        game_init = self._wait_for_game_init(timeout_sec=timeout_sec)
        self._record_step(
            f"{cfg.name}_game_init_seen",
            True,
            {
                "phaseSequence": game_init.get("phaseSequence"),
                "difficulty": game_init.get("difficulty"),
                "rounds": len(game_init.get("rounds") or []) if isinstance(game_init.get("rounds"), list) else None,
            },
        )

        phase_intro = self.wait_for_state(
            lambda s: s.state.upper() == "GAME" and (s.game_state or "").upper() == "PHASE_INTRO",
            timeout_sec=timeout_sec,
        )
        expected_intro = self._expected_intro_text(phase_intro.payload)
        expected_phase = str(phase_intro.payload.get("phase") or "")
        self._assert_game_intro_manifest(expected_intro, timeout_sec=timeout_sec, expected_phase=expected_phase)
        self.log_manifest(f"{cfg.name}_phase_intro", phase_intro)
        self._record_step(f"{cfg.name}_phase_intro_manifest_ok", True, {"text": expected_intro})

        phases_seen: Set[str] = set()
        phases_answered_correct: Set[str] = set()
        phases_failed_once: Set[str] = set()
        pause_done = False
        skip_done = False
        reset_done = False
        exited_early = False

        # Drive rounds until completion or explicit exit.
        while True:
            snap = self.wait_for_state(
                lambda s: s.state.upper() == "IDLE"
                or (s.game_state or "").upper() in {"QUESTION_PRESENT", "PHASE_COMPLETE"},
                timeout_sec=timeout_sec,
            )
            if snap.state.upper() == "IDLE":
                break

            game_state_upper = (snap.game_state or "").upper()
            if game_state_upper == "PHASE_COMPLETE":
                phase = str(snap.payload.get("phase") or "")
                expected_text = f"¡Fase {phase} completada!" if phase else "¡Fase completada!"
                # Give the controller a moment to patch the UI to the phase-complete screen
                # before we snapshot the manifest.
                self.wait_for_manifest(
                    lambda m: (
                        str(self.instance_config(m, "game_screen").get("mode") or "") == "game"
                        and (self.instance_config(m, "game_screen").get("question") or {}).get("questionType")
                        == "phase_complete"
                        and (self.instance_config(m, "game_screen").get("question") or {}).get("text") == expected_text
                        and len(self.instance_config(m, "game_screen").get("options") or []) == 0
                    ),
                    timeout_sec=min(10.0, timeout_sec),
                )
                self.log_manifest(f"{cfg.name}_phase_complete", snap)
                self._record_step(f"{cfg.name}_phase_complete_seen", True, {"payload": snap.payload})
                # Auto-advance should return to IDLE; verify menu manifest comes back.
                idle = self.wait_for_state(lambda s: s.state.upper() == "IDLE", timeout_sec=timeout_sec)
                self._assert_menu_manifest(timeout_sec=timeout_sec)
                self.log_manifest(f"{cfg.name}_menu_after_complete", idle)
                break

            # QUESTION_PRESENT
            question = snap.payload.get("question") if isinstance(snap.payload.get("question"), dict) else {}
            _require(bool(question), "Missing question payload in QUESTION_PRESENT")
            qid = question.get("questionId")
            self._assert_question_manifest(question, timeout_sec=timeout_sec)
            self.log_manifest(f"{cfg.name}_question_present_q{qid}", snap)

            wait_input = self.wait_for_state(
                lambda s: s.state.upper() == "IDLE" or (s.game_state or "").upper() == "WAIT_INPUT",
                timeout_sec=timeout_sec,
            )
            if wait_input.state.upper() == "IDLE":
                break

            round_id = wait_input.payload.get("roundId")
            wait_qid = wait_input.payload.get("questionId")
            phase = self._phase_for(round_id, wait_qid if wait_qid is not None else qid)
            if phase:
                phases_seen.add(phase)

            # WAIT_INPUT must enable user interaction.
            wait_input_manifest = self.wait_for_manifest(
                lambda m: (
                    str(self.instance_config(m, "game_screen").get("mode") or "") == "game"
                    and self.instance_config(m, "game_screen").get("inputDisabled") is False
                    and (
                        not phase
                        or str(self.instance_config(m, "game_screen").get("phase") or "").upper() == str(phase).upper()
                    )
                ),
                timeout_sec=timeout_sec,
            )
            self.log_manifest(f"{cfg.name}_wait_input_q{qid}", wait_input)
            _require(
                (self.instance_config(wait_input_manifest.manifest, "game_screen").get("controls") or {}).get("showStop")
                is True,
                "Expected showStop=True in game mode",
            )

            if cfg.pause_once and not pause_done:
                self._pause_and_resume(timeout_sec=timeout_sec, tag=cfg.name)
                # Ensure we are back in WAIT_INPUT after resume.
                wait_input = self.wait_for_state(
                    lambda s: (s.game_state or "").upper() == "WAIT_INPUT" and s.state.upper() == "GAME",
                    timeout_sec=timeout_sec,
                )
                pause_done = True

            if cfg.stop_on_phase and phase == cfg.stop_on_phase:
                self._stop_to_menu(timeout_sec=timeout_sec, tag=cfg.name)
                self._record_step(f"{cfg.name}_stopped_on_phase_{phase}", True)
                exited_early = True
                break

            if cfg.skip_phase_on_phase and not skip_done and phase == cfg.skip_phase_on_phase:
                try:
                    idx = phase_sequence.index(str(phase).upper())
                except ValueError:
                    idx = -1
                next_phase = phase_sequence[idx + 1] if idx >= 0 and idx + 1 < len(phase_sequence) else ""
                _require(bool(next_phase), f"Cannot SKIP_PHASE from {phase}: no next phase in sequence {phase_sequence}")
                self._skip_to_intro(timeout_sec=timeout_sec, tag=cfg.name, expected_phase=next_phase)
                self._record_step(f"{cfg.name}_skipped_to_{next_phase}", True, {"from": phase, "to": next_phase})
                skip_done = True
                continue

            if cfg.reset_on_phase and not reset_done and phase == cfg.reset_on_phase:
                _require(bool(first_phase), "Cannot RESET: empty phases[]")
                self._reset_to_intro(timeout_sec=timeout_sec, tag=cfg.name, expected_phase=first_phase)
                self._record_step(f"{cfg.name}_reset_to_{first_phase}", True, {"from": phase, "to": first_phase})
                reset_done = True
                continue

            correct_value, wrong_value = self._extract_correct_and_wrong(question)
            _require(bool(correct_value), "Could not determine correct answer value")
            _require(bool(wrong_value), "Could not determine wrong answer value")

            # Exercise failure path once per phase (except P6: all options are correct).
            if phase and phase not in phases_failed_once and phase != "P6":
                self.publish_ui_action(wrong_value)
                self._wait_for_user_intent(wrong_value, wait_input.transaction_id, timeout_sec=timeout_sec)
                fail_l1 = self.wait_for_state(
                    lambda s: (s.game_state or "").upper() == "FAIL_L1",
                    timeout_sec=timeout_sec,
                )
                expected_hint = str(fail_l1.payload.get("hint") or "Inténtalo de nuevo.")
                self.wait_for_manifest(
                    lambda m: (self.instance_config(m, "game_screen").get("question") or {}).get("text") == expected_hint,
                    timeout_sec=timeout_sec,
                )
                self.log_manifest(f"{cfg.name}_fail_l1_phase_{phase}", fail_l1)
                phases_failed_once.add(phase)

                # Auto-advance back to WAIT_INPUT.
                wait_input = self.wait_for_state(
                    lambda s: (s.game_state or "").upper() == "WAIT_INPUT", timeout_sec=timeout_sec
                )

            self.publish_ui_action(correct_value)
            self._wait_for_user_intent(correct_value, wait_input.transaction_id, timeout_sec=timeout_sec)
            correct_state = self.wait_for_state(
                lambda s: (s.game_state or "").upper() == "CORRECT",
                timeout_sec=timeout_sec,
            )
            self.wait_for_manifest(
                lambda m: (
                    (self.instance_config(m, "game_screen").get("question") or {}).get("text") == "¡Muy bien!"
                    and len(self.instance_config(m, "game_screen").get("options") or []) == 0
                ),
                timeout_sec=timeout_sec,
            )
            self.log_manifest(f"{cfg.name}_correct_phase_{phase}", correct_state)
            if phase:
                phases_answered_correct.add(phase)

        return RunResult(
            name=cfg.name,
            phases_requested=cfg.phases,
            phases_seen=phases_seen,
            phases_answered_correct=phases_answered_correct,
            phases_failed_once=phases_failed_once,
            exited_early=exited_early,
        )

    # -------------------- main scenario --------------------
    def run(self) -> None:
        timeout = float(os.environ.get("TEST_TIMEOUT", "180"))
        started_at = _now()
        success = False
        error: Optional[str] = None

        try:
            idle = self.wait_for_state(lambda s: s.state.upper() == "IDLE", timeout_sec=timeout)
            self.get_logger().info("decision_making is IDLE")
            self._assert_menu_manifest(timeout_sec=timeout)
            self.log_manifest("startup_menu", idle)
            self._record_step("startup_menu_manifest_ok", True)

            game_slug = os.environ.get("TEST_GAME_SLUG", "colores").strip() or "colores"
            difficulty = os.environ.get("TEST_DIFFICULTY", "basic").strip() or "basic"
            run_a_phases = self._parse_phase_csv(os.environ.get("TEST_RUN_A_PHASES", "P1,P2,P3"))
            run_b_phases = self._parse_phase_csv(os.environ.get("TEST_RUN_B_PHASES", "P3,P4,P5,P6"))
            run_c_phases = self._parse_phase_csv(os.environ.get("TEST_RUN_C_PHASES", "P1,P2,P3"))

            run_a_stop_on = (
                os.environ.get("TEST_RUN_A_STOP_ON_PHASE")
                or os.environ.get("TEST_RUN_A_EXIT_ON_PHASE")
                or "P3"
            ).strip().upper() or "P3"
            run_c_skip_on = (os.environ.get("TEST_RUN_C_SKIP_ON_PHASE") or "P1").strip().upper() or "P1"
            run_c_reset_on = (os.environ.get("TEST_RUN_C_RESET_ON_PHASE") or "P2").strip().upper() or "P2"

            runs = [
                RunConfig(
                    name="run_a",
                    user_id=1,
                    user_name="Pepe",
                    game_slug=game_slug,
                    phases=run_a_phases,
                    difficulty=difficulty,
                    rounds_per_phase=1,
                    stop_on_phase=run_a_stop_on,
                    pause_once=True,
                ),
                RunConfig(
                    name="run_b",
                    user_id=2,
                    user_name="María",
                    game_slug=game_slug,
                    phases=run_b_phases,
                    difficulty=difficulty,
                    rounds_per_phase=1,
                    stop_on_phase=None,
                    pause_once=False,
                ),
                RunConfig(
                    name="run_c_controls",
                    user_id=3,
                    user_name="Juan",
                    game_slug=game_slug,
                    phases=run_c_phases,
                    difficulty=difficulty,
                    rounds_per_phase=1,
                    stop_on_phase=None,
                    skip_phase_on_phase=run_c_skip_on,
                    reset_on_phase=run_c_reset_on,
                    pause_once=False,
                ),
            ]

            for cfg in runs:
                result = self._run_session(cfg, timeout_sec=timeout)
                self._run_results.append(result)
                self._record_step(f"{cfg.name}_finished", True)

            success = True
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(error)
            self.get_logger().error(traceback.format_exc())
            raise
        finally:
            self._write_report(success, error, started_at)


def main() -> None:
    rclpy.init()
    node = IntegrationHarness()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
