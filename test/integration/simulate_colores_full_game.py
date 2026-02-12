#!/usr/bin/env python3
"""Complete Colores Game Simulator - Captures all manifest states for UI development.

This script simulates a full colores game session with all 6 phases (P1, P2, P3, P4_YESNO, P6, P7)
using basic difficulty (2 colors: red, blue) and captures EVERY manifest state transition.

Output Structure:
    /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/
    ├── README.md
    ├── index.md
    ├── 00_initial_menu/
    │   ├── manifest.json
    │   ├── description.md
    │   └── decision_state.json
    ├── 01_P1_phase_intro/
    │   ├── manifest.json
    │   ├── description.md
    │   └── decision_state.json
    ├── 02_P1_question_1_present/
    │   └── ...
    └── ... (all states in all phases)

Usage:
    docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit
    # Or run directly:
    python3 test/integration/simulate_colores_full_game.py
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

try:
    from generic_ui_interfaces.srv import GetManifest, UpdateManifest
    HAS_MANIFEST_SERVICES = True
except ImportError:
    GetManifest = None  # type: ignore
    UpdateManifest = None  # type: ignore
    HAS_MANIFEST_SERVICES = False

try:
    from hri_actions_msgs.msg import Intent
    HAS_INTENT = True
except ImportError:
    Intent = None  # type: ignore
    HAS_INTENT = False


@dataclass
class ManifestState:
    """Represents a captured manifest state."""
    sequence_num: int
    name: str
    description: str
    manifest: Dict[str, Any]
    patches: Optional[List[Dict[str, Any]]]
    decision_state: Dict[str, Any]
    timestamp: float
    phase: str = ""
    question_num: int = 0
    interaction_available: bool = False
    expected_user_actions: List[str] = field(default_factory=list)
    next_transitions: List[str] = field(default_factory=list)


class ColoresGameSimulator(Node):
    """Simulates a complete colores game and captures all manifest states."""

    def __init__(self) -> None:
        super().__init__("colores_full_game_simulator")

        # Output configuration
        self._output_dir = Path(os.environ.get(
            "MANIFEST_OUTPUT_DIR",
            "/home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game"
        ))
        self._output_dir.mkdir(parents=True, exist_ok=True)

        # ROS subscriptions
        decision_state_qos = QoSProfile(depth=1)
        decision_state_qos.reliability = QoSReliabilityPolicy.RELIABLE
        decision_state_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._state_sub = self.create_subscription(
            String, "/decision/state", self._on_decision_state, decision_state_qos
        )
        self._events_sub = self.create_subscription(
            String, "/decision/events", self._on_decision_event, 10
        )
        self._ui_input_sub = self.create_subscription(
            String, "/ui/input", self._on_ui_input, 10
        )

        # ROS publishers
        self._user_pub = self.create_publisher(String, "/game/user_selector", 10)
        self._game_pub = self.create_publisher(String, "/game/game_selector", 10)
        self._ui_input_pub = self.create_publisher(String, "/ui/input", 10)
        self._decision_events_pub = self.create_publisher(String, "/decision/events", 10)

        if HAS_INTENT:
            self._intent_pub = self.create_publisher(Intent, "/intents", 10)
        else:
            self._intent_pub = None

        # ROS clients
        if HAS_MANIFEST_SERVICES:
            self._get_manifest_cli = self.create_client(GetManifest, "/generic_ui/get_manifest")
            self._update_manifest_sub = self.create_subscription(
                String, "/generic_ui/update_manifest_response", self._on_manifest_update, 10
            )
        else:
            self._get_manifest_cli = None
            self._update_manifest_sub = None

        # State tracking
        self._latest_decision_state: Optional[Dict[str, Any]] = None
        self._latest_event: Optional[Dict[str, Any]] = None
        self._latest_manifest: Optional[Dict[str, Any]] = None
        self._latest_patches: Optional[List[Dict[str, Any]]] = None
        self._captured_states: List[ManifestState] = []
        self._sequence_num = 0

        # Game tracking
        self._current_phase = ""
        self._question_count_per_phase: Dict[str, int] = {}
        self._phases_completed: Set[str] = set()

        self.get_logger().info(f"Output directory: {self._output_dir}")

    # -------------------- ROS Callbacks --------------------

    def _on_decision_state(self, msg: String) -> None:
        """Track decision_making state changes."""
        try:
            state = json.loads(msg.data)
            self._latest_decision_state = state
            self.get_logger().debug(f"Decision state: {state.get('state')} / {state.get('gameState')}")
        except json.JSONDecodeError:
            pass

    def _on_decision_event(self, msg: String) -> None:
        """Track decision_making events."""
        try:
            event = json.loads(msg.data)
            self._latest_event = event
            self.get_logger().debug(f"Decision event: {event.get('type')}")
        except json.JSONDecodeError:
            pass

    def _on_ui_input(self, msg: String) -> None:
        """Bridge UI input to intents."""
        if not HAS_INTENT or self._intent_pub is None:
            return

        intent = Intent()
        intent.intent = Intent.RAW_USER_INPUT
        intent.modality = Intent.MODALITY_TOUCHSCREEN
        intent.source = Intent.REMOTE_SUPERVISOR
        intent.priority = 128
        intent.confidence = 1.0
        intent.data = json.dumps({"input": msg.data}, ensure_ascii=False)
        self._intent_pub.publish(intent)

    def _on_manifest_update(self, msg: String) -> None:
        """Track manifest patches as they're applied."""
        try:
            update = json.loads(msg.data)
            if update.get("operation") == "patch":
                patches = json.loads(update.get("json_payload", "[]"))
                self._latest_patches = patches
                self.get_logger().debug(f"Captured {len(patches)} patches")
        except json.JSONDecodeError:
            pass

    # -------------------- Utility Methods --------------------

    def _now(self) -> float:
        return time.monotonic()

    def spin_until(self, predicate, timeout_sec: float) -> bool:
        """Spin until predicate returns True or timeout."""
        end = self._now() + timeout_sec
        while self._now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_for_decision_state(self, predicate, timeout_sec: float) -> Dict[str, Any]:
        """Wait for a decision state matching predicate."""
        ok = self.spin_until(
            lambda: self._latest_decision_state and predicate(self._latest_decision_state),
            timeout_sec
        )
        if not ok:
            raise TimeoutError(f"Timeout waiting for decision state ({timeout_sec}s)")
        return self._latest_decision_state

    def wait_for_event(self, event_type: str, timeout_sec: float) -> Dict[str, Any]:
        """Wait for a specific decision event type."""
        start_time = self._now()
        matched = None

        def check():
            nonlocal matched
            if self._latest_event and self._latest_event.get("type") == event_type:
                matched = self._latest_event
                return True
            return False

        ok = self.spin_until(check, timeout_sec)
        if not ok:
            raise TimeoutError(f"Timeout waiting for event {event_type} ({timeout_sec}s)")
        return matched

    def get_manifest(self, timeout_sec: float = 5.0) -> Dict[str, Any]:
        """Get current manifest from generic_ui."""
        if not HAS_MANIFEST_SERVICES or not self._get_manifest_cli:
            raise RuntimeError("GetManifest service unavailable")

        if not self._get_manifest_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("GetManifest service not available")

        request = GetManifest.Request()
        future = self._get_manifest_cli.call_async(request)

        ok = self.spin_until(lambda: future.done(), timeout_sec)
        if not ok:
            raise TimeoutError("GetManifest request timed out")

        response = future.result()
        manifest = json.loads(response.manifest_json)
        self._latest_manifest = manifest
        return manifest

    def publish_user_selection(self, user_id: int, user_name: str) -> None:
        """Publish user selection."""
        payload = {
            "userName": user_name,
            "user": {"id": str(user_id), "name": user_name}
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._user_pub.publish(msg)
        self.get_logger().info(f"Published user selection: {user_name}")

    def publish_game_selection(
        self,
        slug: str,
        phases: List[str],
        difficulty: str = "basic",
        rounds_per_phase: int = 1
    ) -> None:
        """Publish game selection."""
        payload = {
            "game": {"slug": slug},
            "level": {"id": 1, "name": "Nivel 1"},
            "phases": phases,
            "difficulty": difficulty,
            "roundsPerPhase": rounds_per_phase
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._game_pub.publish(msg)
        self.get_logger().info(f"Published game selection: {slug}, phases={phases}, difficulty={difficulty}")

    def publish_ui_action(self, label: str) -> None:
        """Publish UI action."""
        payload = {"label": label}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._ui_input_pub.publish(msg)
        self.get_logger().info(f"Published UI action: {label}")

    # -------------------- Manifest Capture --------------------

    def capture_state(
        self,
        name: str,
        description: str,
        phase: str = "",
        question_num: int = 0,
        interaction_available: bool = False,
        expected_user_actions: Optional[List[str]] = None,
        next_transitions: Optional[List[str]] = None
    ) -> ManifestState:
        """Capture current manifest state."""
        manifest = self.get_manifest()

        state = ManifestState(
            sequence_num=self._sequence_num,
            name=name,
            description=description,
            manifest=manifest,
            patches=self._latest_patches.copy() if self._latest_patches else None,
            decision_state=self._latest_decision_state.copy() if self._latest_decision_state else {},
            timestamp=self._now(),
            phase=phase,
            question_num=question_num,
            interaction_available=interaction_available,
            expected_user_actions=expected_user_actions or [],
            next_transitions=next_transitions or []
        )

        self._captured_states.append(state)
        self._sequence_num += 1
        self._latest_patches = None  # Reset for next capture

        self.get_logger().info(f"Captured state #{self._sequence_num}: {name}")
        return state

    def save_all_states(self) -> None:
        """Save all captured states to disk."""
        self.get_logger().info(f"Saving {len(self._captured_states)} states to {self._output_dir}")

        # Create folder for each state
        for state in self._captured_states:
            folder_name = f"{state.sequence_num:02d}_{state.name}"
            state_dir = self._output_dir / folder_name
            state_dir.mkdir(exist_ok=True)

            # Save manifest.json
            with open(state_dir / "manifest.json", "w", encoding="utf-8") as f:
                json.dump(state.manifest, f, indent=2, ensure_ascii=False)

            # Save patches.json if available
            if state.patches:
                with open(state_dir / "patches.json", "w", encoding="utf-8") as f:
                    json.dump(state.patches, f, indent=2, ensure_ascii=False)

            # Save decision_state.json
            with open(state_dir / "decision_state.json", "w", encoding="utf-8") as f:
                json.dump(state.decision_state, f, indent=2, ensure_ascii=False)

            # Save description.md
            self._write_description_md(state_dir, state)

        # Create README.md
        self._write_readme()

        # Create index.md
        self._write_index()

        self.get_logger().info("All states saved successfully")

    def _write_description_md(self, state_dir: Path, state: ManifestState) -> None:
        """Write description.md for a state."""
        lines = [
            f"# {state.name}",
            "",
            f"**Sequence:** {state.sequence_num}",
            f"**Phase:** {state.phase or 'N/A'}",
            f"**Question:** {state.question_num if state.question_num > 0 else 'N/A'}",
            f"**Timestamp:** {state.timestamp:.2f}s",
            "",
            "## Description",
            "",
            state.description,
            "",
        ]

        if state.interaction_available:
            lines.extend([
                "## User Interactions Available",
                "",
                "The user can interact with the UI in this state.",
                "",
            ])

            if state.expected_user_actions:
                lines.extend([
                    "### Expected User Actions",
                    "",
                ])
                for action in state.expected_user_actions:
                    lines.append(f"- {action}")
                lines.append("")
        else:
            lines.extend([
                "## User Interactions",
                "",
                "**Input Disabled** - The UI is in a non-interactive state (auto-advance).",
                "",
            ])

        if state.next_transitions:
            lines.extend([
                "## Next State Transitions",
                "",
            ])
            for transition in state.next_transitions:
                lines.append(f"- {transition}")
            lines.append("")

        # Add decision state info
        ds = state.decision_state
        if ds:
            lines.extend([
                "## Decision State",
                "",
                f"- **State:** {ds.get('state', 'N/A')}",
                f"- **Game State:** {ds.get('gameState', 'N/A')}",
                f"- **Transaction ID:** {ds.get('transactionId', 'N/A')}",
                "",
            ])

        # Add manifest highlights
        game_screen = self._extract_game_screen_config(state.manifest)
        if game_screen:
            lines.extend([
                "## Manifest Highlights",
                "",
                f"- **Mode:** {game_screen.get('mode', 'N/A')}",
                f"- **Phase:** {game_screen.get('phase', 'N/A')}",
                f"- **Input Disabled:** {game_screen.get('inputDisabled', False)}",
                "",
            ])

            question = game_screen.get('question', {})
            if question:
                lines.extend([
                    "### Question",
                    "",
                    f"- **Text:** {question.get('text', 'N/A')}",
                    f"- **Type:** {question.get('questionType', 'N/A')}",
                    "",
                ])

            options = game_screen.get('options', [])
            if options:
                lines.extend([
                    "### Options",
                    "",
                ])
                for opt in options:
                    correct = " [CORRECT]" if opt.get('correct') else ""
                    lines.append(f"- **{opt.get('id', 'N/A')}:** {opt.get('label', 'N/A')}{correct}")
                lines.append("")

        with open(state_dir / "description.md", "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    def _extract_game_screen_config(self, manifest: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Extract game_screen config from manifest."""
        instances = manifest.get("instances", [])
        for inst in instances:
            if inst.get("id") == "game_screen":
                return inst.get("config", {})
        return None

    def _write_readme(self) -> None:
        """Write README.md explaining the structure."""
        content = """# Colores Full Game - Manifest States

This directory contains captured manifest states from a complete colores game session.

## Game Configuration

- **Game:** colores
- **Phases:** P1, P2, P3, P4_YESNO, P6, P7 (all 6 phases)
- **Difficulty:** basic (2 colors: red, blue)
- **Rounds per Phase:** 1

## Purpose

These manifest snapshots are designed for UI developers to:

1. Understand the complete game flow and state transitions
2. See the exact manifest structure at each point in the game
3. Build and test UI components with real game data
4. Verify UI behavior matches expected game states

## Directory Structure

Each numbered directory represents a captured state:

```
00_initial_menu/              - Starting menu state
01_P1_phase_intro/            - Phase 1 introduction
02_P1_question_1_present/     - Question being shown
03_P1_question_1_wait_input/  - Waiting for user answer
04_P1_question_1_correct/     - Correct answer feedback
... (continues through all phases)
```

### Files in Each State Directory

- **manifest.json** - Complete manifest at this state
- **patches.json** - JSON patches applied to reach this state (if available)
- **decision_state.json** - decision_making FSM state
- **description.md** - Human-readable description and documentation

## How to Use

1. **Browse sequentially** to understand game flow
2. **Check manifest.json** to see exact UI configuration
3. **Read description.md** for context and expected behavior
4. **Use patches.json** to understand incremental changes

## Phase Descriptions

### P1 - Matching/Association
"Une los colores iguales" - Match identical colors

### P2 - Voice/Repetition
"Repite el nombre del color" - Repeat the color name

### P3 - Discrimination
"Señala el color correcto" - Point to the correct color

### P4_YESNO - Yes/No Questions
"Responde sí o no a las preguntas" - Answer yes or no questions

### P6 - Child Asks
"Dime qué color quieres señalar" - Tell which color you want to point to

### P7 - Two-Option Choice
"¿Cuál es el color correcto?" - Which is the correct color?

## Colors Used

- **red** - Primary color (difficulty: null - always available)
- **blue** - Primary color (difficulty: 1 - available in basic)

## Related Documentation

- See `index.md` for a complete state listing
- See main game_controller docs for architecture details
- See UI_integration.md for UI integration patterns

---

Generated by: simulate_colores_full_game.py
"""
        with open(self._output_dir / "README.md", "w", encoding="utf-8") as f:
            f.write(content)

    def _write_index(self) -> None:
        """Write index.md with complete state listing."""
        lines = [
            "# Colores Full Game - State Index",
            "",
            f"Total States Captured: {len(self._captured_states)}",
            "",
            "| # | Name | Phase | Question | Interactive | Description |",
            "|---|------|-------|----------|-------------|-------------|",
        ]

        for state in self._captured_states:
            phase_str = state.phase or "-"
            q_str = str(state.question_num) if state.question_num > 0 else "-"
            interactive = "Yes" if state.interaction_available else "No"
            desc_short = state.description.split("\n")[0][:50]

            lines.append(
                f"| {state.sequence_num:02d} | {state.name} | {phase_str} | {q_str} | "
                f"{interactive} | {desc_short} |"
            )

        lines.extend([
            "",
            "## Phase Coverage",
            "",
        ])

        phases = ["P1", "P2", "P3", "P4", "P5", "P6"]
        for phase in phases:
            phase_states = [s for s in self._captured_states if s.phase == phase]
            lines.append(f"- **{phase}:** {len(phase_states)} states")

        lines.extend([
            "",
            "## State Type Breakdown",
            "",
        ])

        state_types = {}
        for state in self._captured_states:
            state_type = state.name.split("_")[-1]
            state_types[state_type] = state_types.get(state_type, 0) + 1

        for state_type, count in sorted(state_types.items()):
            lines.append(f"- **{state_type}:** {count}")

        with open(self._output_dir / "index.md", "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    # -------------------- Game Simulation --------------------

    def simulate_game(self) -> None:
        """Run complete game simulation."""
        timeout = 180.0

        try:
            self.get_logger().info("=== Starting Colores Full Game Simulation ===")

            # Wait for initial IDLE state
            self.wait_for_decision_state(lambda s: s.get("state") == "IDLE", timeout)
            self.get_logger().info("System ready - decision_making is IDLE")

            # Capture initial menu state
            time.sleep(0.5)  # Let manifest stabilize
            self.capture_state(
                "initial_menu",
                "Initial menu state before game selection. Shows user selection panel and game menu.",
                interaction_available=True,
                expected_user_actions=["Select user", "Select game", "Configure game settings"],
                next_transitions=["User selection", "Game selection", "Phase intro"]
            )

            # Start game
            self.publish_user_selection(1, "TestUser")
            time.sleep(0.2)

            phase_env = os.environ.get("COLORES_SIM_PHASES", "P1,P2,P3,P4,P5,P6")
            phases = [phase.strip() for phase in phase_env.split(",") if phase.strip()]
            if not phases:
                phases = ["P1", "P2", "P3"]
            self.publish_game_selection("colores", phases, difficulty="basic", rounds_per_phase=1)

            # Wait for GAME_INIT event
            self.wait_for_event("GAME_INIT", timeout)
            self.get_logger().info("GAME_INIT received")

            # Simulate all phases
            for phase in phases:
                self.get_logger().info(f"=== Simulating Phase: {phase} ===")
                self._simulate_phase(phase, timeout)

            self.get_logger().info("=== Game Simulation Complete ===")

        except Exception as e:
            self.get_logger().error(f"Simulation failed: {e}")
            raise
        finally:
            # Save all captured states
            self.save_all_states()

    def _simulate_phase(self, phase: str, timeout: float) -> None:
        """Simulate a single phase."""
        self._current_phase = phase
        self._question_count_per_phase[phase] = 0

        # Wait until this phase becomes active. Depending on timing, we may
        # join on PHASE_INTRO or on a later state if auto-advance already fired.
        entry_state = self.wait_for_decision_state(
            lambda s: (
                s.get("payload", {}).get("phase") == phase
                and s.get("gameState") in {
                    "PHASE_INTRO",
                    "QUESTION_PRESENT",
                    "WAIT_INPUT",
                    "FAIL_L1",
                    "FAIL_L2",
                    "CORRECT",
                    "PHASE_COMPLETE",
                }
            ),
            timeout
        )

        if entry_state.get("gameState") == "PHASE_INTRO":
            time.sleep(0.3)  # Let manifest update
            intro_text = self._get_phase_intro_text(phase)
            self.capture_state(
                f"{phase}_phase_intro",
                f"Phase {phase} introduction screen.\n\n{intro_text}",
                phase=phase,
                interaction_available=False,
                next_transitions=["Auto-advance to first question"]
            )

        # Process questions in this phase
        question_num = 0
        while True:
            # Check if we've moved to next phase or completed
            current_state = self._latest_decision_state
            if not current_state:
                break

            state = current_state.get("state", "")
            game_state = current_state.get("gameState", "")
            current_phase = current_state.get("payload", {}).get("phase", "")

            # Check for phase complete
            if game_state == "PHASE_COMPLETE" and current_phase == phase:
                time.sleep(0.3)
                self.capture_state(
                    f"{phase}_phase_complete",
                    f"Phase {phase} completed successfully. Shows completion message.",
                    phase=phase,
                    interaction_available=False,
                    next_transitions=["Auto-advance to menu or next phase intro"]
                )
                break

            # Check if moved to different phase
            if current_phase and current_phase != phase:
                break

            # Check for IDLE (end of game)
            if state == "IDLE":
                break

            question_present_seen = False
            question_state = current_state

            if game_state == "QUESTION_PRESENT":
                question_present_seen = True
            elif game_state != "WAIT_INPUT":
                # Wait for question presentation in this phase.
                try:
                    question_state = self.wait_for_decision_state(
                        lambda s: (
                            s.get("gameState") == "QUESTION_PRESENT"
                            and s.get("payload", {}).get("phase") == phase
                        ),
                        timeout_sec=5.0
                    )
                    question_present_seen = True
                except TimeoutError:
                    # No more questions in this phase.
                    break

            question_num += 1
            self._question_count_per_phase[phase] = question_num
            time.sleep(0.3)

            question = question_state.get("payload", {}).get("question", {})
            prompt = question.get("prompt", "")

            if question_present_seen:
                self.capture_state(
                    f"{phase}_question_{question_num}_present",
                    f"Question {question_num} in phase {phase}.\n\nPrompt: {prompt}\n\nQuestion is being presented to the user.",
                    phase=phase,
                    question_num=question_num,
                    interaction_available=False,
                    next_transitions=["Auto-advance to WAIT_INPUT"]
                )

                # Wait for WAIT_INPUT in this phase.
                self.wait_for_decision_state(
                    lambda s: (
                        s.get("gameState") == "WAIT_INPUT"
                        and s.get("payload", {}).get("phase") == phase
                    ),
                    timeout
                )
                time.sleep(0.3)
            else:
                # We already joined on WAIT_INPUT and missed QUESTION_PRESENT.
                time.sleep(0.1)

            correct_answer, wrong_answer, options = self._extract_answers(question)

            expected_actions = []
            if options:
                expected_actions = [f"Select option: {opt['label']}" for opt in options]
            else:
                expected_actions = [f"Provide answer: {correct_answer}"]

            self.capture_state(
                f"{phase}_question_{question_num}_wait_input",
                f"Waiting for user input on question {question_num}.\n\nThe UI is interactive and ready to receive user's answer.",
                phase=phase,
                question_num=question_num,
                interaction_available=True,
                expected_user_actions=expected_actions,
                next_transitions=["User selects correct answer -> CORRECT", "User selects wrong answer -> FAIL_L1"]
            )

            # First submit wrong answer (to test failure path)
            if wrong_answer and phase != "P6":  # P6 doesn't have wrong answers
                self.publish_ui_action(wrong_answer)
                time.sleep(0.2)

                # Wait for FAIL_L1
                self.wait_for_decision_state(
                    lambda s: (
                        s.get("gameState") == "FAIL_L1"
                        and s.get("payload", {}).get("phase") == phase
                    ),
                    timeout
                )
                time.sleep(0.3)

                self.capture_state(
                    f"{phase}_question_{question_num}_fail_l1",
                    f"Incorrect answer feedback for question {question_num}.\n\nShows hint and encourages retry.",
                    phase=phase,
                    question_num=question_num,
                    interaction_available=False,
                    next_transitions=["Auto-advance back to WAIT_INPUT"]
                )

                # Wait to return to WAIT_INPUT
                self.wait_for_decision_state(
                    lambda s: (
                        s.get("gameState") == "WAIT_INPUT"
                        and s.get("payload", {}).get("phase") == phase
                    ),
                    timeout
                )
                time.sleep(0.3)

                self.capture_state(
                    f"{phase}_question_{question_num}_wait_input_retry",
                    f"Ready for retry after incorrect answer.\n\nUser can try again.",
                    phase=phase,
                    question_num=question_num,
                    interaction_available=True,
                    expected_user_actions=expected_actions,
                    next_transitions=["User selects correct answer -> CORRECT"]
                )

            # Submit correct answer
            self.publish_ui_action(correct_answer)
            time.sleep(0.2)

            # Wait for CORRECT
            self.wait_for_decision_state(
                lambda s: (
                    s.get("gameState") == "CORRECT"
                    and s.get("payload", {}).get("phase") == phase
                ),
                timeout
            )
            time.sleep(0.3)

            self.capture_state(
                f"{phase}_question_{question_num}_correct",
                f"Correct answer feedback for question {question_num}.\n\nShows positive feedback and celebration.",
                phase=phase,
                question_num=question_num,
                interaction_available=False,
                next_transitions=["Auto-advance to next question or phase complete"]
            )

            # Short delay before next question
            time.sleep(0.5)

    def _get_phase_intro_text(self, phase: str) -> str:
        """Get phase introduction text."""
        intros = {
            "P1": "Une los colores iguales.",
            "P2": "Repite el nombre del color.",
            "P3": "Señala el color correcto.",
            "P4": "Responde sí o no a las preguntas.",
            "P5": "Dime qué color quieres señalar.",
            "P6": "¿Cuál es el color correcto?"
        }
        return intros.get(phase, "")

    def _extract_answers(self, question: Dict[str, Any]) -> tuple:
        """Extract correct and wrong answers from question."""
        options = question.get("options", [])
        correct = None
        wrong = None

        if options:
            for opt in options:
                if opt.get("correct"):
                    correct = opt.get("id", opt.get("label"))
                elif wrong is None:
                    wrong = opt.get("id", opt.get("label"))
        else:
            # Voice phases
            correct = question.get("answer", "")
            wrong = "__wrong__"

        return correct, wrong, options


def main() -> None:
    """Main entry point."""
    rclpy.init()
    simulator = ColoresGameSimulator()
    exit_code = 0

    try:
        simulator.simulate_game()
    except KeyboardInterrupt:
        simulator.get_logger().info("Simulation interrupted by user")
    except Exception as e:
        simulator.get_logger().error(f"Simulation failed: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
