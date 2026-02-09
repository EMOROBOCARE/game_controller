"""Game Controller Node.

Main ROS2 node that orchestrates the Colors (Colores) game by:
1. Loading game content and building GAME_INIT payloads
2. Sending UI manifests to generic_ui
3. Translating /intents to decision_making events
4. Scheduling ON_COMPLETE events for auto-advancing states
5. Patching manifest based on game state changes
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int16
from hri_actions_msgs.msg import Intent

from .auto_advance import AutoAdvanceConfig, AutoAdvanceScheduler
from .decision_events import (
    build_on_complete_event,
    event_to_json,
    parse_decision_state,
)
from .input_translation import InputTranslator, parse_input_json
from .speech_gate import SpeechGate, SpeechRequest
from .tts_client import ExpressiveTtsClient
from .content.loaders import load_game_content
from .content.loaders import get_all_games_metadata
from .content.builder import build_game_init_payload
from .ui.manifest_builder import (
    DEFAULT_GAME_SCREEN_INDEX,
    GAME_SCREEN_INSTANCE_ID,
    build_initial_manifest,
    build_state_based_patches,
    build_game_screen_controls_patch,
    build_game_screen_input_disabled_patch,
    build_game_screen_mode_patch,
    build_game_screen_options_patch,
    build_game_screen_phase_patch,
    build_game_screen_question_patch,
    build_game_screen_state_patch,
    get_instance_index,
)
from .ui.manifest_client import ManifestClient


class GameControllerNode(Node):
    """ROS2 node for game orchestration.
    
    Subscribes to:
        - /decision/state: Track game state from decision_making
        - /intents: User input and control commands (via communication_hub)
        - /game/game_selector: Game selection
        - /game/user_selector: User selection
        
    Publishes to:
        - /decision/events: Events for decision_making FSM
        - /game/current_user: Current active user
        
    Services (client):
        - /generic_ui/update_manifest: Update UI manifest
    """
    
    def __init__(self) -> None:
        super().__init__("game_controller")
        
        # Declare parameters
        self._declare_parameters()
        
        # Callback group for async operations
        self._cb_group = ReentrantCallbackGroup()
        
        # State tracking
        self._active_session_id: Optional[int] = None
        self._latest_transaction_id: Optional[int] = 0
        self._current_game_state: Optional[str] = None
        self._current_question_cache: Dict[int, Dict[str, Any]] = {}
        self._current_round_id: Optional[int] = None
        self._current_phase: Optional[str] = None
        self._current_difficulty: str = "basic"
        self._session_counter = 0
        
        # Selected game/user
        self._selected_game: Optional[str] = None
        self._selected_user: Optional[int] = None
        self._selected_phases: Optional[List[str]] = None
        self._selected_difficulty_override: Optional[str] = None
        self._selected_rounds_per_phase_override: Optional[int] = None
        self._selected_question_idx: Optional[int] = None
        # Gate session starts so user selection alone doesn't auto-start a stale game.
        self._start_requested: bool = False

        # UI screen state
        self._ui_screen: str = "menu"
        self._ui_paused: bool = False
        self._game_screen_index: int = DEFAULT_GAME_SCREEN_INDEX
        
        # Loaded game content
        self._game_content_cache: Dict[str, Dict[str, Any]] = {}
        self._games_metadata: List[Dict[str, Any]] = get_all_games_metadata()

        # Helpers to infer phase even when decision_making payloads omit it.
        self._round_id_to_phase: Dict[int, str] = {}
        self._question_id_to_phase: Dict[int, str] = {}
        
        # UI input translator
        self._ui_translator = InputTranslator()
        
        # Auto-advance scheduler
        auto_config = AutoAdvanceConfig.from_dict(
            self._get_param_dict("auto_advance")
        )
        self._auto_scheduler = AutoAdvanceScheduler(
            auto_config,
            self._on_auto_advance,
            logger=self.get_logger(),
        )

        # Expressive TTS: in QUESTION_PRESENT, advance only after expressive_say completes.
        self._tts_enabled = bool(self.get_parameter("tts.enabled").value)
        self._tts_language = str(self.get_parameter("tts.language").value or "es")
        self._tts_client = None
        self._speech_gate = None
        if self._tts_enabled:
            self._tts_client = ExpressiveTtsClient(
                self,
                action_name=self.get_parameter("tts.action_server").value,
                server_wait_timeout_sec=self.get_parameter("tts.server_wait_timeout_sec").value,
                callback_group=self._cb_group,
                logger=self.get_logger(),
            )

            def _on_speech_complete(tx_id: int) -> None:
                # Avoid emitting stale ON_COMPLETE events if the system already moved on.
                if int(tx_id) != int(self._latest_transaction_id or 0):
                    self.get_logger().debug(
                        f"Ignoring speech completion for stale tx={tx_id} "
                        f"(current tx={self._latest_transaction_id})"
                    )
                    return
                self._on_auto_advance(int(tx_id))

            self._speech_gate = SpeechGate(
                speak=self._tts_client.speak,
                on_complete=_on_speech_complete,
                logger=self.get_logger(),
            )
        
        # Setup publishers
        self._setup_publishers()
        
        # Setup subscribers
        self._setup_subscribers()
        
        # Setup manifest client
        self._manifest_client = ManifestClient(
            self,
            service_name=self.get_parameter("generic_ui.update_manifest_service").value,
            timeout_sec=self.get_parameter("generic_ui.manifest_timeout_sec").value,
            callback_group=self._cb_group,
        )
        
        # Send initial manifest after short delay
        self.create_timer(
            2.0,
            self._send_initial_manifest,
            callback_group=self._cb_group,
        )
        
        self.get_logger().info("Game Controller node initialized")
    
    def _declare_parameters(self) -> None:
        """Declare ROS parameters."""
        # Topic parameters
        self.declare_parameter("topics.decision_state", "/decision/state")
        self.declare_parameter("topics.intents", "/intents")
        self.declare_parameter("topics.game_selector", "/game/game_selector")
        self.declare_parameter("topics.user_selector", "/game/user_selector")
        self.declare_parameter("topics.decision_events", "/decision/events")
        self.declare_parameter("topics.current_user", "/game/current_user")
        
        # Generic UI parameters
        self.declare_parameter("generic_ui.update_manifest_service", "/generic_ui/update_manifest")
        self.declare_parameter("generic_ui.manifest_timeout_sec", 5.0)
        
        # Auto-advance timeouts
        self.declare_parameter("auto_advance.phase_intro", 2.0)
        self.declare_parameter("auto_advance.round_setup", 0.05)
        self.declare_parameter("auto_advance.question_present", 0.05)
        self.declare_parameter("auto_advance.fail_l1", 2.0)
        self.declare_parameter("auto_advance.fail_l2", 2.0)
        self.declare_parameter("auto_advance.correct", 0.6)
        self.declare_parameter("auto_advance.phase_complete", 0.3)

        # Expressive TTS (EmorobCare communication_hub action interface).
        self.declare_parameter("tts.enabled", True)
        self.declare_parameter("tts.action_server", "/expressive_say")
        self.declare_parameter("tts.language", "es")
        self.declare_parameter("tts.server_wait_timeout_sec", 0.2)
        
        # Game defaults
        self.declare_parameter("game_defaults.difficulty", "basic")
        self.declare_parameter("game_defaults.rounds_per_phase", 2)
        self.declare_parameter("game_defaults.phases", ["P1", "P2", "P3"])
    
    def _get_param_dict(self, prefix: str) -> Dict[str, Any]:
        """Get parameters as a dict by prefix."""
        result: Dict[str, Any] = {}
        prefix_dot = f"{prefix}."
        for name in self._parameters.keys():
            if name.startswith(prefix_dot):
                key = name[len(prefix_dot):]
                result[key] = self.get_parameter(name).value
        return result
    
    def _setup_publishers(self) -> None:
        """Setup ROS publishers."""
        self._events_pub = self.create_publisher(
            String,
            self.get_parameter("topics.decision_events").value,
            10,
        )
        
        self._current_user_pub = self.create_publisher(
            Int16,
            self.get_parameter("topics.current_user").value,
            10,
        )
    
    def _setup_subscribers(self) -> None:
        """Setup ROS subscribers."""
        # Decision state subscriber
        self._state_sub = self.create_subscription(
            String,
            self.get_parameter("topics.decision_state").value,
            self._on_decision_state,
            10,
        )
        
        # Intents subscriber
        self._intents_sub = self.create_subscription(
            Intent,
            self.get_parameter("topics.intents").value,
            self._on_intent,
            10,
        )
        
        # Game selector subscriber
        self._game_selector_sub = self.create_subscription(
            String,
            self.get_parameter("topics.game_selector").value,
            self._on_game_selected,
            10,
        )
        
        # User selector subscriber
        self._user_selector_sub = self.create_subscription(
            String,
            self.get_parameter("topics.user_selector").value,
            self._on_user_selected,
            10,
        )
    
    def _send_initial_manifest(self) -> None:
        """Send initial manifest to generic_ui."""
        if hasattr(self, "_manifest_sent") and self._manifest_sent:
            return

        manifest = build_initial_manifest(games=self._games_metadata)
        self._game_screen_index = get_instance_index(
            manifest.get("instances", []),
            GAME_SCREEN_INSTANCE_ID,
            DEFAULT_GAME_SCREEN_INDEX,
        )
        
        def on_response(success: bool, message: str, hash_: str) -> None:
            if success:
                self.get_logger().info(f"Initial manifest sent. Hash: {hash_}")
            else:
                self.get_logger().error(f"Failed to send manifest: {message}")
        
        if self._manifest_client.set_manifest(manifest, on_response):
            self._manifest_sent = True
    
    def _publish_event(self, event: Dict[str, Any]) -> None:
        """Publish an event to /decision/events."""
        msg = String()
        msg.data = event_to_json(event)
        self._events_pub.publish(msg)
        self.get_logger().info(f"Published event: {event.get('type')}")
    
    def _publish_current_user(self, user_id: int) -> None:
        """Publish current user ID."""
        msg = Int16()
        msg.data = user_id
        self._current_user_pub.publish(msg)
    
    # -------------------- Callbacks --------------------
    
    def _on_decision_state(self, msg: String) -> None:
        """Handle state updates from decision_making."""
        state_data = parse_decision_state(msg.data)
        if state_data is None:
            self.get_logger().warn("Failed to parse decision state")
            return
        
        system_state = state_data.get("state")
        transaction_id = state_data.get("transactionId", 0)
        game_state = state_data.get("gameState")
        session_id = state_data.get("sessionId")
        payload = state_data.get("payload", {})

        self._update_ui_mode(system_state)
        self._update_pause_controls(system_state)

        system_state_upper = str(system_state or "").upper().strip()
        ui_game_state = game_state
        if ui_game_state is None and system_state_upper == "PAUSED" and isinstance(payload, dict):
            raw_underlying = payload.get("gameState")
            if raw_underlying:
                ui_game_state = str(raw_underlying)

        # Update state tracking
        self._latest_transaction_id = transaction_id
        self._current_game_state = game_state
        
        if session_id is not None:
            self._active_session_id = session_id
        elif system_state_upper == "IDLE":
            # Clear session-local state when returning to IDLE.
            self._active_session_id = None
            self._current_round_id = None
            self._current_phase = None
            self._current_question_cache.clear()
            self._round_id_to_phase.clear()
            self._question_id_to_phase.clear()
            self._start_requested = False
            if self._speech_gate is not None:
                self._speech_gate.reset()
        
        # Update translator state
        question = payload.get("question")
        self._ui_translator.update_state(
            transaction_id,
            game_state,
            question,
        )
        
        # Cache question if present
        if question:
            q_id = question.get("questionId")
            if q_id is not None:
                self._current_question_cache[q_id] = question
        
        # Update round info
        if payload.get("roundId"):
            self._current_round_id = payload.get("roundId")
        explicit_phase = payload.get("phase")
        if explicit_phase:
            self._current_phase = explicit_phase
        else:
            # Infer current phase if decision_making payload doesn't include it.
            if question and isinstance(question, dict):
                qid = question.get("questionId")
                if isinstance(qid, int) and qid in self._question_id_to_phase:
                    self._current_phase = self._question_id_to_phase[qid]
            elif self._current_round_id is not None and self._current_round_id in self._round_id_to_phase:
                self._current_phase = self._round_id_to_phase[self._current_round_id]

        # Always patch the embedded state snapshot for the UI screen (and keep phase visible).
        patches = [
            build_game_screen_state_patch(
                str(system_state or ""),
                str(ui_game_state) if ui_game_state is not None else None,
                session_id if isinstance(session_id, int) else None,
                int(transaction_id or 0),
                instance_index=self._game_screen_index,
            ),
            build_game_screen_phase_patch(
                self._current_phase if system_state_upper != "IDLE" else "",
                instance_index=self._game_screen_index,
            ),
        ]
        self._manifest_client.patch_manifest(patches)
        
        self.get_logger().info(
            f"State update: {game_state} (tx={transaction_id})"
        )

        # Schedule completion:
        # - QUESTION_PRESENT: wait for expressive_say completion and then emit ON_COMPLETE
        # - other states: keep existing time-based auto-advance behavior
        if game_state:
            handled = False
            if (
                self._tts_enabled
                and self._speech_gate is not None
                and str(game_state).upper().strip() == "QUESTION_PRESENT"
                and isinstance(payload, dict)
            ):
                question_payload = payload.get("question")
                question_dict = question_payload if isinstance(question_payload, dict) else {}
                prompt = str(question_dict.get("prompt") or "").strip()
                if prompt:
                    handled = self._speech_gate.maybe_speak(
                        SpeechRequest(
                            transaction_id=int(transaction_id or 0),
                            text=prompt,
                            language=self._tts_language,
                        )
                    )

            if not handled:
                self._auto_scheduler.schedule_if_needed(game_state, transaction_id)
        
        # Patch UI based on state
        self._patch_ui_for_state(game_state, payload)
    
    def _on_intent(self, msg: Intent) -> None:
        """Handle input from /intents topic."""
        input_data = self._parse_intent_input_data(msg)
        if input_data is None:
            self.get_logger().warn("Failed to parse /intents data")
            return

        self.get_logger().info(f"Intent input: {input_data}")

        modality = self._map_intent_modality(msg.modality)
        event = self._ui_translator.translate_input_data(
            input_data,
            modality=modality,
        )
        if event:
            # Mark transaction as completed externally
            if self._latest_transaction_id:
                self._auto_scheduler.mark_completed(self._latest_transaction_id)

            self._publish_event(event)

    def _parse_intent_input_data(self, msg: Intent) -> Optional[Dict[str, Any]]:
        """Extract ui-like input data from an Intent message."""
        if not msg.data:
            return None

        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return None

        if not isinstance(data, dict):
            return None

        if "input" in data and isinstance(data["input"], str):
            raw_input = data["input"]
            parsed = parse_input_json(raw_input)
            if parsed is not None:
                return parsed
            return {"label": raw_input}

        if any(key in data for key in ("label", "value", "answer")):
            return data

        return None

    def _map_intent_modality(self, modality: str) -> str:
        """Map Intent modality to game_controller input type strings."""
        if modality == Intent.MODALITY_TOUCHSCREEN:
            return "touch"
        if modality == Intent.MODALITY_SPEECH:
            return "speech"
        if modality == Intent.MODALITY_MOTION:
            return "motion"
        if modality == Intent.MODALITY_OTHER:
            return "other"
        if modality == Intent.MODALITY_INTERNAL:
            return "internal"
        return "unknown"
    
    def _on_game_selected(self, msg: String) -> None:
        """Handle game selection."""
        parsed = self._try_parse_json(msg.data)

        # Reset per-selection overrides.
        self._selected_phases = None
        self._selected_difficulty_override = None
        self._selected_rounds_per_phase_override = None
        self._selected_question_idx = None

        game_slug = ""
        if isinstance(parsed, dict):
            game_info = parsed.get("game")
            if isinstance(game_info, dict):
                game_slug = str(game_info.get("slug") or "").strip().lower()
            if not game_slug:
                game_slug = str(parsed.get("slug") or "").strip().lower()

            phases = parsed.get("phases") or parsed.get("phaseSequence") or parsed.get("phase_sequence")
            if isinstance(phases, list):
                self._selected_phases = [str(p).strip().upper() for p in phases if str(p).strip()]

            difficulty = parsed.get("difficulty")
            if isinstance(difficulty, str) and difficulty.strip():
                self._selected_difficulty_override = difficulty.strip()

            rounds_per_phase = parsed.get("roundsPerPhase") or parsed.get("rounds_per_phase")
            if isinstance(rounds_per_phase, int):
                self._selected_rounds_per_phase_override = rounds_per_phase

            question_idx = parsed.get("questionIdx") or parsed.get("question_idx")
            if isinstance(question_idx, int):
                self._selected_question_idx = question_idx
        else:
            game_slug = msg.data.strip().lower()

        self.get_logger().info(f"Game selected: {game_slug}")

        self._selected_game = game_slug or None
        self._start_requested = True
        
        # Try to start game if we have both game and user
        self._try_start_game()
    
    def _on_user_selected(self, msg: String) -> None:
        """Handle user selection."""
        parsed = self._try_parse_json(msg.data)

        user_id = 1
        if isinstance(parsed, dict):
            user_info = parsed.get("user")
            if isinstance(user_info, dict):
                raw_id = user_info.get("id")
            else:
                raw_id = parsed.get("userId") or parsed.get("id") or parsed.get("user_id")
            try:
                user_id = int(str(raw_id).strip())
            except (TypeError, ValueError):
                user_id = 1
        else:
            try:
                user_id = int(msg.data.strip())
            except ValueError:
                # Might be a username, use 1 as default
                user_id = 1
        
        self.get_logger().info(f"User selected: {user_id}")
        
        self._selected_user = user_id
        self._publish_current_user(user_id)
        
        # Start only if a game selection was explicitly requested.
        if self._start_requested:
            self._try_start_game()
    
    def _on_auto_advance(self, transaction_id: int) -> None:
        """Callback for auto-advance timer."""
        # Publish ON_COMPLETE event
        event = build_on_complete_event(transaction_id)
        self._publish_event(event)
    
    # -------------------- Game Logic --------------------
    
    def _try_start_game(self) -> None:
        """Try to start a game if both game and user are selected."""
        if self._selected_game is None:
            self.get_logger().debug("Cannot start game: no game selected")
            return
        
        if self._selected_user is None:
            self.get_logger().debug("Cannot start game: no user selected")
            return
        
        # Don't start if already in a session
        if self._active_session_id is not None and self._current_game_state not in (None, "IDLE"):
            self.get_logger().info("Session already active, ignoring start request")
            return
        
        self._start_game(self._selected_game)
    
    def _start_game(self, game_slug: str) -> None:
        """Start a game session.
        
        Args:
            game_slug: Game identifier (e.g., "colores")
        """
        # Load game content
        if game_slug not in self._game_content_cache:
            content = load_game_content(game_slug)
            if content is None:
                self.get_logger().error(f"Failed to load game: {game_slug}")
                return
            self._game_content_cache[game_slug] = content
        
        game_content = self._game_content_cache[game_slug]
        
        # Get configuration (allow UI-driven overrides).
        difficulty = self._selected_difficulty_override or self.get_parameter("game_defaults.difficulty").value
        rounds_per_phase = (
            self._selected_rounds_per_phase_override
            if self._selected_rounds_per_phase_override is not None
            else self.get_parameter("game_defaults.rounds_per_phase").value
        )
        phases = self._selected_phases or self.get_parameter("game_defaults.phases").value
        
        # Generate session ID
        self._session_counter += 1
        session_id = self._session_counter
        
        # Build GAME_INIT payload
        payload = build_game_init_payload(
            game_content,
            phases=phases,
            difficulty=difficulty,
            rounds_per_phase=rounds_per_phase,
            session_id=session_id,
        )
        self._index_rounds(payload)

        if self._selected_question_idx is not None:
            payload["questionIdx"] = self._selected_question_idx
        
        # Build and publish event
        event = {
            "type": "GAME_INIT",
            "payload": payload,
        }
        
        self.get_logger().info(
            f"Starting game: {game_slug} (session={session_id}, "
            f"phases={phases}, difficulty={difficulty})"
        )
        
        self._publish_event(event)
        self._start_requested = False

    def _index_rounds(self, game_init_payload: Dict[str, Any]) -> None:
        """Index roundId/questionId to phase for UI convenience."""
        self._round_id_to_phase.clear()
        self._question_id_to_phase.clear()
        rounds = game_init_payload.get("rounds")
        if not isinstance(rounds, list):
            return
        for r in rounds:
            if not isinstance(r, dict):
                continue
            phase = str(r.get("phase") or "").strip().upper()
            rid = r.get("id")
            q = r.get("question")
            qid = q.get("questionId") if isinstance(q, dict) else None
            if phase and isinstance(rid, int):
                self._round_id_to_phase[rid] = phase
            if phase and isinstance(qid, int):
                self._question_id_to_phase[qid] = phase
    
    def _patch_ui_for_state(
        self,
        game_state: Optional[str],
        payload: Dict[str, Any],
    ) -> None:
        """Patch UI manifest based on game state.
        
        Args:
            game_state: Current game state
            payload: State payload
        """
        if not game_state:
            return
        
        patches = build_state_based_patches(
            game_state,
            payload,
            instance_index=self._game_screen_index,
        )
        
        if patches:
            self._manifest_client.patch_manifest(patches)

    def _try_parse_json(self, raw: str) -> Optional[Any]:
        """Try to parse a JSON string, returning None on failure."""
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return None

    def _update_ui_mode(self, system_state: Any) -> None:
        """Swap the GameScreenComponent between menu/game modes."""
        state_upper = str(system_state or "").upper().strip()
        target = "menu" if state_upper == "IDLE" else "game"
        if target == self._ui_screen:
            return

        patches: List[Dict[str, Any]] = [
            build_game_screen_mode_patch(target, instance_index=self._game_screen_index)
        ]

        # When returning to menu, clear any stale question/options so the manifest is coherent
        # even if the UI keeps the component mounted.
        if target == "menu":
            patches.extend(
                [
                    build_game_screen_question_patch(
                        "",
                        [],
                        question_id=0,
                        question_type="",
                        instance_index=self._game_screen_index,
                    ),
                    build_game_screen_options_patch(
                        [],
                        disabled=True,
                        instance_index=self._game_screen_index,
                    ),
                    build_game_screen_controls_patch(
                        show_pause=False,
                        show_resume=False,
                        show_stop=False,
                        show_reset=False,
                        show_skip_phase=False,
                        instance_index=self._game_screen_index,
                    ),
                    build_game_screen_phase_patch("", instance_index=self._game_screen_index),
                    build_game_screen_input_disabled_patch(False, instance_index=self._game_screen_index),
                ]
            )
        else:
            patches.extend(
                [
                    build_game_screen_controls_patch(
                        show_pause=True,
                        show_resume=False,
                        show_stop=True,
                        show_reset=True,
                        show_skip_phase=True,
                        instance_index=self._game_screen_index,
                    ),
                ]
            )

        self._manifest_client.patch_manifest(patches)
        self._ui_screen = target

    def _update_pause_controls(self, system_state: Any) -> None:
        """Toggle pause/resume controls based on decision_making system state."""
        state_upper = str(system_state or "").upper().strip()
        if state_upper == "IDLE":
            # Menu state is handled by _update_ui_mode; never override menu controls from here.
            self._ui_paused = False
            return

        paused = state_upper == "PAUSED"
        if paused == self._ui_paused:
            return

        patches: List[Dict[str, Any]] = []
        if paused:
            patches.append(
                build_game_screen_controls_patch(
                    show_pause=False,
                    show_resume=True,
                    show_stop=True,
                    show_reset=True,
                    show_skip_phase=True,
                    instance_index=self._game_screen_index,
                )
            )
            # decision_making does not accept answers while paused.
            patches.append(build_game_screen_input_disabled_patch(True, instance_index=self._game_screen_index))
        else:
            patches.append(
                build_game_screen_controls_patch(
                    show_pause=True,
                    show_resume=False,
                    show_stop=True,
                    show_reset=True,
                    show_skip_phase=True,
                    instance_index=self._game_screen_index,
                )
            )

        self._manifest_client.patch_manifest(patches)
        self._ui_paused = paused


def main(args: Optional[List[str]] = None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    
    node = GameControllerNode()
    
    # Use multi-threaded executor for async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
