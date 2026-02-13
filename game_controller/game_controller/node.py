"""Game Controller Node.

Main ROS2 node that orchestrates the Colors (Colores) game by:
1. Loading game content and building GAME_INIT payloads
2. Sending UI manifests to generic_ui
3. Translating /intents to decision_making events
4. Scheduling ON_COMPLETE events for auto-advancing states
5. Patching manifest based on game state changes
"""

from __future__ import annotations

import copy
import json
import os
import random
import time
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
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
from .chatbot_client import ChatbotClient
from .content.loaders import load_game_content
from .content.loaders import get_all_games_metadata
from .content.builder import build_game_init_payload
from .content.correctness import compute_correct_for_question, normalize_text
from .ui.manifest_builder import (
    CONTROLS_PAUSED,
    CONTROLS_PLAYING,
    DEFAULT_GAME_SCREEN_INDEX,
    GAME_SCREEN_INSTANCE_ID,
    build_game_screen_answer_type_patch,
    build_controls_patch,
    build_game_screen_mode_patches,
    build_game_screen_pause_patch,
    build_selector_config,
    build_initial_manifest,
    build_input_disabled_patch,
    build_state_based_patches,
    build_hint_highlight_patches,
    build_game_screen_effect_patch,
    build_game_screen_phase_patch,
    build_game_screen_state_patch,
    get_instance_index,
)
from .ui.manifest_client import ManifestClient


class GameControllerNode(Node):
    """ROS2 node for game orchestration.
    
    Subscribes to:
        - /decision/state: Track game state from decision_making
        - /intents: User input and control commands (via communication_hub)
        - /ui/input: Direct UI events fallback (when communication_hub is absent)
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
        self._current_system_state: str = "IDLE"
        self._current_question_cache: Dict[int, Dict[str, Any]] = {}
        self._current_round_id: Optional[int] = None
        self._current_phase: Optional[str] = None
        self._current_difficulty: str = "basic"
        self._active_game_slug: Optional[str] = None
        self._latest_question_payload: Optional[Dict[str, Any]] = None
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
        self._manifest_seed_in_flight: bool = False
        self._manifest_sent: bool = False
        self._startup_idle_reset_requested: bool = False
        
        # Loaded game content
        self._game_content_cache: Dict[str, Dict[str, Any]] = {}
        self._games_metadata: List[Dict[str, Any]] = get_all_games_metadata()
        self._game_screen_config_cache: Dict[str, Any] = build_selector_config(games=self._games_metadata)
        self._manifest_snapshot: Dict[str, Any] = {}
        self._manifest_debug_path = str(
            self.get_parameter("debug.manifest_dump_path").value or "/tmp/game_controller_last_manifest.json"
        ).strip()

        # Helpers to infer phase even when decision_making payloads omit it.
        self._round_id_to_phase: Dict[int, str] = {}
        self._question_id_to_phase: Dict[int, str] = {}
        
        # UI input translator
        self._ui_translator = InputTranslator()
        self._ui_input_bridge_enabled = bool(self.get_parameter("input_bridge.enabled").value)
        dedupe_window = float(self.get_parameter("input_bridge.dedupe_window_sec").value or 0.0)
        self._input_dedupe_window_sec = max(0.0, dedupe_window)
        self._recent_input_fingerprints: Dict[str, float] = {}
        # P1 matching progress gate: keep WAIT_INPUT active until all matches are done.
        self._p1_match_tracking_tx: Optional[int] = None
        self._p1_expected_match_ids: Set[str] = set()
        self._p1_completed_match_ids: Set[str] = set()
        self._p1_match_finalized_tx: Optional[int] = None
        
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
        self._tts_rephrase_question_enabled = bool(
            self.get_parameter("tts.rephrase_question_enabled").value
        )
        self._tts_rephrase_correct_enabled = bool(
            self.get_parameter("tts.rephrase_correct_enabled").value
        )
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

        # Optional chatbot clients for semantic answer evaluation and rephrasing.
        self._chatbot_enabled = bool(self.get_parameter("chatbot.enabled").value)
        self._chatbot_evaluate_speech_only = bool(
            self.get_parameter("chatbot.evaluate_speech_only").value
        )
        self._chatbot = None
        if self._chatbot_enabled:
            self._chatbot = ChatbotClient(
                self,
                rephrase_service=self.get_parameter("chatbot.rephrase_service").value,
                evaluate_service=self.get_parameter("chatbot.evaluate_service").value,
                service_wait_timeout_sec=self.get_parameter("chatbot.service_wait_timeout_sec").value,
                callback_group=self._cb_group,
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

        # Track manifest service availability transitions so backend restarts can
        # be recovered by re-sending a full manifest.
        self._manifest_service_available: bool = False
        self.create_timer(
            1.0,
            self._monitor_manifest_service,
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
        self.declare_parameter("topics.ui_input", "/ui/input")
        self.declare_parameter("topics.game_selector", "/game/game_selector")
        self.declare_parameter("topics.user_selector", "/game/user_selector")
        self.declare_parameter("topics.decision_events", "/decision/events")
        self.declare_parameter("topics.current_user", "/game/current_user")
        self.declare_parameter("input_bridge.enabled", True)
        self.declare_parameter("input_bridge.dedupe_window_sec", 0.35)
        
        # Generic UI parameters
        self.declare_parameter("generic_ui.update_manifest_service", "/generic_ui/update_manifest")
        self.declare_parameter("generic_ui.manifest_timeout_sec", 5.0)
        
        # Auto-advance timeouts
        # Intro is skipped by default in the refactored flow.
        self.declare_parameter("auto_advance.phase_intro", 0.0)
        self.declare_parameter("auto_advance.round_setup", 0.05)
        self.declare_parameter("auto_advance.question_present", 0.05)
        self.declare_parameter("auto_advance.fail_l1", 2.0)
        self.declare_parameter("auto_advance.fail_l2", 2.0)
        self.declare_parameter("auto_advance.correct", 0.6)
        self.declare_parameter("auto_advance.correct_p1", 3.0)
        self.declare_parameter("auto_advance.phase_complete", 0.3)

        # Expressive TTS (EmorobCare communication_hub action interface).
        self.declare_parameter("tts.enabled", False)
        self.declare_parameter("tts.action_server", "/expressive_say")
        self.declare_parameter("tts.language", "es")
        self.declare_parameter("tts.server_wait_timeout_sec", 0.2)
        self.declare_parameter("tts.rephrase_question_enabled", True)
        self.declare_parameter("tts.rephrase_correct_enabled", True)

        # Chatbot services for semantic evaluation/rephrasing.
        self.declare_parameter("chatbot.enabled", True)
        self.declare_parameter("chatbot.rephrase_service", "/chatbot/rephrase")
        self.declare_parameter("chatbot.evaluate_service", "/chatbot/evaluate_answer")
        self.declare_parameter("chatbot.service_wait_timeout_sec", 0.05)
        self.declare_parameter("chatbot.evaluate_speech_only", True)
        
        # Game defaults
        self.declare_parameter("game_defaults.difficulty", "basic")
        self.declare_parameter("game_defaults.rounds_per_phase", 2)
        self.declare_parameter("game_defaults.phases", ["P1", "P2", "P3"])
        self.declare_parameter("debug.manifest_dump_path", "/tmp/game_controller_last_manifest.json")
    
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
        # Decision state subscriber (match TRANSIENT_LOCAL QoS used by decision_making publisher)
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_sub = self.create_subscription(
            String,
            self.get_parameter("topics.decision_state").value,
            self._on_decision_state,
            state_qos,
        )
        
        # Intents subscriber
        self._intents_sub = self.create_subscription(
            Intent,
            self.get_parameter("topics.intents").value,
            self._on_intent,
            10,
        )

        # Optional direct UI input subscriber for WebUI flows without communication_hub.
        self._ui_input_sub = None
        if self._ui_input_bridge_enabled:
            self._ui_input_sub = self.create_subscription(
                String,
                self.get_parameter("topics.ui_input").value,
                self._on_ui_input,
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
    
    def _send_initial_manifest(self, force: bool = False) -> None:
        """Send initial manifest to generic_ui."""
        if not force and getattr(self, "_manifest_sent", False):
            return
        if self._manifest_seed_in_flight:
            return

        manifest = build_initial_manifest(games=self._games_metadata)
        self._manifest_snapshot = copy.deepcopy(manifest)
        self._write_manifest_snapshot(reason="set:prepared", manifest_hash="")
        self._refresh_instance_indices(manifest)
        self._sync_game_screen_config_cache(manifest)
        
        def on_response(success: bool, message: str, hash_: str) -> None:
            self._manifest_seed_in_flight = False
            if success:
                self._manifest_sent = True
                self.get_logger().info(f"Initial manifest sent. Hash: {hash_}")
                self._write_manifest_snapshot(reason="set:sent", manifest_hash=hash_)
            else:
                self._manifest_sent = False
                self.get_logger().error(f"Failed to send manifest: {message}")
        
        if self._manifest_client.set_manifest(manifest, on_response):
            self._manifest_seed_in_flight = True
            self._manifest_sent = True
    
    def _publish_event(self, event: Dict[str, Any]) -> None:
        """Publish an event to /decision/events."""
        msg = String()
        msg.data = event_to_json(event)
        self._events_pub.publish(msg)
        event_type = event.get('type')
        payload = event.get('payload', {})
        self.get_logger().info(
            f"[GC] Published /decision/events: type={event_type}, "
            f"tx={payload.get('transactionId', '?')}, "
            f"value={payload.get('value', '-')}, correct={payload.get('correct', '?')}"
        )
    
    def _publish_current_user(self, user_id: int) -> None:
        """Publish current user ID."""
        msg = Int16()
        msg.data = user_id
        self._current_user_pub.publish(msg)
    
    # -------------------- Callbacks --------------------
    
    def _on_decision_state(self, msg: String) -> None:
        """Handle state updates from decision_making."""
        if not getattr(self, "_manifest_sent", False):
            self._send_initial_manifest()

        state_data = parse_decision_state(msg.data)
        if state_data is None:
            self.get_logger().warn("Failed to parse decision state")
            return
        
        system_state = state_data.get("state")
        transaction_id = state_data.get("transactionId", 0)
        game_state = state_data.get("gameState")
        session_id = state_data.get("sessionId")
        raw_payload = state_data.get("payload", {})
        payload: Dict[str, Any] = raw_payload if isinstance(raw_payload, dict) else {}

        system_state_upper = str(system_state or "").upper().strip()
        if self._should_reset_stale_startup_state(system_state_upper, game_state, session_id, payload):
            self._startup_idle_reset_requested = True
            self.get_logger().warn(
                "Detected stale non-IDLE decision state on startup; sending EXIT to recover menu."
            )
            self._publish_event(
                {
                    "type": "GAME_CONTROL",
                    "payload": {"command": "EXIT"},
                }
            )
            return

        mode_patches = self._update_ui_mode(system_state)
        pause_patches = self._update_pause_controls(system_state)

        ui_game_state = game_state
        if ui_game_state is None and system_state_upper == "PAUSED" and isinstance(payload, dict):
            raw_underlying = payload.get("gameState")
            if raw_underlying:
                ui_game_state = str(raw_underlying)

        # Update state tracking
        self._latest_transaction_id = transaction_id
        self._current_system_state = system_state_upper or "IDLE"
        
        if session_id is not None:
            self._active_session_id = session_id
        elif system_state_upper == "IDLE":
            # Clear session-local state when returning to IDLE.
            self._active_session_id = None
            self._active_game_slug = None
            self._current_round_id = None
            self._current_phase = None
            self._latest_question_payload = None
            self._current_question_cache.clear()
            self._round_id_to_phase.clear()
            self._question_id_to_phase.clear()
            self._recent_input_fingerprints.clear()
            self._reset_p1_match_tracking()
            self._start_requested = False
            self._startup_idle_reset_requested = False
            if self._speech_gate is not None:
                self._speech_gate.reset()
        
        # Update translator state
        question_payload = payload.get("question")
        question = question_payload if isinstance(question_payload, dict) else None
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
            self._latest_question_payload = question
        
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
        state_patches: List[Dict[str, Any]] = [
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
        
        self.get_logger().info(
            f"[GC] State update: system={system_state_upper}, game={game_state}, "
            f"tx={transaction_id}, session={session_id}, phase={self._current_phase}"
        )

        # Schedule completion:
        # - QUESTION_PRESENT / CORRECT can gate ON_COMPLETE on expressive speech completion
        # - PHASE_INTRO is configured to auto-skip (timeout 0 by default)
        # - other states keep time-based auto-advance behavior
        game_state_upper = str(game_state or "").upper().strip()
        if game_state_upper in {"FAIL_L1", "FAIL_L2"}:
            self._enrich_failure_payload(payload, game_state_upper)
        if game_state_upper == "CORRECT" and not payload.get("feedback"):
            payload["feedback"] = self._build_correct_feedback_text(payload)
        if game_state:
            handled = self._maybe_handle_state_speech(
                game_state_upper,
                payload,
                int(transaction_id or 0),
            )
            if not handled:
                auto_state = self._auto_advance_state_key(game_state_upper, payload)
                self._auto_scheduler.schedule_if_needed(auto_state, transaction_id)
        
        # Patch UI based on state.
        state_based_patches = self._patch_ui_for_state(game_state, payload)
        all_patches = mode_patches + pause_patches + state_patches + state_based_patches
        self._apply_game_screen_config_patches(all_patches)
        if all_patches:
            self.get_logger().info(f"Sending manifest patch ({len(all_patches)} ops)")
            self._apply_patches_to_manifest_snapshot(all_patches)
            self._write_manifest_snapshot(reason="patch:prepared", manifest_hash="")
            self._manifest_client.patch_manifest(
                all_patches,
                self._on_manifest_patch_response,
            )

    def _monitor_manifest_service(self) -> None:
        """Watch service availability and re-seed manifest after backend restarts."""
        if not self._manifest_client.available:
            self._manifest_service_available = False
            self._manifest_sent = False
            return

        service_available = self._manifest_client.wait_for_service(timeout_sec=0.0)
        if service_available and not self._manifest_service_available:
            if not self._manifest_sent:
                self.get_logger().info(
                    "Manifest service became available; re-sending initial manifest"
                )
                self._send_initial_manifest(force=True)
        elif not service_available and self._manifest_service_available:
            self.get_logger().warn(
                "Manifest service became unavailable; waiting for backend recovery"
            )
            self._manifest_sent = False
            self._manifest_seed_in_flight = False

        self._manifest_service_available = service_available

    def _on_manifest_patch_response(
        self,
        success: bool,
        message: str,
        manifest_hash: str,
    ) -> None:
        """Recover from patch failures by re-sending a full manifest."""
        if success:
            self.get_logger().info(f"Manifest patch applied. Hash: {manifest_hash}")
            self._write_manifest_snapshot(reason="patch:sent", manifest_hash=manifest_hash)
            return
        self.get_logger().warn(
            f"Manifest patch failed: {message}. Re-sending full manifest."
        )
        self._manifest_sent = False
        self._send_initial_manifest(force=True)
    
    def _on_intent(self, msg: Intent) -> None:
        """Handle input from /intents topic."""
        self.get_logger().debug(
            f"[GC] /intents received: intent={msg.intent}, modality={msg.modality}, data={msg.data}"
        )
        input_data = self._parse_intent_input_data(msg)
        if input_data is None:
            self.get_logger().warn(f"[GC] Failed to parse /intents data: {msg.data}")
            return

        modality = self._map_intent_modality(msg.modality)
        self.get_logger().debug(f"[GC] Parsed intent input_data={input_data}, modality={modality}")
        self._handle_ui_like_input(
            input_data=input_data,
            modality=modality,
            source="/intents",
        )

    def _on_ui_input(self, msg: String) -> None:
        """Handle direct input from /ui/input for WebUI-only debugging stacks."""
        self.get_logger().debug(f"[GC] /ui/input received: {msg.data}")
        input_data = self._parse_ui_input_data(msg.data)
        if input_data is None:
            self.get_logger().warn(f"[GC] Failed to parse /ui/input data: {msg.data}")
            return
        self.get_logger().debug(f"[GC] Parsed /ui/input → input_data={input_data}")
        self._handle_ui_like_input(
            input_data=input_data,
            modality="touch",
            source="/ui/input",
        )

    def _handle_ui_like_input(
        self,
        input_data: Dict[str, Any],
        modality: str,
        source: str,
    ) -> None:
        """Translate normalized input data to decision events."""
        if self._is_duplicate_input(input_data, modality):
            self.get_logger().debug(f"[GC] Ignored duplicate input from {source}: {input_data}")
            return

        self.get_logger().info(
            f"[GC] Input from {source}: {input_data} "
            f"(system_state={self._current_system_state}, "
            f"game_state={getattr(self._ui_translator, '_current_game_state', '?')}, "
            f"tx={self._latest_transaction_id}, phase={self._current_phase})"
        )
        if self._handle_p1_matching_input(input_data, modality):
            return
        if self._should_semantic_evaluate(input_data, modality):
            self.get_logger().debug(f"[GC] Routing to semantic evaluation")
            if self._evaluate_and_publish_input(input_data, modality):
                return

        self._publish_translated_input_event(input_data, modality)

    def _publish_translated_input_event(
        self,
        input_data: Dict[str, Any],
        modality: str,
    ) -> None:
        event = self._ui_translator.translate_input_data(
            input_data,
            modality=modality,
        )
        if not event:
            self.get_logger().warn(
                f"[GC] Translator returned None for input_data={input_data} "
                f"(game_state={getattr(self._ui_translator, '_current_game_state', '?')}, "
                f"tx={getattr(self._ui_translator, '_current_transaction_id', '?')})"
            )
            return
        self.get_logger().info(f"[GC] Translated event: {event}")
        if self._latest_transaction_id:
            self._auto_scheduler.mark_completed(self._latest_transaction_id)
        self._publish_event(event)

    def _reset_p1_match_tracking(self) -> None:
        self._p1_match_tracking_tx = None
        self._p1_expected_match_ids.clear()
        self._p1_completed_match_ids.clear()
        self._p1_match_finalized_tx = None

    def _handle_p1_matching_input(
        self,
        input_data: Dict[str, Any],
        modality: str,
    ) -> bool:
        """Gate P1 matching so CORRECT is emitted only after all matches complete."""
        if str(self._current_system_state or "").upper().strip() != "GAME":
            return False
        if str(self._current_phase or "").upper().strip() != "P1":
            return False

        game_state = str(getattr(self._ui_translator, "_current_game_state", "") or "").upper().strip()
        if game_state != "WAIT_INPUT":
            return False
        if self._is_control_input(input_data):
            return False
        if not self._looks_like_matching_input(input_data):
            return False

        tx = int(self._latest_transaction_id or 0)
        self._prepare_p1_match_tracking(tx)
        correct = self._resolve_match_correctness(input_data)

        if correct is False:
            self.get_logger().info("[GC][P1] Incorrect match, forwarding USER_INTENT(correct=false)")
            self._publish_translated_input_event(input_data, modality)
            return True
        if correct is not True:
            self.get_logger().debug("[GC][P1] Could not resolve matching correctness, using default flow")
            return False

        if self._p1_match_finalized_tx == tx and tx > 0:
            self.get_logger().debug(f"[GC][P1] Final match already published for tx={tx}, ignoring duplicate")
            return True

        match_id = self._extract_match_identifier(input_data)
        if not match_id:
            self.get_logger().debug("[GC][P1] Missing match identifier; forwarding event")
            self._publish_translated_input_event(input_data, modality)
            self._p1_match_finalized_tx = tx if tx > 0 else self._p1_match_finalized_tx
            return True

        is_new_match = match_id not in self._p1_completed_match_ids
        if is_new_match:
            self._p1_completed_match_ids.add(match_id)
            self._speak_p1_match_label(match_id)

        expected_total = len(self._p1_expected_match_ids)
        completed_total = len(self._p1_completed_match_ids)
        explicit_done = self._matching_payload_signals_completion(input_data)
        all_done = explicit_done or (
            expected_total > 0 and completed_total >= expected_total
        )

        self.get_logger().info(
            f"[GC][P1] match progress tx={tx}: "
            f"{completed_total}/{expected_total or '?'} "
            f"(match={match_id}, new={is_new_match}, explicit_done={explicit_done})"
        )

        if expected_total == 0 and not explicit_done:
            # Fallback for incomplete payloads: preserve previous single-match semantics
            # instead of blocking the state machine indefinitely.
            self.get_logger().debug("[GC][P1] Expected match count unavailable; forwarding event")
            self._publish_translated_input_event(input_data, modality)
            if tx > 0:
                self._p1_match_finalized_tx = tx
            return True

        if not all_done:
            # Keep WAIT_INPUT active while each individual match gets immediate TTS feedback.
            return True

        self._publish_translated_input_event(input_data, modality)
        if tx > 0:
            self._p1_match_finalized_tx = tx
        return True

    def _prepare_p1_match_tracking(self, transaction_id: int) -> None:
        if transaction_id <= 0:
            return
        if self._p1_match_tracking_tx != transaction_id:
            self._p1_match_tracking_tx = transaction_id
            self._p1_completed_match_ids.clear()
            self._p1_match_finalized_tx = None
            self._p1_expected_match_ids = self._expected_p1_match_ids()
        elif not self._p1_expected_match_ids:
            self._p1_expected_match_ids = self._expected_p1_match_ids()

    def _expected_p1_match_ids(self) -> Set[str]:
        question = (
            self._latest_question_payload
            if isinstance(self._latest_question_payload, dict)
            else {}
        )
        options = question.get("options")
        candidates = options if isinstance(options, list) else []
        expected: Set[str] = set()
        for opt in candidates:
            if not isinstance(opt, dict):
                continue
            for key in ("id", "label", "value", "text"):
                norm = normalize_text(str(opt.get(key) or ""))
                if norm:
                    expected.add(norm)
                    break
        return expected

    def _is_control_input(self, input_data: Dict[str, Any]) -> bool:
        label = str(input_data.get("label") or "").strip().upper()
        return label in {
            "PAUSE",
            "RESUME",
            "RESTART",
            "RESET",
            "EXIT",
            "STOP",
            "BACK",
            "SKIP_PHASE",
            "SKIP",
        }

    def _looks_like_matching_input(self, input_data: Dict[str, Any]) -> bool:
        if not isinstance(input_data, dict):
            return False
        return any(
            key in input_data
            for key in (
                "leftId",
                "rightId",
                "matchedCount",
                "totalMatches",
                "allMatched",
                "all_matches",
                "completed",
                "isComplete",
            )
        ) or (
            "correct" in input_data and input_data.get("correct") is not None
        )

    def _coerce_optional_bool(self, value: Any) -> Optional[bool]:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value or "").strip().lower()
        if not text:
            return None
        if text in {"true", "1", "yes", "si", "sí"}:
            return True
        if text in {"false", "0", "no"}:
            return False
        return None

    def _resolve_match_correctness(self, input_data: Dict[str, Any]) -> Optional[bool]:
        explicit = self._coerce_optional_bool(input_data.get("correct"))
        if explicit is not None:
            return explicit

        nested_answer = input_data.get("answer")
        if isinstance(nested_answer, dict):
            nested = self._coerce_optional_bool(nested_answer.get("correct"))
            if nested is not None:
                return nested

        value = self._extract_input_value(input_data)
        question = (
            self._latest_question_payload
            if isinstance(self._latest_question_payload, dict)
            else {}
        )
        return compute_correct_for_question(question, value)

    def _extract_match_identifier(self, input_data: Dict[str, Any]) -> str:
        for key in ("leftId", "label", "value", "rightId"):
            candidate = input_data.get(key)
            norm = normalize_text(str(candidate or ""))
            if norm:
                return norm
        nested = input_data.get("answer")
        if isinstance(nested, dict):
            for key in ("id", "label", "value", "text"):
                norm = normalize_text(str(nested.get(key) or ""))
                if norm:
                    return norm
        return ""

    def _matching_payload_signals_completion(self, input_data: Dict[str, Any]) -> bool:
        bool_keys = (
            "completed",
            "isComplete",
            "complete",
            "done",
            "finished",
            "allMatched",
            "all_matches",
            "allMatchesComplete",
            "isFinal",
            "final",
        )
        for key in bool_keys:
            if key not in input_data:
                continue
            value = self._coerce_optional_bool(input_data.get(key))
            if value is not None:
                return value

        matched_count = input_data.get("matchedCount")
        total_matches = input_data.get("totalMatches")
        try:
            if matched_count is not None and total_matches is not None:
                return int(matched_count) >= int(total_matches) > 0
        except (TypeError, ValueError):
            pass

        remaining = input_data.get("remaining")
        if remaining is not None:
            try:
                return int(remaining) <= 0
            except (TypeError, ValueError):
                return False
        return False

    def _resolve_p1_match_label(self, match_id: str) -> str:
        question = (
            self._latest_question_payload
            if isinstance(self._latest_question_payload, dict)
            else {}
        )
        options = question.get("options")
        if isinstance(options, list):
            for opt in options:
                if not isinstance(opt, dict):
                    continue
                candidates = (
                    opt.get("id"),
                    opt.get("label"),
                    opt.get("value"),
                    opt.get("text"),
                )
                if any(normalize_text(str(candidate or "")) == match_id for candidate in candidates):
                    label = opt.get("label") or opt.get("id") or opt.get("value") or opt.get("text")
                    if label is not None and str(label).strip():
                        return str(label).strip()
        return str(match_id).strip()

    def _speak_p1_match_label(self, match_id: str) -> None:
        if not self._tts_enabled or self._tts_client is None:
            return
        text = self._resolve_p1_match_label(match_id)
        if not text:
            return

        def _done(success: bool) -> None:
            self.get_logger().debug(
                f"[GC][P1] Match TTS done (label='{text}', success={bool(success)})"
            )

        started = self._tts_client.speak(text, self._tts_language, _done)
        if not started:
            self.get_logger().debug(f"[GC][P1] Match TTS unavailable for label='{text}'")

    def _should_semantic_evaluate(
        self,
        input_data: Dict[str, Any],
        modality: str,
    ) -> bool:
        if not self._chatbot_enabled or self._chatbot is None:
            return False
        if self._chatbot_evaluate_speech_only and modality != "speech":
            return False
        if self._current_system_state != "GAME":
            return False
        current_state = str(getattr(self._ui_translator, "_current_game_state", "") or "").upper()
        if current_state != "WAIT_INPUT":
            return False
        if self._has_explicit_correctness(input_data):
            return False
        phase_code = str(self._current_phase or "").strip().upper()
        return phase_code in {"P1", "P2", "P3", "P4", "P5", "P6"}

    def _has_explicit_correctness(self, input_data: Dict[str, Any]) -> bool:
        if "correct" in input_data and input_data.get("correct") is not None:
            return True
        nested_answer = input_data.get("answer")
        return isinstance(nested_answer, dict) and nested_answer.get("correct") is not None

    def _extract_input_value(self, input_data: Dict[str, Any]) -> Optional[str]:
        value = input_data.get("label")
        if value is None:
            value = input_data.get("value")
        nested_answer = input_data.get("answer")
        if value is None and isinstance(nested_answer, dict):
            value = (
                nested_answer.get("label")
                or nested_answer.get("value")
                or nested_answer.get("id")
                or nested_answer.get("text")
            )
        if value is None and nested_answer is not None and not isinstance(nested_answer, dict):
            value = nested_answer
        if value is None:
            return None
        text = str(value).strip()
        return text if text else None

    def _extract_expected_answer(self, question: Dict[str, Any]) -> str:
        if not isinstance(question, dict):
            return ""
        raw_answer = question.get("answer")
        if raw_answer is not None and str(raw_answer).strip():
            return str(raw_answer).strip()
        options = question.get("options")
        if isinstance(options, list):
            for option in options:
                if not isinstance(option, dict):
                    continue
                if bool(option.get("correct")):
                    candidate = option.get("label") or option.get("id")
                    if candidate is not None and str(candidate).strip():
                        return str(candidate).strip()
        meta = question.get("meta")
        if isinstance(meta, dict):
            for key in ("correct_answer", "correctAnswer", "color", "answer"):
                candidate = meta.get(key)
                if candidate is not None and str(candidate).strip():
                    return str(candidate).strip()
        return ""

    def _extract_question_prompt(self, question: Dict[str, Any]) -> str:
        if not isinstance(question, dict):
            return ""
        for key in ("promptText", "promptVerbal", "prompt", "text"):
            candidate = question.get(key)
            if candidate is not None and str(candidate).strip():
                return str(candidate).strip()
        return ""

    def _evaluate_and_publish_input(
        self,
        input_data: Dict[str, Any],
        modality: str,
    ) -> bool:
        if self._chatbot is None:
            return False

        request_tx = int(self._latest_transaction_id or 0)
        if request_tx <= 0:
            return False

        question = self._latest_question_payload if isinstance(self._latest_question_payload, dict) else {}
        question_prompt = self._extract_question_prompt(question)
        expected_answer = self._extract_expected_answer(question)
        user_answer = self._extract_input_value(input_data)
        if not question_prompt or not expected_answer or not user_answer:
            return False

        def _done(is_correct: Optional[bool], raw_label: str) -> None:
            if int(self._latest_transaction_id or 0) != request_tx:
                self.get_logger().debug(
                    f"Ignoring stale EvaluateAnswer result for tx={request_tx} "
                    f"(current tx={self._latest_transaction_id})"
                )
                return
            resolved = is_correct
            if resolved is None:
                resolved = compute_correct_for_question(question, user_answer)
            if resolved is None:
                resolved = normalize_text(user_answer) == normalize_text(expected_answer)

            enriched = dict(input_data)
            enriched["correct"] = bool(resolved)
            self.get_logger().debug(
                f"EvaluateAnswer resolved speech intent (tx={request_tx}, "
                f"label='{raw_label}', correct={bool(resolved)})"
            )
            self._publish_translated_input_event(enriched, modality)

        return self._chatbot.evaluate_answer(
            question=question_prompt,
            expected_answer=expected_answer,
            answer=user_answer,
            done=_done,
        )

    def _is_duplicate_input(self, input_data: Dict[str, Any], modality: str) -> bool:
        """Best-effort de-duplication across /ui/input and /intents sources."""
        if self._input_dedupe_window_sec <= 0.0:
            return False

        try:
            payload_key = json.dumps(input_data, ensure_ascii=False, sort_keys=True)
        except (TypeError, ValueError):
            payload_key = str(input_data)
        key = f"{modality}:{payload_key}"
        now = time.monotonic()

        # Prune stale keys.
        stale_keys = [
            old_key
            for old_key, ts in self._recent_input_fingerprints.items()
            if now - ts > self._input_dedupe_window_sec
        ]
        for old_key in stale_keys:
            self._recent_input_fingerprints.pop(old_key, None)

        previous_ts = self._recent_input_fingerprints.get(key)
        self._recent_input_fingerprints[key] = now
        if previous_ts is None:
            return False
        return (now - previous_ts) <= self._input_dedupe_window_sec

    def _parse_intent_input_data(self, msg: Intent) -> Optional[Dict[str, Any]]:
        """Extract ui-like input data from an Intent message."""
        if not msg.data:
            self.get_logger().debug("[GC] _parse_intent: empty msg.data")
            return None

        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().debug(f"[GC] _parse_intent: JSON parse failed: {msg.data}")
            return None

        if not isinstance(data, dict):
            self.get_logger().debug(f"[GC] _parse_intent: data is not dict: {type(data)}")
            return None

        if "input" in data and isinstance(data["input"], str):
            raw_input = data["input"]
            parsed = parse_input_json(raw_input)
            if parsed is not None:
                result = self._normalize_input_data(parsed)
                self.get_logger().debug(f"[GC] _parse_intent: unwrapped 'input' key → {result}")
                return result
            return {"label": raw_input}

        if any(key in data for key in ("label", "value", "answer", "leftId")):
            result = self._normalize_input_data(data)
            self.get_logger().debug(f"[GC] _parse_intent: normalized data → {result}")
            return result

        self.get_logger().debug(
            f"[GC] _parse_intent: no recognized keys in data. Keys={list(data.keys())}"
        )
        return None

    def _parse_ui_input_data(self, raw: str) -> Optional[Dict[str, Any]]:
        """Extract ui-like input data from a /ui/input String payload."""
        parsed = parse_input_json(raw)
        if parsed is not None:
            return self._normalize_input_data(parsed)
        label = str(raw or "").strip()
        if not label:
            return None
        return {"label": label}

    def _normalize_input_data(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Normalize nested UI payload variants into translator-friendly dicts."""
        if not isinstance(data, dict):
            return None

        normalized: Dict[str, Any] = dict(data)

        # Unwrap common nested payload wrappers from generic_ui events.
        for _ in range(3):
            candidate: Optional[Dict[str, Any]] = None
            raw_input = normalized.get("input")
            raw_data = normalized.get("data")
            if isinstance(raw_input, str):
                candidate = parse_input_json(raw_input)
            elif isinstance(raw_data, str):
                candidate = parse_input_json(raw_data)
            if isinstance(candidate, dict):
                normalized = dict(candidate)
                continue
            break

        # Flatten nested answer object emitted by ButtonAnswer.
        nested_answer = normalized.get("answer")
        if isinstance(nested_answer, dict):
            if "label" not in normalized:
                raw_label = (
                    nested_answer.get("label")
                    or nested_answer.get("value")
                    or nested_answer.get("id")
                    or nested_answer.get("text")
                )
                if raw_label is not None:
                    normalized["label"] = raw_label
            if "value" not in normalized and normalized.get("label") is not None:
                normalized["value"] = normalized.get("label")
            if "correct" not in normalized and "correct" in nested_answer:
                normalized["correct"] = nested_answer.get("correct")

        # Normalize MatchingPhase payloads: {leftId, rightId, correct} → {label, value, correct}
        if "label" not in normalized and "value" not in normalized:
            left_id = normalized.get("leftId")
            if left_id is not None and str(left_id).strip():
                normalized["label"] = str(left_id).strip()
                normalized["value"] = str(left_id).strip()
                self.get_logger().debug(
                    f"[GC] _normalize: MatchingPhase leftId='{left_id}' → label/value"
                )

        # Treat plain event text as label when structured fields are missing.
        if not any(key in normalized for key in ("label", "value", "answer", "correct")):
            text = normalized.get("text")
            if isinstance(text, str) and text.strip():
                normalized["label"] = text.strip()

        return normalized if normalized else None

    def _map_intent_modality(self, modality: Any) -> str:
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
        raw = str(modality or "").strip().lower()
        if raw in {"__modality_touchscreen__", "touchscreen", "touch"}:
            return "touch"
        if raw in {"__modality_speech__", "speech", "voice"}:
            return "speech"
        if raw in {"__modality_motion__", "motion", "gesture"}:
            return "motion"
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
        if self._active_session_id is not None and self._current_system_state != "IDLE":
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
        
        self._active_game_slug = game_slug
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

    def _maybe_handle_state_speech(
        self,
        game_state_upper: str,
        payload: Dict[str, Any],
        transaction_id: int,
    ) -> bool:
        if not self._tts_enabled or self._speech_gate is None:
            return False
        if transaction_id <= 0:
            return False

        if game_state_upper == "PHASE_INTRO":
            return False

        if game_state_upper == "QUESTION_PRESENT":
            question_payload = payload.get("question")
            question = question_payload if isinstance(question_payload, dict) else {}
            prompt = str(
                question.get("promptVerbal")
                or question.get("prompt")
                or question.get("promptText")
                or ""
            ).strip()
            if not prompt:
                return False
            expected_answer = self._extract_expected_answer(question)
            return self._speak_with_optional_rephrase(
                transaction_id=transaction_id,
                text=prompt,
                expected_answer=expected_answer,
                allow_rephrase=self._tts_rephrase_question_enabled,
            )

        if game_state_upper == "CORRECT":
            feedback = str(payload.get("feedback") or "").strip()
            if not feedback:
                return False
            phase_code = str(payload.get("phase") or self._current_phase or "").strip().upper()
            if phase_code == "P1":
                return False
            expected_answer = self._extract_expected_answer(
                self._latest_question_payload if isinstance(self._latest_question_payload, dict) else {}
            )
            return self._speak_with_optional_rephrase(
                transaction_id=transaction_id,
                text=feedback,
                expected_answer=expected_answer,
                allow_rephrase=self._tts_rephrase_correct_enabled and phase_code != "P1",
            )

        if game_state_upper in {"FAIL_L1", "FAIL_L2"}:
            action = self._normalize_hint_action(payload.get("action"))
            should_speak = (
                self._is_say_correct_action(action)
                or game_state_upper == "FAIL_L2"
                or bool(payload.get("autoAdvance"))
            )
            if not should_speak:
                return False
            hint = str(payload.get("hint") or "").strip()
            if not hint:
                correct_text = self._resolve_correct_answer_text(payload.get("correctOptionId"))
                hint = (
                    f"La respuesta correcta es {correct_text}."
                    if correct_text
                    else "Te digo la respuesta correcta."
                )
            return self._speak_with_optional_rephrase(
                transaction_id=transaction_id,
                text=hint,
                expected_answer=self._resolve_correct_answer_text(payload.get("correctOptionId")),
                allow_rephrase=False,
            )

        return False

    def _auto_advance_state_key(self, game_state_upper: str, payload: Dict[str, Any]) -> str:
        state_key = str(game_state_upper or "").upper().strip()
        if state_key == "CORRECT":
            phase_code = str(payload.get("phase") or self._current_phase or "").strip().upper()
            if phase_code == "P1":
                return "CORRECT_P1"
        return state_key

    def _speak_with_optional_rephrase(
        self,
        transaction_id: int,
        text: str,
        expected_answer: str = "",
        allow_rephrase: bool = True,
    ) -> bool:
        cleaned = str(text or "").strip()
        if not cleaned or self._speech_gate is None:
            return False

        if not allow_rephrase or not self._chatbot_enabled or self._chatbot is None:
            return self._speech_gate.maybe_speak(
                SpeechRequest(
                    transaction_id=int(transaction_id),
                    text=cleaned,
                    language=self._tts_language,
                )
            )

        def _speak_candidate(candidate_text: str) -> None:
            if int(self._latest_transaction_id or 0) != int(transaction_id):
                return
            started = self._speech_gate.maybe_speak(
                SpeechRequest(
                    transaction_id=int(transaction_id),
                    text=candidate_text,
                    language=self._tts_language,
                )
            )
            if not started:
                self._on_auto_advance(int(transaction_id))

        def _on_rephrased(rephrased: str) -> None:
            candidate = str(rephrased or "").strip() or cleaned
            if expected_answer and not self._rephrase_preserves_expected_answer(
                candidate,
                expected_answer,
            ):
                self.get_logger().debug(
                    "Discarding rephrase candidate because expected answer token is missing"
                )
                candidate = cleaned
            _speak_candidate(candidate)

        started = self._chatbot.rephrase(
            sentence=cleaned,
            forbidden_expressions=[],
            n_alternatives=3,
            done=_on_rephrased,
        )
        if started:
            return True

        return self._speech_gate.maybe_speak(
            SpeechRequest(
                transaction_id=int(transaction_id),
                text=cleaned,
                language=self._tts_language,
            )
        )

    def _rephrase_preserves_expected_answer(
        self,
        candidate_text: str,
        expected_answer: str,
    ) -> bool:
        expected = normalize_text(str(expected_answer or ""))
        if not expected:
            return True
        normalized_candidate = normalize_text(str(candidate_text or ""))
        if not normalized_candidate:
            return False
        tokens = {tok for tok in normalized_candidate.split(" ") if tok}
        if len(expected) <= 3:
            return expected in tokens
        if expected in normalized_candidate:
            return True
        return expected in tokens

    def _build_positive_feedback_text(self, payload: Dict[str, Any]) -> str:
        existing = str(payload.get("feedback") or "").strip()
        question = (
            self._latest_question_payload
            if isinstance(self._latest_question_payload, dict)
            else {}
        )
        expr = self._extract_expected_answer(question)

        game_content = self._game_content_for_active_session()
        positive_feedback = game_content.get("positive_feedback") if isinstance(game_content, dict) else None
        templates = []
        if isinstance(positive_feedback, dict):
            raw_templates = positive_feedback.get("fewshot_examples")
            if isinstance(raw_templates, list):
                templates = [str(item).strip() for item in raw_templates if str(item).strip()]

        base = existing or "¡Muy bien!"
        if templates:
            base = random.choice(templates)

        values = {
            "expr": expr,
            "value": expr,
            "answer": expr,
            "colour": expr,
            "color": expr,
            "animal": expr,
            "fruta": expr,
            "emocion": expr,
            "forma": expr,
            "objeto": expr,
            "lugar": expr,
        }
        return self._safe_format(base, values).strip() or "¡Muy bien!"

    def _phase_success_response(self, phase_code: str) -> str:
        game_content = self._game_content_for_active_session()
        phase_config = game_content.get("phaseConfig") if isinstance(game_content, dict) else None
        if not isinstance(phase_config, dict):
            return ""

        normalized = str(phase_code or "").strip()
        candidates = (
            normalized,
            normalized.upper(),
            normalized.lower(),
        )
        cfg = None
        for key in candidates:
            value = phase_config.get(key)
            if isinstance(value, dict):
                cfg = value
                break
        if not isinstance(cfg, dict):
            return ""

        response = cfg.get("successResponse") or cfg.get("success_response")
        return str(response or "").strip()

    def _build_correct_feedback_text(self, payload: Dict[str, Any]) -> str:
        phase_code = str(payload.get("phase") or self._current_phase or "").strip().upper()
        if phase_code == "P1":
            return "¡Correcto!"
        if phase_code == "P5":
            response = self._phase_success_response("P5")
            if response:
                return response
            return "Aquííí"
        return self._build_positive_feedback_text(payload)

    def _game_content_for_active_session(self) -> Dict[str, Any]:
        slug = str(self._active_game_slug or self._selected_game or "").strip().lower()
        if not slug:
            return {}
        cached = self._game_content_cache.get(slug)
        if isinstance(cached, dict):
            return cached
        loaded = load_game_content(slug)
        if isinstance(loaded, dict):
            self._game_content_cache[slug] = loaded
            return loaded
        return {}

    def _safe_format(self, template: str, values: Dict[str, Any]) -> str:
        class _SafeDict(dict):
            def __missing__(self, key: str) -> str:  # type: ignore[override]
                return "{" + key + "}"

        try:
            return str(template or "").format_map(_SafeDict(values))
        except Exception:
            return str(template or "")

    def _normalize_hint_action(self, action: Any) -> str:
        return str(action or "").strip().lower()

    def _is_say_correct_action(self, action: str) -> bool:
        normalized = self._normalize_hint_action(action)
        return normalized in {"say_correct", "sayanswer", "say_answer"}

    def _resolve_correct_answer_text(self, correct_option_id: Any = None) -> str:
        question = (
            self._latest_question_payload
            if isinstance(self._latest_question_payload, dict)
            else {}
        )
        options = question.get("options")
        target = normalize_text(str(correct_option_id or ""))

        if isinstance(options, list):
            if target:
                for option in options:
                    if not isinstance(option, dict):
                        continue
                    candidates = [
                        option.get("id"),
                        option.get("label"),
                        option.get("value"),
                    ]
                    if any(normalize_text(str(candidate or "")) == target for candidate in candidates):
                        label = option.get("label") or option.get("id") or option.get("value")
                        if label is not None and str(label).strip():
                            return str(label).strip()

            for option in options:
                if not isinstance(option, dict):
                    continue
                if bool(option.get("correct")):
                    label = option.get("label") or option.get("id") or option.get("value")
                    if label is not None and str(label).strip():
                        return str(label).strip()

        expected = self._extract_expected_answer(question)
        if expected:
            return expected
        return ""

    def _enrich_failure_payload(self, payload: Dict[str, Any], game_state_upper: str) -> None:
        action = self._normalize_hint_action(payload.get("action"))
        final_attempt = game_state_upper == "FAIL_L2" or bool(payload.get("autoAdvance"))
        if not final_attempt and not self._is_say_correct_action(action):
            return

        correct_text = self._resolve_correct_answer_text(payload.get("correctOptionId"))
        if correct_text:
            base_text = f"La respuesta correcta es {correct_text}."
        else:
            base_text = "Te digo la respuesta correcta."

        if final_attempt:
            positive = self._build_positive_feedback_text(payload)
            combined = " ".join(part for part in (base_text, positive) if str(part).strip()).strip()
            if combined:
                payload["hint"] = combined
        else:
            payload["hint"] = base_text

        payload["action"] = "say_correct"

    def _patch_ui_for_state(
        self,
        game_state: Optional[str],
        payload: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        """Patch UI manifest based on game state.
        
        Args:
            game_state: Current game state
            payload: State payload
        """
        if not game_state:
            return []
        state_upper = str(game_state).upper().strip()
        if state_upper == "PHASE_INTRO":
            return []

        payload_for_ui: Dict[str, Any] = dict(payload) if isinstance(payload, dict) else {}
        # decision_making QUESTION_PRESENT payloads may omit `phase`; inject the
        # latest known phase so UI answer type selection can stay phase-aware.
        if self._current_phase and not payload_for_ui.get("phase"):
            payload_for_ui["phase"] = self._current_phase
        if (
            state_upper in {"WAIT_INPUT", "FAIL_L1", "FAIL_L2", "CORRECT"}
            and isinstance(self._latest_question_payload, dict)
            and "question" not in payload_for_ui
        ):
            payload_for_ui["question"] = self._latest_question_payload

        patches = build_state_based_patches(
            game_state,
            payload_for_ui,
            instance_index=self._game_screen_index,
        )
        if state_upper == "CORRECT":
            patches.append(
                build_game_screen_effect_patch(
                    "confetti",
                    instance_index=self._game_screen_index,
                )
            )
            phase_code = str(payload_for_ui.get("phase") or self._current_phase or "").strip().upper()
            if phase_code == "P5":
                options: List[Dict[str, Any]] = []
                question_payload = payload_for_ui.get("question")
                if isinstance(question_payload, dict):
                    raw_options = question_payload.get("options")
                    if isinstance(raw_options, list):
                        options = [opt for opt in raw_options if isinstance(opt, dict)]
                if not options and isinstance(payload_for_ui.get("options"), list):
                    options = [opt for opt in payload_for_ui.get("options", []) if isinstance(opt, dict)]
                if not payload_for_ui.get("correctOptionId"):
                    inferred_correct = self._resolve_correct_answer_text()
                    if inferred_correct:
                        payload_for_ui["correctOptionId"] = inferred_correct
                highlight_patches = build_hint_highlight_patches(
                    options,
                    payload_for_ui.get("correctOptionId"),
                    instance_index=self._game_screen_index,
                )
                if highlight_patches:
                    patches.extend(highlight_patches)
                    patches.append(
                        build_game_screen_answer_type_patch(
                            "button",
                            instance_index=self._game_screen_index,
                        )
                    )
        if state_upper == "FAIL_L1":
            action = self._normalize_hint_action(payload_for_ui.get("action"))
            if not self._is_say_correct_action(action):
                options: List[Dict[str, Any]] = []
                question_payload = payload_for_ui.get("question")
                if isinstance(question_payload, dict):
                    raw_options = question_payload.get("options")
                    if isinstance(raw_options, list):
                        options = [opt for opt in raw_options if isinstance(opt, dict)]
                if not options and isinstance(payload_for_ui.get("options"), list):
                    options = [opt for opt in payload_for_ui.get("options", []) if isinstance(opt, dict)]
                highlight_patches = build_hint_highlight_patches(
                    options,
                    payload_for_ui.get("correctOptionId"),
                    instance_index=self._game_screen_index,
                )
                patches.extend(highlight_patches)
        return patches

    def _refresh_instance_indices(self, manifest: Dict[str, Any]) -> None:
        instances = manifest.get("instances") if isinstance(manifest.get("instances"), list) else []
        self._game_screen_index = get_instance_index(
            instances,
            GAME_SCREEN_INSTANCE_ID,
            DEFAULT_GAME_SCREEN_INDEX,
        )

    def _sync_game_screen_config_cache(self, manifest: Dict[str, Any]) -> None:
        instances = manifest.get("instances") if isinstance(manifest.get("instances"), list) else []
        if 0 <= self._game_screen_index < len(instances):
            inst = instances[self._game_screen_index]
            if isinstance(inst, dict) and isinstance(inst.get("config"), dict):
                self._game_screen_config_cache = copy.deepcopy(inst["config"])

    def _apply_game_screen_config_patches(self, patches: List[Dict[str, Any]]) -> None:
        root_path = f"/instances/{self._game_screen_index}/config"
        field_prefix = f"{root_path}/"
        for patch in patches:
            if str(patch.get("op") or "").lower() != "replace":
                continue
            path = str(patch.get("path") or "")
            if path == root_path:
                val = patch.get("value")
                if isinstance(val, dict):
                    self._game_screen_config_cache = copy.deepcopy(val)
                continue
            if not path.startswith(field_prefix):
                continue
            field = path[len(field_prefix) :]
            if "/" in field or not field:
                continue
            self._game_screen_config_cache[field] = copy.deepcopy(patch.get("value"))

    def _write_manifest_snapshot(self, reason: str, manifest_hash: str) -> None:
        path = str(self._manifest_debug_path or "").strip()
        if not path:
            return
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(self._manifest_snapshot, handle, ensure_ascii=False, indent=2)
            self.get_logger().info(
                f"Manifest snapshot saved ({reason}, hash={manifest_hash or '-'}) -> {path}"
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to save manifest snapshot to {path}: {exc}")

    def _apply_patches_to_manifest_snapshot(self, patches: List[Dict[str, Any]]) -> None:
        if not isinstance(self._manifest_snapshot, dict) or not self._manifest_snapshot:
            return
        for patch in patches:
            op = str(patch.get("op") or "").lower().strip()
            if op not in {"replace", "add"}:
                continue
            path = str(patch.get("path") or "")
            if not path.startswith("/"):
                continue
            try:
                self._set_json_pointer_value(
                    self._manifest_snapshot,
                    path,
                    copy.deepcopy(patch.get("value")),
                    create_missing=(op == "add"),
                )
            except Exception as exc:
                self.get_logger().warn(f"Failed applying local manifest patch '{path}': {exc}")

    def _set_json_pointer_value(
        self,
        document: Any,
        pointer: str,
        value: Any,
        create_missing: bool = False,
    ) -> None:
        tokens = pointer.lstrip("/").split("/")
        current = document
        for raw_token in tokens[:-1]:
            token = raw_token.replace("~1", "/").replace("~0", "~")
            if isinstance(current, list):
                index = int(token)
                current = current[index]
                continue
            if token not in current:
                if not create_missing:
                    raise KeyError(token)
                current[token] = {}
            current = current[token]

        last = tokens[-1].replace("~1", "/").replace("~0", "~")
        if isinstance(current, list):
            index = int(last)
            if index >= len(current):
                if not create_missing:
                    raise IndexError(index)
                current.append(value)
            else:
                current[index] = value
            return
        current[last] = value

    def _should_reset_stale_startup_state(
        self,
        system_state_upper: str,
        game_state: Any,
        session_id: Any,
        payload: Dict[str, Any],
    ) -> bool:
        if self._startup_idle_reset_requested:
            return False
        if system_state_upper != "GAME":
            return False
        if self._start_requested or self._selected_game is not None:
            return False
        if self._active_session_id is not None:
            return False
        if session_id is None:
            return False
        if isinstance(payload.get("question"), dict):
            return False
        game_state_upper = str(game_state or "").upper().strip()
        return game_state_upper in {
            "WAIT_INPUT",
            "QUESTION_PRESENT",
            "ROUND_SETUP",
            "CORRECT",
            "FAIL_L1",
            "FAIL_L2",
        }

    def _try_parse_json(self, raw: str) -> Optional[Any]:
        """Try to parse a JSON string, returning None on failure."""
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return None

    def _update_ui_mode(self, system_state: Any) -> List[Dict[str, Any]]:
        """Swap `game_screen` component based on system state."""
        state_upper = str(system_state or "").upper().strip()
        target = "menu" if state_upper == "IDLE" else "game"
        if target == self._ui_screen:
            return []

        self._ui_screen = target
        if target == "menu":
            self._ui_paused = False
        return build_game_screen_mode_patches(
            target,
            games=self._games_metadata,
            instance_index=self._game_screen_index,
        )

    def _update_pause_controls(self, system_state: Any) -> List[Dict[str, Any]]:
        """Toggle pause state based on decision_making system state."""
        state_upper = str(system_state or "").upper().strip()
        if state_upper == "IDLE":
            # Menu state is handled by _update_ui_mode.
            self._ui_paused = False
            return []

        paused = state_upper == "PAUSED"
        if paused == self._ui_paused:
            return []

        controls = CONTROLS_PAUSED if paused else CONTROLS_PLAYING
        self._ui_paused = paused
        return [
            build_game_screen_pause_patch(paused, instance_index=self._game_screen_index),
            build_controls_patch(controls, instance_index=self._game_screen_index),
            build_input_disabled_patch(paused, instance_index=self._game_screen_index),
        ]


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
