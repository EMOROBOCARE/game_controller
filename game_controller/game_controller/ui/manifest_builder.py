"""Generic UI manifest builder.

This workspace uses `generic_ui` to render a remotely-federated React UI. The controller updates
the manifest via `/generic_ui/update_manifest` (set once, then JSON patches).

Contract (EmorobCare):
- The manifest always contains a `UserPanel` for user selection.
- The manifest always contains a single `GameScreenComponent` that renders both:
  - Main menu game selection
  - In-game phases/questions/controls

This keeps the UI stable (same components always mounted) and pushes all rendering logic into a
single screen component.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Set


REMOTE_ENTRY_URL = "/emorobcare-components/assets/remoteEntry.js"
REMOTE_SCOPE = "demo"
GAME_SCREEN_INSTANCE_ID = "game_screen"
DEFAULT_GAME_SCREEN_INDEX = 0
ASSET_CDN_ENV = "ASSET_CDN_URL"
ASSET_CDN_DEFAULT = "http://localhost:8084/emorobcare-components"
INCLUDE_CORRECT_ENV = "GAME_CONTROLLER_INCLUDE_CORRECT_OPTIONS"

# Minimal default users list for the UserPanel menu component.
DEFAULT_USERS = [
    {"id": "1", "name": "Pepe"},
    {"id": "2", "name": "María"},
    {"id": "3", "name": "Juan"},
]


def build_component_registry() -> Dict[str, Dict[str, Any]]:
    """Build the component registry for emorobcare components."""
    return {
        "UserPanel": {
            "url": REMOTE_ENTRY_URL,
            "scope": REMOTE_SCOPE,
            "module": "./UserPanel",
        },
        "GameScreenComponent": {
            "url": REMOTE_ENTRY_URL,
            "scope": REMOTE_SCOPE,
            "module": "./GameScreenComponent",
        },
    }


def build_ops() -> Dict[str, Dict[str, Any]]:
    """Build ROS operations for the game controller."""
    return {
        "game_selector": {
            "kind": "topic_pub",
            "rosType": "std_msgs/msg/String",
            "topic": "/game/game_selector",
        },
        "user_selector": {
            "kind": "topic_pub",
            "rosType": "std_msgs/msg/String",
            "topic": "/game/user_selector",
        },
        "ui_input": {
            "kind": "topic_pub",
            "rosType": "std_msgs/msg/String",
            "topic": "/ui/input",
        },
        "current_user": {
            "kind": "topic_sub",
            "rosType": "std_msgs/msg/Int16",
            "topic": "/game/current_user",
        },
        "decision_state": {
            "kind": "topic_sub",
            "rosType": "std_msgs/msg/String",
            "topic": "/decision/state",
        },
        "ui_update": {
            "kind": "topic_sub",
            "rosType": "std_msgs/msg/String",
            "topic": "/ui/update",
        },
    }

def build_root_layout() -> Dict[str, Any]:
    """Build the root layout.

    This layout is stable across all states: the UI decides what to render based on
    `GameScreenComponent` config + decision_making state.
    """
    return {
        "mode": "grid",
        "columns": 12,
        "items": [
            {"instanceId": "user_panel", "col": 1, "row": 1, "colSpan": 12, "rowSpan": 1},
            {"instanceId": "game_screen", "col": 1, "row": 2, "colSpan": 12, "rowSpan": 6},
        ],
    }


def build_menu_instances(
    users: Optional[List[Dict[str, Any]]] = None,
) -> List[Dict[str, Any]]:
    """Build menu-related instances (currently only UserPanel)."""
    if users is None:
        users = list(DEFAULT_USERS)

    active_user = users[0] if users else None
    user_panel_config: Dict[str, Any] = {
        "users": users,
        "userPanelOpId": "user_selector",
        "disabled": False,
    }
    if active_user is not None:
        user_panel_config["activeUser"] = active_user

    return [
        {
            "id": "user_panel",
            "component": "UserPanel",
            "config": user_panel_config,
            "capabilities": [],
        },
    ]


def build_game_screen_instance(
    games: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Build the main GameScreenComponent instance."""
    if games is None:
        games = []
    return {
        "id": GAME_SCREEN_INSTANCE_ID,
        "component": "GameScreenComponent",
        "config": {
            # "menu" | "game"
            "mode": "menu",
            # Operation IDs (must match ops table).
            "startGameOpId": "game_selector",
            "uiInputOpId": "ui_input",
            # Menu content (game list + defaults UI may present).
            "games": games,
            # Debug / state fields (controller can patch these).
            "state": {"system": "IDLE", "gameState": None, "sessionId": None, "transactionId": 0},
            "phase": "",
            # Current prompt/question shown on screen.
            "question": {"questionId": 0, "questionType": "", "text": "", "imgs": []},
            # Options for current interaction (touch/matching/yesno/etc.).
            "options": [],
            # Controls shown in the screen.
            "controls": {
                "showPause": False,
                "showResume": False,
                "showStop": False,
                "showReset": False,
                "showSkipPhase": False,
            },
            # Disable user interaction (e.g., PHASE_INTRO, PAUSED).
            "inputDisabled": False,
        },
        "capabilities": ["ui_input", "game_selector", "decision_state", "ui_update"],
        "bindings": {"uiInput": "ui_input", "startGame": "game_selector"},
    }


def build_ui_config() -> Dict[str, Any]:
    """Build UI shell configuration."""
    return {"chrome": "kiosk", "rowHeightPx": 96, "background": "#f5f0e8"}


def build_initial_manifest(
    games: Optional[List[Dict[str, Any]]] = None,
    users: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Build the complete initial manifest for the game controller."""
    return {
        "version": 1,
        "componentRegistry": build_component_registry(),
        "ops": build_ops(),
        # Stable layout (UserPanel + GameScreenComponent always present).
        "layout": build_root_layout(),
        # Keep indices stable for JSON patching.
        "instances": [build_game_screen_instance(games=games)] + build_menu_instances(users=users),
        "ui": build_ui_config(),
    }

def get_instance_index(
    instances: List[Dict[str, Any]],
    instance_id: str,
    default: int = DEFAULT_GAME_SCREEN_INDEX,
) -> int:
    """Find the index for an instance id in the manifest."""
    for idx, inst in enumerate(instances):
        if inst.get("id") == instance_id:
            return idx
    return default


def build_instance_config_path(instance_index: int, field: str) -> str:
    """Build a JSON patch path for an instance config field."""
    return f"/instances/{int(instance_index)}/config/{field}"


def _asset_cdn_base() -> str:
    base = os.environ.get(ASSET_CDN_ENV, ASSET_CDN_DEFAULT).rstrip("/")
    return base or ASSET_CDN_DEFAULT.rstrip("/")


def rewrite_asset_url(path: Optional[str]) -> str:
    """Rewrite asset paths to browser-reachable URLs."""
    if not path:
        return ""
    if path.startswith("http://") or path.startswith("https://"):
        return path
    if path.startswith("assets/"):
        stripped = path[len("assets/") :].lstrip("/")
        return f"{_asset_cdn_base()}/{stripped}"
    return path


def _normalize_question_images(question: Dict[str, Any]) -> List[str]:
    images: List[str] = []
    for key in ("imageUrls", "image_urls", "imgs", "images"):
        value = question.get(key)
        if isinstance(value, list):
            for entry in value:
                if isinstance(entry, str):
                    images.append(entry)
                elif isinstance(entry, dict):
                    candidate = (
                        entry.get("src")
                        or entry.get("url")
                        or entry.get("imageUrl")
                        or entry.get("image_url")
                        or entry.get("img")
                    )
                    if candidate:
                        images.append(candidate)
    single = (
        question.get("imageUrl")
        or question.get("image_url")
        or question.get("img")
    )
    if isinstance(single, str) and single:
        images.append(single)
    return [rewrite_asset_url(img) for img in images if img]


def _include_correct_in_ui_options(include_correct: Optional[bool]) -> bool:
    if include_correct is not None:
        return include_correct
    env = os.environ.get(INCLUDE_CORRECT_ENV, "")
    return env.strip().lower() in {"1", "true", "yes", "on"}


def build_game_screen_mode_patch(mode: str, instance_index: int = DEFAULT_GAME_SCREEN_INDEX) -> Dict[str, Any]:
    """Patch GameScreenComponent mode ('menu'|'game')."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "mode"),
        "value": str(mode),
    }


def build_game_screen_state_patch(
    system_state: str,
    game_state: Optional[str],
    session_id: Optional[int],
    transaction_id: int,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch the debug/telemetry state embedded in the UI config."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "state"),
        "value": {
            "system": system_state,
            "gameState": game_state,
            "sessionId": session_id,
            "transactionId": int(transaction_id),
        },
    }


def build_game_screen_phase_patch(
    phase: Optional[str],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "phase"),
        "value": str(phase or ""),
    }


def build_game_screen_question_patch(
    text: str,
    images: Optional[List[str]] = None,
    question_id: int = 0,
    question_type: str = "",
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch the question/prompt shown in the GameScreenComponent."""
    if images is None:
        images = []
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "question"),
        "value": {"questionId": int(question_id or 0), "questionType": str(question_type or ""), "text": text, "imgs": images},
    }


def build_game_screen_options_patch(
    options: List[Dict[str, Any]],
    disabled: bool = False,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    patched_options = []
    for opt in options:
        o = dict(opt)
        o.setdefault("disabled", False)
        if disabled:
            o["disabled"] = True
        patched_options.append(o)
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "options"),
        "value": patched_options,
    }


def build_game_screen_controls_patch(
    show_pause: bool = True,
    show_resume: bool = False,
    show_stop: bool = True,
    show_reset: bool = True,
    show_skip_phase: bool = True,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch the pause/resume/exit controls shown in the GameScreenComponent."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "controls"),
        "value": {
            "showPause": bool(show_pause),
            "showResume": bool(show_resume),
            "showStop": bool(show_stop),
            "showReset": bool(show_reset),
            "showSkipPhase": bool(show_skip_phase),
        },
    }


def build_game_screen_input_disabled_patch(
    disabled: bool,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "inputDisabled"),
        "value": bool(disabled),
    }


def build_highlighting_patch(
    options: List[Dict[str, Any]],
    highlighted_ids: Set[str],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Build a patch for P6 pointing phase highlighting."""
    patched = format_options_for_ui(options)
    highlight_set = {str(opt_id) for opt_id in highlighted_ids}
    for opt in patched:
        opt_id = str(opt.get("id", ""))
        if opt_id in highlight_set:
            opt["highlighted"] = True
            opt["hidden"] = False
        else:
            opt["hidden"] = True
            opt["highlighted"] = False
        opt["disabled"] = False
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "options"),
        "value": patched,
    }


def format_options_for_ui(
    options: List[Dict[str, Any]],
    include_correct: Optional[bool] = None,
) -> List[Dict[str, Any]]:
    """Format options list for UI consumption."""
    include_correct = _include_correct_in_ui_options(include_correct)
    ui_options = []
    for opt in options:
        image = opt.get("imageUrl", opt.get("image_url", opt.get("img", "")))
        img_value = rewrite_asset_url(image) if image else ""
        ui_option = {
            "id": opt.get("id", opt.get("label", "")),
            "label": opt.get("label", ""),
            "img": img_value,
            "disabled": opt.get("disabled", False),
            "hidden": opt.get("hidden", False),
            "highlighted": opt.get("highlighted", False),
        }
        if include_correct:
            ui_option["correct"] = opt.get("correct", False)
        ui_options.append({
            **ui_option,
        })
    return ui_options


def build_state_based_patches(
    game_state: str,
    payload: Dict[str, Any],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> List[Dict[str, Any]]:
    """Build manifest patches based on game state."""
    patches: List[Dict[str, Any]] = []
    state_upper = (game_state or "").upper()

    if state_upper == "PHASE_INTRO":
        intro = payload.get("introduction", "")
        phase = payload.get("phase", "")
        if not intro:
            intro = "Vamos a empezar."
        text = f"Fase {phase}: {intro}" if phase else intro
        patches.append(build_game_screen_question_patch(text, [], question_id=0, question_type="phase_intro", instance_index=instance_index))
        patches.append(build_game_screen_options_patch([], disabled=True, instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "ROUND_SETUP":
        patches.append(build_game_screen_question_patch("Preparando...", [], question_id=0, question_type="round_setup", instance_index=instance_index))
        patches.append(build_game_screen_options_patch([], disabled=True, instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "QUESTION_PRESENT":
        question = payload.get("question", {})
        prompt = question.get("prompt", "")
        images = _normalize_question_images(question)
        options = question.get("options", [])
        ui_options = format_options_for_ui(options)
        patches.append(
            build_game_screen_question_patch(
                prompt,
                images,
                question_id=int(question.get("questionId") or 0),
                question_type=str(question.get("questionType") or ""),
                instance_index=instance_index,
            )
        )
        patches.append(build_game_screen_options_patch(ui_options, disabled=False, instance_index=instance_index))
        # decision_making only accepts answers in WAIT_INPUT (and FAIL_L1); keep input disabled here.
        patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "WAIT_INPUT":
        patches.append(build_game_screen_input_disabled_patch(False, instance_index=instance_index))

    elif state_upper == "FAIL_L1":
        hint = payload.get("hint") or "Inténtalo de nuevo."
        patches.append(build_game_screen_question_patch(hint, [], question_id=0, question_type="fail_l1", instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(False, instance_index=instance_index))

    elif state_upper == "FAIL_L2":
        hint = payload.get("hint") or "Continuemos."
        patches.append(build_game_screen_question_patch(hint, [], question_id=0, question_type="fail_l2", instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "CORRECT":
        feedback = payload.get("feedback", "¡Muy bien!")
        patches.append(build_game_screen_question_patch(feedback, [], question_id=0, question_type="correct", instance_index=instance_index))
        patches.append(build_game_screen_options_patch([], disabled=True, instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "PHASE_COMPLETE":
        phase = payload.get("phase", "")
        if not phase:
            patches.append(build_game_screen_question_patch("¡Fase completada!", [], question_id=0, question_type="phase_complete", instance_index=instance_index))
            patches.append(build_game_screen_options_patch([], disabled=True, instance_index=instance_index))
            patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))
            return patches
        patches.append(
            build_game_screen_question_patch(
                f"¡Fase {phase} completada!", [], question_id=0, question_type="phase_complete", instance_index=instance_index
            )
        )
        patches.append(build_game_screen_options_patch([], disabled=True, instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "P6_HIGHLIGHT":
        highlighted = set(payload.get("highlighted_ids", []))
        options = payload.get("options", [])
        patches.append(build_highlighting_patch(options, highlighted, instance_index=instance_index))
        response = payload.get("response", "Aquííí")
        patches.append(build_game_screen_question_patch(response, [], question_id=0, question_type="p6_highlight", instance_index=instance_index))
        patches.append(build_game_screen_input_disabled_patch(False, instance_index=instance_index))

    return patches
