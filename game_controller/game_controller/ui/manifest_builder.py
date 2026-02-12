"""Generic UI manifest builder.

This controller publishes manifests for two runtime components:
- `GameSelector` for menu/start flow
- `GameComponent` for active gameplay
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Set


REMOTE_ENTRY_URL_ENV = "GAME_CONTROLLER_REMOTE_ENTRY_URL"
REMOTE_ENTRY_BASE_ENV = "GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL"
REMOTE_ENTRY_BASE_URL = "/emorobcare-components/assets/remoteEntry.js"
REMOTE_ENTRY_RELATIVE_PATH = "assets/remoteEntry.js"
REMOTE_ENTRY_VERSION_ENV = "GAME_CONTROLLER_REMOTE_ENTRY_VERSION"
REMOTE_ENTRY_DEFAULT_VERSION = "20260210"
REMOTE_SCOPE = "demo"
GAME_SCREEN_INSTANCE_ID = "game_screen"
DEFAULT_GAME_SCREEN_INDEX = 0
# Backward-compatibility alias: historically used for all rewritten assets.
ASSET_CDN_ENV = "ASSET_CDN_URL"
PROJECT_ASSET_BASE_ENV = "PROJECT_ASSET_BASE_URL"
PROJECT_ASSET_BASE_COMPOSE_ENV = "EMOROBCARE_PROJECT_ASSET_BASE_URL"
PROJECT_ASSET_BASE_DEFAULT = "/assets"
SHARED_ASSET_BASE_ENV = "SHARED_ASSET_BASE_URL"
SHARED_ASSET_BASE_DEFAULT = "/emorobcare-components"
PROJECT_ASSET_IMAGE_DIR_ENV = "PROJECT_ASSET_IMAGE_DIR"
PROJECT_ASSET_IMAGE_DIR_DEFAULT = "images"
PROJECT_ASSET_ID_EXTENSION_ENV = "PROJECT_ASSET_ID_EXTENSION"
PROJECT_ASSET_ID_EXTENSION_DEFAULT = "png"
INCLUDE_CORRECT_ENV = "GAME_CONTROLLER_INCLUDE_CORRECT_OPTIONS"
UI_TRACE_OP_ID = "ui_trace"
# Use an empty message type to absorb bubbling click/select events from nested
# UI buttons without affecting game logic.
UI_TRACE_ROS_TYPE = "std_msgs/msg/Empty"

# Default controls dict with all buttons hidden (menu state).
CONTROLS_HIDDEN: Dict[str, bool] = {
    "showPause": False,
    "showResume": False,
    "showStop": False,
    "showReset": False,
    "showSkipPhase": False,
}

# Controls during active gameplay (not paused).
CONTROLS_PLAYING: Dict[str, bool] = {
    "showPause": True,
    "showResume": False,
    "showStop": True,
    "showReset": True,
    "showSkipPhase": True,
}

# Controls while paused.
CONTROLS_PAUSED: Dict[str, bool] = {
    "showPause": False,
    "showResume": True,
    "showStop": True,
    "showReset": True,
    "showSkipPhase": True,
}

# Minimal default users list for the UserPanel menu component.
DEFAULT_USERS = [
    {"id": "1", "name": "Pepe"},
    {"id": "2", "name": "María"},
    {"id": "3", "name": "Juan"},
]


def _remote_entry_url() -> str:
    """Build remoteEntry URL with a cache-busting version query."""
    base_url = _resolve_remote_entry_url()
    version = str(os.environ.get(REMOTE_ENTRY_VERSION_ENV, REMOTE_ENTRY_DEFAULT_VERSION)).strip()
    if not version:
        return base_url
    sep = "&" if "?" in base_url else "?"
    return f"{base_url}{sep}v={version}"


def _looks_like_remote_entry_url(url: str) -> bool:
    lowered = str(url or "").strip().lower()
    return lowered.endswith("remoteentry.js") or "remoteentry.js?" in lowered


def _resolve_remote_entry_url() -> str:
    """Resolve the component remote entry URL from env configuration.

    Resolution order:
    1. GAME_CONTROLLER_REMOTE_ENTRY_URL (full URL/path or base URL)
    2. GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL (base URL/path)
    3. SHARED_ASSET_BASE_URL + /assets/remoteEntry.js
    """
    explicit = str(os.environ.get(REMOTE_ENTRY_URL_ENV, "")).strip()
    if explicit:
        if _looks_like_remote_entry_url(explicit):
            return explicit
        return _join_asset(_clean_base(explicit, explicit), REMOTE_ENTRY_RELATIVE_PATH)

    remote_base = str(os.environ.get(REMOTE_ENTRY_BASE_ENV, "")).strip()
    if remote_base:
        return _join_asset(_clean_base(remote_base, remote_base), REMOTE_ENTRY_RELATIVE_PATH)

    return _join_asset(_shared_asset_base(), REMOTE_ENTRY_RELATIVE_PATH)


def build_component_registry() -> Dict[str, Dict[str, Any]]:
    """Build the component registry for the game_controller runtime surface."""
    remote_entry_url = _remote_entry_url()
    return {
        "UserPanel": {
            "url": remote_entry_url,
            "scope": REMOTE_SCOPE,
            "module": "./UserPanel",
        },
        "GameSelector": {
            "url": remote_entry_url,
            "scope": REMOTE_SCOPE,
            "module": "./GameSelector",
        },
        "GameComponent": {
            "url": remote_entry_url,
            "scope": REMOTE_SCOPE,
            "module": "./GameComponent",
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
        UI_TRACE_OP_ID: {
            "kind": "topic_pub",
            "rosType": UI_TRACE_ROS_TYPE,
            "topic": "/ui/tracing",
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
    """Build menu layout (UserPanel + GameSelector)."""
    return {
        "mode": "grid",
        "columns": 12,
        "items": [
            {"instanceId": "user_panel", "col": 1, "row": 1, "colSpan": 12, "rowSpan": 1},
            {"instanceId": GAME_SCREEN_INSTANCE_ID, "col": 1, "row": 2, "colSpan": 12, "rowSpan": 6},
        ],
    }


def build_game_layout() -> Dict[str, Any]:
    """Build gameplay layout.

    Kept as a compatibility alias; gameplay now uses the same 2-instance layout
    as menu mode (`user_panel` + `game_screen`).
    """
    return build_root_layout()


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
        # Avoid remote-bundle-relative icon path resolution issues on host shell.
        "userIconUrl": "/assets/user.png",
    }
    if active_user is not None:
        user_panel_config["activeUser"] = active_user

    return [
        {
            "id": "user_panel",
            "component": "UserPanel",
            "config": user_panel_config,
            "capabilities": ["user_selector"],
            "bindings": {"user_selector": "user_selector"},
        },
    ]


def build_selector_config(
    games: Optional[List[Dict[str, Any]]] = None,
    username: str = "",
    round_tx: int = 0,
    answer_op_id: str = "ui_input",
) -> Dict[str, Any]:
    """Build menu config for GameSelector.

    Includes compatibility fields so transient component swaps stay stable.
    """
    if games is None:
        games = []
    games_for_ui = _normalize_games_for_ui(games)
    return {
        "mode": "menu",
        "startGameOpId": "game_selector",
        "uiInputOpId": answer_op_id,
        "games": games_for_ui,
        "username": username,
        "round": int(round_tx),
        "inputDisabled": False,
        "controls": dict(CONTROLS_HIDDEN),
        # Compatibility defaults with GameComponent shape.
        "effect": "none",
        "question": {"id": 0, "text": "", "img": [], "imgs": []},
        "answerOpId": answer_op_id,
        "answerType": "none",
        "items": [],
        "options": [],
        "pause": False,
        "gameFlowOpId": answer_op_id,
        "volumeOpId": answer_op_id,
        # Telemetry extras used by controller.
        "state": {"system": "IDLE", "gameState": None, "sessionId": None, "transactionId": int(round_tx)},
        "phase": "",
    }


def build_game_config(
    question_text: str = "",
    images: Optional[List[str]] = None,
    items: Optional[List[Dict[str, Any]]] = None,
    answer_type: str = "none",
    effect: str = "none",
    pause: bool = False,
    answer_op_id: str = "ui_input",
    games: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Build gameplay config for GameComponent.

    Includes selector-compatible keys to avoid transient swap crashes.
    """
    if images is None:
        images = []
    if items is None:
        items = []
    if games is None:
        games = []
    games_for_ui = _normalize_games_for_ui(games)
    return {
        "mode": "game",
        # Compatibility defaults with GameSelector shape.
        "startGameOpId": "game_selector",
        "uiInputOpId": answer_op_id,
        "games": games_for_ui,
        "username": "",
        "round": 0,
        "inputDisabled": True,
        "controls": dict(CONTROLS_PLAYING),
        # GameComponent fields.
        "effect": effect,
        "question": {
            "id": 0,
            "text": str(question_text or ""),
            "img": list(images),
            "imgs": list(images),
        },
        "answerOpId": answer_op_id,
        "answerType": str(answer_type or "none"),
        "items": list(items),
        "options": list(items),
        "pause": bool(pause),
        "gameFlowOpId": answer_op_id,
        "volumeOpId": answer_op_id,
        # Telemetry extras used by controller.
        "state": {"system": "GAME", "gameState": None, "sessionId": None, "transactionId": 0},
        "phase": "",
    }


def build_game_screen_instance(
    games: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Build the main game view instance (starts in selector mode)."""
    return {
        "id": GAME_SCREEN_INSTANCE_ID,
        "component": "GameSelector",
        "config": build_selector_config(games=games),
        "capabilities": ["ui_input", "game_selector", "decision_state", "ui_update", UI_TRACE_OP_ID],
        "bindings": {
            # GameSelector emits the op-id as the event name.
            "game_selector": "game_selector",
            # Keep semantic aliases for compatibility.
            "startGame": "game_selector",
            "ui_input": "ui_input",
            "answer": "ui_input",
            "gameFlow": "ui_input",
            "volume": "ui_input",
            # Silence unsupported bubbling events from nested Button components.
            "click": UI_TRACE_OP_ID,
            "select": UI_TRACE_OP_ID,
        },
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
        "layout": build_root_layout(),
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


def build_instance_config_root_path(instance_index: int) -> str:
    """Build JSON patch path for replacing full instance config."""
    return f"/instances/{int(instance_index)}/config"


def build_instance_component_path(instance_index: int) -> str:
    """Build JSON patch path for instance component."""
    return f"/instances/{int(instance_index)}/component"


def _clean_base(raw: str, default: str) -> str:
    value = str(raw or "").strip().rstrip("/")
    if value:
        return value
    return default.rstrip("/")


def _project_asset_base() -> str:
    # Maintain backward compatibility with ASSET_CDN_URL while preferring the
    # explicit project asset base variable.
    raw = (
        os.environ.get(PROJECT_ASSET_BASE_ENV)
        or os.environ.get(PROJECT_ASSET_BASE_COMPOSE_ENV)
        or os.environ.get(ASSET_CDN_ENV)
        or PROJECT_ASSET_BASE_DEFAULT
    )
    return _clean_base(str(raw), PROJECT_ASSET_BASE_DEFAULT)


def _shared_asset_base() -> str:
    raw = os.environ.get(SHARED_ASSET_BASE_ENV, SHARED_ASSET_BASE_DEFAULT)
    return _clean_base(str(raw), SHARED_ASSET_BASE_DEFAULT)


def _join_asset(base: str, suffix: str) -> str:
    clean = str(suffix or "").strip().lstrip("/")
    return f"{base}/{clean}" if clean else base


def _rewrite_asset_id(asset_id: str) -> str:
    base = _project_asset_base()
    image_dir = str(
        os.environ.get(PROJECT_ASSET_IMAGE_DIR_ENV, PROJECT_ASSET_IMAGE_DIR_DEFAULT)
    ).strip().strip("/")
    ext = str(
        os.environ.get(PROJECT_ASSET_ID_EXTENSION_ENV, PROJECT_ASSET_ID_EXTENSION_DEFAULT)
    ).strip().lstrip(".")
    target = asset_id
    if "." not in target:
        target = f"{target}.{ext or PROJECT_ASSET_ID_EXTENSION_DEFAULT}"

    # Avoid duplicating the image directory when base already points there,
    # e.g. base ".../images" + symbolic id should become ".../images/foo.png".
    if image_dir and base.endswith(f"/{image_dir}"):
        path = target
    else:
        path = f"{image_dir}/{target}" if image_dir else target
    return _join_asset(base, path)


def rewrite_asset_url(path: Optional[str]) -> str:
    """Rewrite asset paths to browser-reachable URLs."""
    if not path:
        return ""
    raw = str(path).strip()
    if not raw:
        return ""
    lowered = raw.lower()
    if lowered.startswith(("http://", "https://", "data:", "blob:")):
        return raw
    if raw.startswith("/"):
        return raw
    if raw.startswith("assets/"):
        return _join_asset(_project_asset_base(), raw[len("assets/") :])
    if raw.startswith("shared/"):
        return _join_asset(_shared_asset_base(), raw[len("shared/") :])
    if raw.startswith(("images/", "fonts/")):
        return _join_asset(_shared_asset_base(), raw)
    if "/" not in raw:
        return _rewrite_asset_id(raw)
    return _join_asset(_project_asset_base(), raw)


def _normalize_games_for_ui(games: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    normalized: List[Dict[str, Any]] = []
    for game in games:
        if not isinstance(game, dict):
            continue
        item = dict(game)
        image = game.get("image")
        if isinstance(image, str):
            item["image"] = rewrite_asset_url(image)
        normalized.append(item)
    return normalized


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
    single = question.get("imageUrl") or question.get("image_url") or question.get("img")
    if isinstance(single, str) and single:
        images.append(single)

    # decision_making may provide both imageUrl and images/imageUrls with identical values.
    # Deduplicate while preserving order to keep UI and integration checks deterministic.
    normalized: List[str] = []
    seen: Set[str] = set()
    for img in images:
        if not img:
            continue
        rewritten = rewrite_asset_url(img)
        if rewritten in seen:
            continue
        seen.add(rewritten)
        normalized.append(rewritten)
    return normalized


def _include_correct_in_ui_options(include_correct: Optional[bool]) -> bool:
    if include_correct is not None:
        return include_correct
    env = os.environ.get(INCLUDE_CORRECT_ENV, "")
    return env.strip().lower() in {"1", "true", "yes", "on"}


def build_game_screen_mode_patches(
    mode: str,
    games: Optional[List[Dict[str, Any]]] = None,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> List[Dict[str, Any]]:
    """Switch game view between selector and gameplay components."""
    if mode == "menu":
        selector_cfg = build_selector_config(games=games)
        return [
            {"op": "replace", "path": build_instance_config_root_path(instance_index), "value": selector_cfg},
            {"op": "replace", "path": build_instance_component_path(instance_index), "value": "GameSelector"},
        ]

    game_cfg = build_game_config(games=games)
    return [
        {"op": "replace", "path": build_instance_config_root_path(instance_index), "value": game_cfg},
        {"op": "replace", "path": build_instance_component_path(instance_index), "value": "GameComponent"},
    ]


def _question_payload_value(
    text: str,
    images: Optional[List[str]],
    question_id: int = 0,
    question_type: str = "",
) -> Dict[str, Any]:
    normalized_images = list(images or [])
    val: Dict[str, Any] = {
        "id": int(question_id or 0),
        "text": str(text or ""),
        # Keep both keys for compatibility with older/newer GameComponent builds.
        "img": normalized_images,
        "imgs": list(normalized_images),
    }
    if question_type:
        val["questionType"] = question_type
    return val


def build_game_screen_state_patch(
    system_state: str,
    game_state: Optional[str],
    session_id: Optional[int],
    transaction_id: int,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch embedded state snapshot (telemetry for UI/config consumers)."""
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


def build_game_screen_pause_patch(
    paused: bool,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch pause state for GameComponent settings tray."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "pause"),
        "value": bool(paused),
    }


def build_input_disabled_patch(
    disabled: bool,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch inputDisabled flag for the game screen."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "inputDisabled"),
        "value": bool(disabled),
    }


def build_controls_patch(
    controls: Dict[str, bool],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch controls visibility for the game screen."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "controls"),
        "value": dict(controls),
    }


def build_game_screen_question_patch(
    text: str,
    images: Optional[List[str]] = None,
    question_id: int = 0,
    question_type: str = "",
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch gameplay question payload."""
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "question"),
        "value": _question_payload_value(text, images, question_id=question_id, question_type=question_type),
    }


def _normalize_item_for_game_component(opt: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if bool(opt.get("hidden")):
        return None

    item_id = str(opt.get("id") or opt.get("label") or "").strip()
    if not item_id:
        return None

    image = opt.get("img") or opt.get("imageUrl") or opt.get("image_url")
    image_val = rewrite_asset_url(str(image)) if image else ""

    item: Dict[str, Any] = {
        # Keep both id and label for compatibility with integration harness and UI components.
        "id": item_id,
        "label": item_id,
    }
    if image_val:
        item["img"] = image_val

    text = opt.get("label")
    if text is not None:
        item["text"] = str(text)

    if bool(opt.get("highlighted")):
        item["highlighted"] = True

    return item


def _normalized_items_for_component(options: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [
        item
        for item in (_normalize_item_for_game_component(dict(opt)) for opt in options if isinstance(opt, dict))
        if item is not None
    ]


def build_game_screen_options_patch(
    options: List[Dict[str, Any]],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch gameplay selectable options for GameComponent."""
    items = _normalized_items_for_component(options)
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "options"),
        "value": items,
    }


def build_game_screen_items_patch(
    options: List[Dict[str, Any]],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Patch gameplay items for GameComponent builds that read config.items."""
    items = _normalized_items_for_component(options)
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "items"),
        "value": items,
    }


def build_game_screen_answer_type_patch(
    answer_type: str,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "answerType"),
        "value": str(answer_type or "none"),
    }


def build_game_screen_effect_patch(
    effect: str,
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    return {
        "op": "replace",
        "path": build_instance_config_path(instance_index, "effect"),
        "value": str(effect or "none"),
    }


def build_highlighting_patch(
    options: List[Dict[str, Any]],
    highlighted_ids: Set[str],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Build a patch for P5 pointing phase highlighting."""
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
    return build_game_screen_options_patch(patched, instance_index=instance_index)


def build_highlighting_items_patch(
    options: List[Dict[str, Any]],
    highlighted_ids: Set[str],
    instance_index: int = DEFAULT_GAME_SCREEN_INDEX,
) -> Dict[str, Any]:
    """Build a config.items patch for P5 pointing phase highlighting."""
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
    return build_game_screen_items_patch(patched, instance_index=instance_index)


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


def _fallback_options_from_question(question: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Build minimal clickable options when decision payload has none.

    This is mainly used for speech-like prompts in WebUI flows where touch input
    is required to keep the game session debuggable end-to-end.
    """
    meta = question.get("meta")
    meta_dict = meta if isinstance(meta, dict) else {}

    raw_answer = (
        question.get("answer")
        or meta_dict.get("correct_answer")
        or meta_dict.get("answer")
    )
    answer = str(raw_answer or "").strip()
    if not answer:
        return []

    answer_lower = answer.lower()
    yes_aliases = {"si", "sí", "yes", "true", "1"}
    no_aliases = {"no", "false", "0"}
    if answer_lower in yes_aliases or answer_lower in no_aliases:
        return [
            {
                "id": "si",
                "label": "Sí",
                "correct": answer_lower in yes_aliases,
            },
            {
                "id": "no",
                "label": "No",
                "correct": answer_lower in no_aliases,
            },
        ]

    colour_aliases = {
        "red": "rojo",
        "yellow": "amarillo",
        "blue": "azul",
        "green": "verde",
        "rojo": "rojo",
        "amarillo": "amarillo",
        "azul": "azul",
        "verde": "verde",
    }

    display_answer = colour_aliases.get(answer_lower, answer)
    display_answer_lower = display_answer.lower()

    # Keep color prompts interactive even when the payload omits explicit options.
    color_candidates = [
        "rojo",
        "amarillo",
        "azul",
        "verde",
    ]
    distractor = next((c for c in color_candidates if c.lower() != display_answer_lower), None)
    image = question.get("imageUrl") or question.get("image_url") or question.get("img")
    options: List[Dict[str, Any]] = [
        {
            "id": display_answer,
            "label": display_answer,
            "imageUrl": image or "",
            "correct": True,
        }
    ]
    if distractor:
        options.append({"id": distractor, "label": distractor, "correct": False})
    return options


def _answer_type_for_payload(phase: str, question_type: str, options: List[Dict[str, Any]]) -> str:
    if not options:
        return "none"
    phase_code = str(phase or "").upper()
    phase_code_alnum = "".join(ch for ch in phase_code if ch.isalnum())
    # P1 is the matching/association phase and should render MatchingPhase.
    if phase_code_alnum in {"P1", "MATCHING", "ASSOCIATION", "MATCHINGCOMPONENTS"}:
        return "match"
    q_type = str(question_type or "").lower()
    if q_type in {"phase_intro", "phase_complete", "correct", "fail_l1", "fail_l2"}:
        return "none"
    return "button"


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
        patches.append(build_game_screen_options_patch([], instance_index=instance_index))
        patches.append(build_game_screen_items_patch([], instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch("none", instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "ROUND_SETUP":
        patches.append(build_game_screen_question_patch("Preparando...", [], question_id=0, instance_index=instance_index))
        patches.append(build_game_screen_options_patch([], instance_index=instance_index))
        patches.append(build_game_screen_items_patch([], instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch("none", instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "QUESTION_PRESENT":
        question = payload.get("question", {})
        prompt = question.get("prompt") or question.get("text") or ""
        images = _normalize_question_images(question)
        options = question.get("options", [])
        if not isinstance(options, list):
            options = []
        if not options:
            options = _fallback_options_from_question(question)
        ui_options = format_options_for_ui(options)
        answer_type = _answer_type_for_payload(payload.get("phase", ""), question.get("questionType", ""), ui_options)
        patches.append(
            build_game_screen_question_patch(
                str(prompt),
                images,
                question_id=int(question.get("questionId") or question.get("id") or 0),
                instance_index=instance_index,
            )
        )
        patches.append(build_game_screen_options_patch(ui_options, instance_index=instance_index))
        patches.append(build_game_screen_items_patch(ui_options, instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch(answer_type, instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "WAIT_INPUT":
        # Keep question/options from QUESTION_PRESENT; enable input.
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(False, instance_index=instance_index))

    elif state_upper == "FAIL_L1":
        hint = payload.get("hint") or "Inténtalo de nuevo."
        patches.append(build_game_screen_question_patch(str(hint), [], question_id=0, instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(False, instance_index=instance_index))

    elif state_upper == "FAIL_L2":
        hint = payload.get("hint") or "Continuemos."
        patches.append(build_game_screen_question_patch(str(hint), [], question_id=0, instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch("none", instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "CORRECT":
        feedback = payload.get("feedback", "¡Muy bien!")
        patches.append(build_game_screen_question_patch(str(feedback), [], question_id=0, instance_index=instance_index))
        patches.append(build_game_screen_options_patch([], instance_index=instance_index))
        patches.append(build_game_screen_items_patch([], instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch("none", instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("confetti", instance_index=instance_index))
        patches.append(build_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "PHASE_COMPLETE":
        phase = payload.get("phase", "")
        text = f"¡Fase {phase} completada!" if phase else "¡Fase completada!"
        patches.append(build_game_screen_question_patch(text, [], question_id=0, question_type="phase_complete", instance_index=instance_index))
        patches.append(build_game_screen_options_patch([], instance_index=instance_index))
        patches.append(build_game_screen_items_patch([], instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch("none", instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(True, instance_index=instance_index))

    elif state_upper == "P5_HIGHLIGHT":
        highlighted = set(payload.get("highlighted_ids", []))
        options = payload.get("options", [])
        patches.append(build_highlighting_patch(options, highlighted, instance_index=instance_index))
        patches.append(build_highlighting_items_patch(options, highlighted, instance_index=instance_index))
        response = payload.get("response", "Aquííí")
        patches.append(build_game_screen_question_patch(str(response), [], question_id=0, instance_index=instance_index))
        patches.append(build_game_screen_answer_type_patch("button", instance_index=instance_index))
        patches.append(build_game_screen_effect_patch("none", instance_index=instance_index))
        patches.append(build_input_disabled_patch(False, instance_index=instance_index))

    return patches
