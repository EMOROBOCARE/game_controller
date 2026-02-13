"""Unit tests for UI manifest builder helpers (EmorobCare contract)."""

from __future__ import annotations

from game_controller.ui import manifest_builder as mb
from game_controller.ui.manifest_builder import (
    GAME_SCREEN_INSTANCE_ID,
    build_hint_highlight_patches,
    build_game_screen_options_patch,
    build_instance_config_path,
    build_initial_manifest,
    build_state_based_patches,
    format_options_for_ui,
    get_instance_index,
)


def test_build_initial_manifest_structure():
    manifest = build_initial_manifest(
        games=[{"id": "g1", "title": "Game", "image": "assets/cards/g1.png"}],
        users=[{"id": "1", "name": "User"}],
    )

    assert manifest["version"] == 1
    assert "componentRegistry" in manifest
    assert "UserPanel" in manifest["componentRegistry"]
    assert "GameSelector" in manifest["componentRegistry"]
    assert "GameComponent" in manifest["componentRegistry"]
    assert "volume" in manifest["ops"]

    instance_ids = [instance["id"] for instance in manifest["instances"]]
    assert instance_ids == ["game_screen", "user_panel"]
    screen_cfg = manifest["instances"][0]["config"]
    user_panel_cfg = manifest["instances"][1]["config"]
    assert screen_cfg["games"][0]["image"] == "/assets/cards/g1.png"
    assert screen_cfg["volumeOpId"] == "volume"
    assert user_panel_cfg["userIconUrl"] == "/emorobcare-components/images/user.png"


def test_build_game_screen_options_patch_maps_options():
    options = [{"id": "1", "label": "uno"}]

    patch = build_game_screen_options_patch(options)
    items = patch["value"]
    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert patch["path"] == build_instance_config_path(game_screen_index, "options")
    assert items[0]["label"] == "1"
    assert items[0]["text"] == "uno"


def test_format_options_for_ui_maps_fields():
    options = [
        {
            "id": "1",
            "label": "uno",
            "imageUrl": "uno.png",
            "correct": True,
        }
    ]
    ui_options = format_options_for_ui(options)
    assert ui_options == [
        {
            "id": "1",
            "label": "uno",
            "img": "/assets/images/uno.png",
            "disabled": False,
            "hidden": False,
            "highlighted": False,
        }
    ]


def test_build_state_based_patches_question_present():
    payload = {
        "question": {
            "questionId": 11,
            "promptText": "Texto UI",
            "prompt": "Hola",
            "imageUrl": "image.png",
            "questionType": "multiple_choice",
            "options": [
                {
                    "id": "1",
                    "label": "uno",
                    "imageUrl": "uno.png",
                    "correct": True,
                }
            ],
        }
    }
    patches = build_state_based_patches("QUESTION_PRESENT", payload)
    assert len(patches) == 6

    question_patch = patches[0]
    options_patch = patches[1]
    items_patch = patches[2]
    answer_type_patch = patches[3]

    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert question_patch["path"] == build_instance_config_path(game_screen_index, "question")
    assert question_patch["value"]["text"] == "Texto UI"
    assert question_patch["value"]["img"] == ["/assets/images/image.png"]
    assert question_patch["value"]["imgs"] == ["/assets/images/image.png"]
    assert question_patch["value"]["id"] == 11

    patched_options = options_patch["value"]
    assert options_patch["path"] == build_instance_config_path(game_screen_index, "options")
    assert patched_options[0]["img"] == "/assets/images/uno.png"
    assert patched_options[0]["label"] == "1"
    assert patched_options[0]["text"] == "uno"

    patched_items = items_patch["value"]
    assert items_patch["path"] == build_instance_config_path(game_screen_index, "items")
    assert patched_items == patched_options

    assert answer_type_patch["path"] == build_instance_config_path(game_screen_index, "answerType")
    assert answer_type_patch["value"] == "button"


def test_build_state_based_patches_question_present_p1_uses_match():
    payload = {
        "phase": "P1",
        "question": {
            "questionId": 21,
            "prompt": "Une los colores iguales.",
            "imageUrl": "red_circle",
            "questionType": "multiple_choice",
            "options": [
                {"id": "rojo", "label": "rojo", "imageUrl": "red_circle"},
                {"id": "azul", "label": "azul", "imageUrl": "blue_circle"},
            ],
        },
    }
    patches = build_state_based_patches("QUESTION_PRESENT", payload)
    question_patch = patches[0]
    answer_type_patch = patches[3]

    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert question_patch["path"] == build_instance_config_path(game_screen_index, "question")
    assert question_patch["value"]["img"] == []
    assert question_patch["value"]["imgs"] == []
    assert answer_type_patch["path"] == build_instance_config_path(game_screen_index, "answerType")
    assert answer_type_patch["value"] == "match"


def test_build_state_based_patches_question_present_matching_components_alias_uses_match():
    payload = {
        "phase": "matchingComponents",
        "question": {
            "questionId": 22,
            "prompt": "Une los colores iguales.",
            "questionType": "multiple_choice",
            "options": [
                {"id": "rojo", "label": "rojo", "imageUrl": "red_circle"},
                {"id": "azul", "label": "azul", "imageUrl": "blue_circle"},
            ],
        },
    }
    patches = build_state_based_patches("QUESTION_PRESENT", payload)
    answer_type_patch = patches[3]

    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert answer_type_patch["path"] == build_instance_config_path(game_screen_index, "answerType")
    assert answer_type_patch["value"] == "match"


def test_build_state_based_patches_question_present_speech_fallback_options():
    payload = {
        "question": {
            "questionId": 22,
            "prompt": "¿Qué color es?",
            "imageUrl": "red_circle",
            "questionType": "speech",
            "answer": "red",
            "options": [],
        }
    }
    patches = build_state_based_patches("QUESTION_PRESENT", payload)
    options_patch = patches[1]
    answer_type_patch = patches[3]

    assert len(options_patch["value"]) >= 1
    assert options_patch["value"][0]["label"] == "rojo"
    assert options_patch["value"][0]["img"] == "/assets/images/red_circle.png"
    assert answer_type_patch["value"] == "button"


def test_build_state_based_patches_question_present_p2_keeps_no_options():
    payload = {
        "phase": "P2",
        "question": {
            "questionId": 31,
            "promptText": "¿Qué color es?",
            "questionType": "speech",
            "answer": "rojo",
            "options": [],
        },
    }
    patches = build_state_based_patches("QUESTION_PRESENT", payload)
    options_patch = patches[1]
    answer_type_patch = patches[3]

    assert options_patch["value"] == []
    assert answer_type_patch["value"] == "none"


def test_build_state_based_patches_question_present_p6_hides_choices():
    payload = {
        "phase": "P6",
        "question": {
            "questionId": 32,
            "promptText": "¿Es rojo o azul?",
            "questionType": "multiple_choice",
            "answer": "rojo",
            "options": [
                {"id": "rojo", "label": "rojo", "correct": True},
                {"id": "azul", "label": "azul", "correct": False},
            ],
        },
    }
    patches = build_state_based_patches("QUESTION_PRESENT", payload)
    options_patch = patches[1]
    items_patch = patches[2]
    answer_type_patch = patches[3]

    assert options_patch["value"] == []
    assert items_patch["value"] == []
    assert answer_type_patch["value"] == "none"


def test_build_state_based_patches_wait_input_p5_keeps_input_disabled():
    payload = {
        "phase": "P5",
        "question": {
            "options": [
                {"id": "rojo", "label": "rojo", "imageUrl": "red_circle"},
                {"id": "azul", "label": "azul", "imageUrl": "blue_circle"},
            ]
        },
    }
    patches = build_state_based_patches("WAIT_INPUT", payload)
    input_disabled_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/inputDisabled")
    ][-1]
    options_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/options")
    ][-1]
    items_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/items")
    ][-1]

    assert input_disabled_patch["value"] is True
    assert all(bool(opt.get("disabled")) for opt in options_patch["value"])
    assert items_patch["value"] == options_patch["value"]


def test_build_state_based_patches_fail_l1_does_not_replace_question():
    payload = {
        "hint": "Inténtalo de nuevo.",
        "action": "highlight",
        "correctOptionId": "rojo",
    }
    patches = build_state_based_patches("FAIL_L1", payload)
    question_patches = [patch for patch in patches if str(patch.get("path", "")).endswith("/question")]
    input_disabled_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/inputDisabled")
    ][-1]

    assert question_patches == []
    assert input_disabled_patch["value"] is False


def test_build_state_based_patches_p5_highlight():
    payload = {
        "highlighted_ids": ["2"],
        "options": [
            {"id": "1", "label": "uno"},
            {"id": "2", "label": "dos"},
        ],
        "response": "Aqui",
    }
    patches = build_state_based_patches("P5_HIGHLIGHT", payload)
    assert len(patches) == 6

    highlight_patch = patches[0]
    highlight_items_patch = patches[1]
    patched_options = highlight_patch["value"]
    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert highlight_patch["path"] == build_instance_config_path(game_screen_index, "options")
    assert highlight_items_patch["path"] == build_instance_config_path(game_screen_index, "items")
    assert highlight_items_patch["value"] == patched_options
    assert patched_options[0]["label"] == "2"
    assert patched_options[0]["highlighted"] is True


def test_build_hint_highlight_patches_marks_only_correct_option():
    options = [
        {"id": "1", "label": "uno", "imageUrl": "uno.png"},
        {"id": "2", "label": "dos", "imageUrl": "dos.png"},
    ]
    patches = build_hint_highlight_patches(options, "2")
    assert len(patches) == 2

    highlighted = patches[0]["value"]
    assert highlighted[0]["id"] == "1"
    assert highlighted[0].get("highlighted", False) is False
    assert highlighted[1]["id"] == "2"
    assert highlighted[1]["highlighted"] is True


def test_build_hint_highlight_patches_returns_empty_without_match():
    options = [
        {"id": "1", "label": "uno"},
        {"id": "2", "label": "dos"},
    ]
    patches = build_hint_highlight_patches(options, "missing")
    assert patches == []


def test_build_hint_highlight_patches_uses_correct_flag_when_id_missing():
    options = [
        {"id": "1", "label": "uno", "correct": False},
        {"id": "2", "label": "dos", "correct": True},
    ]
    patches = build_hint_highlight_patches(options, None)

    assert len(patches) == 2
    highlighted = patches[0]["value"]
    assert highlighted[0].get("highlighted", False) is False
    assert highlighted[1]["highlighted"] is True


def test_component_registry_remote_entry_from_base_env(monkeypatch):
    monkeypatch.delenv("GAME_CONTROLLER_REMOTE_ENTRY_URL", raising=False)
    monkeypatch.setenv("GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL", "http://localhost:8084/emorobcare-components")
    monkeypatch.setenv("GAME_CONTROLLER_REMOTE_ENTRY_VERSION", "")

    registry = mb.build_component_registry()
    assert registry["UserPanel"]["url"] == "http://localhost:8084/emorobcare-components/assets/remoteEntry.js"


def test_component_registry_remote_entry_falls_back_to_shared_asset_base(monkeypatch):
    monkeypatch.delenv("GAME_CONTROLLER_REMOTE_ENTRY_URL", raising=False)
    monkeypatch.delenv("GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL", raising=False)
    monkeypatch.setenv("SHARED_ASSET_BASE_URL", "http://localhost:8084/emorobcare-components")
    monkeypatch.setenv("GAME_CONTROLLER_REMOTE_ENTRY_VERSION", "")

    registry = mb.build_component_registry()
    assert registry["UserPanel"]["url"] == "http://localhost:8084/emorobcare-components/assets/remoteEntry.js"
