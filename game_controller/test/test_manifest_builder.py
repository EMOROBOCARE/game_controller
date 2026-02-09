"""Unit tests for UI manifest builder helpers (EmorobCare contract)."""

from __future__ import annotations

from game_controller.ui.manifest_builder import (
    GAME_SCREEN_INSTANCE_ID,
    build_game_screen_options_patch,
    build_instance_config_path,
    build_initial_manifest,
    build_state_based_patches,
    format_options_for_ui,
    get_instance_index,
)


def test_build_initial_manifest_structure():
    manifest = build_initial_manifest(
        games=[{"id": "g1", "title": "Game"}],
        users=[{"id": "1", "name": "User"}],
    )

    assert manifest["version"] == 1
    assert "componentRegistry" in manifest
    assert "UserPanel" in manifest["componentRegistry"]
    assert "GameScreenComponent" in manifest["componentRegistry"]

    instance_ids = [instance["id"] for instance in manifest["instances"]]
    assert instance_ids == ["game_screen", "user_panel"]


def test_build_game_screen_options_patch_disabled():
    options = [{"id": "1", "label": "uno"}]

    disabled_patch = build_game_screen_options_patch(options, disabled=True)
    disabled_options = disabled_patch["value"]
    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert disabled_patch["path"] == build_instance_config_path(game_screen_index, "options")
    assert disabled_options[0]["disabled"] is True

    enabled_patch = build_game_screen_options_patch(options, disabled=False)
    enabled_options = enabled_patch["value"]
    assert enabled_options[0]["disabled"] is False


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
            "img": "uno.png",
            "disabled": False,
            "hidden": False,
            "highlighted": False,
        }
    ]


def test_build_state_based_patches_question_present():
    payload = {
        "question": {
            "questionId": 11,
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
    assert len(patches) == 3

    question_patch = patches[0]
    options_patch = patches[1]
    input_disabled_patch = patches[2]

    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert question_patch["path"] == build_instance_config_path(game_screen_index, "question")
    assert question_patch["value"]["text"] == "Hola"
    assert question_patch["value"]["imgs"] == ["image.png"]
    assert question_patch["value"]["questionId"] == 11
    assert question_patch["value"]["questionType"] == "multiple_choice"

    patched_options = options_patch["value"]
    assert options_patch["path"] == build_instance_config_path(game_screen_index, "options")
    assert patched_options[0]["img"] == "uno.png"
    assert patched_options[0]["disabled"] is False

    assert input_disabled_patch["path"] == build_instance_config_path(game_screen_index, "inputDisabled")
    assert input_disabled_patch["value"] is True


def test_build_state_based_patches_p6_highlight():
    payload = {
        "highlighted_ids": ["2"],
        "options": [
            {"id": "1", "label": "uno"},
            {"id": "2", "label": "dos"},
        ],
        "response": "Aqui",
    }
    patches = build_state_based_patches("P6_HIGHLIGHT", payload)
    assert len(patches) == 3

    highlight_patch = patches[0]
    patched_options = highlight_patch["value"]
    manifest = build_initial_manifest()
    game_screen_index = get_instance_index(manifest["instances"], GAME_SCREEN_INSTANCE_ID)
    assert highlight_patch["path"] == build_instance_config_path(game_screen_index, "options")
    assert patched_options[0]["hidden"] is True
    assert patched_options[0]["highlighted"] is False
    assert patched_options[1]["hidden"] is False
    assert patched_options[1]["highlighted"] is True
