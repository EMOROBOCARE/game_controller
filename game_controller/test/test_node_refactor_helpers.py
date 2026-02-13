"""Unit tests for refactor helper behavior in GameControllerNode."""

from __future__ import annotations

from game_controller.node import GameControllerNode


def test_patch_ui_for_state_skips_phase_intro() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P1"
    node._game_screen_index = 0

    patches = GameControllerNode._patch_ui_for_state(node, "PHASE_INTRO", {"phase": "P1"})
    assert patches == []


def test_rephrase_preserves_expected_answer_guard() -> None:
    node = GameControllerNode.__new__(GameControllerNode)

    assert GameControllerNode._rephrase_preserves_expected_answer(node, "Es el color rojo.", "rojo") is True
    assert GameControllerNode._rephrase_preserves_expected_answer(node, "Es el color azul.", "rojo") is False


def test_map_intent_modality_accepts_legacy_strings() -> None:
    node = GameControllerNode.__new__(GameControllerNode)

    assert GameControllerNode._map_intent_modality(node, "__modality_speech__") == "speech"
    assert GameControllerNode._map_intent_modality(node, "__modality_touchscreen__") == "touch"


def test_build_correct_feedback_text_uses_p5_success_response() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P5"
    node._active_game_slug = "colores"
    node._selected_game = "colores"
    node._game_content_cache = {
        "colores": {
            "phaseConfig": {
                "P5": {
                    "successResponse": "Aquííí",
                }
            }
        }
    }

    assert GameControllerNode._build_correct_feedback_text(node, {}) == "Aquííí"


def test_build_correct_feedback_text_uses_correcto_for_p1() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P1"
    node._latest_question_payload = {
        "questionId": 1,
        "options": [
            {"id": "rojo", "label": "Rojo", "correct": True},
            {"id": "azul", "label": "Azul", "correct": False},
        ],
    }

    text = GameControllerNode._build_correct_feedback_text(node, {"correctOptionId": "rojo"})
    assert text == "¡Correcto!"


def test_patch_ui_for_state_correct_p5_keeps_highlighted_option() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P5"
    node._game_screen_index = 0
    node._latest_question_payload = {
        "questionId": 1,
        "options": [
            {"id": "rojo", "label": "rojo", "correct": True},
            {"id": "azul", "label": "azul", "correct": False},
        ],
    }

    patches = GameControllerNode._patch_ui_for_state(
        node,
        "CORRECT",
        {"feedback": "Aquííí"},
    )

    answer_type_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/answerType")
    ][-1]
    options_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/options")
    ][-1]
    assert answer_type_patch["value"] == "button"
    highlighted = options_patch["value"]
    assert highlighted[0]["id"] == "rojo"
    assert highlighted[0]["highlighted"] is True


def test_patch_ui_for_state_correct_p5_infers_target_without_correct_flags() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P5"
    node._game_screen_index = 0
    node._latest_question_payload = {
        "questionId": 1,
        "answer": "rojo",
        "options": [
            {"id": "rojo", "label": "rojo"},
            {"id": "azul", "label": "azul"},
        ],
    }

    patches = GameControllerNode._patch_ui_for_state(
        node,
        "CORRECT",
        {"feedback": "Aquííí"},
    )

    options_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/options")
    ][-1]
    highlighted = options_patch["value"]
    assert highlighted[0]["id"] == "rojo"
    assert highlighted[0]["highlighted"] is True


def test_patch_ui_for_state_wait_input_p5_uses_cached_question_options() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P5"
    node._game_screen_index = 0
    node._latest_question_payload = {
        "questionId": 7,
        "options": [
            {"id": "rojo", "label": "rojo"},
            {"id": "azul", "label": "azul"},
        ],
    }

    patches = GameControllerNode._patch_ui_for_state(
        node,
        "WAIT_INPUT",
        {"roundId": 7, "questionId": 7},
    )

    answer_type_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/answerType")
    ][-1]
    input_disabled_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/inputDisabled")
    ][-1]
    options_patch = [
        patch for patch in patches if str(patch.get("path", "")).endswith("/options")
    ][-1]

    assert answer_type_patch["value"] == "button"
    assert input_disabled_patch["value"] is True
    assert all(bool(opt.get("disabled")) for opt in options_patch["value"])


def test_auto_advance_state_key_uses_p1_correct_override() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P1"

    state_key = GameControllerNode._auto_advance_state_key(
        node,
        "CORRECT",
        {"phase": "P1"},
    )
    assert state_key == "CORRECT_P1"
