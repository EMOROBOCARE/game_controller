"""Regression tests for state-to-manifest patching in GameControllerNode."""

from __future__ import annotations

from game_controller.node import GameControllerNode


def _question_payload() -> dict:
    return {
        "question": {
            "questionId": 1,
            "prompt": "Une los colores iguales.",
            "questionType": "multiple_choice",
            "options": [
                {"id": "rojo", "label": "rojo", "imageUrl": "red_circle"},
                {"id": "azul", "label": "azul", "imageUrl": "blue_circle"},
            ],
        }
    }


def _answer_type_value(patches: list[dict]) -> str:
    answer_patch = next(p for p in patches if str(p.get("path", "")).endswith("/answerType"))
    return str(answer_patch.get("value", ""))


def test_patch_ui_for_state_injects_current_phase_when_missing() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P1"
    node._game_screen_index = 0

    patches = GameControllerNode._patch_ui_for_state(node, "QUESTION_PRESENT", _question_payload())

    assert _answer_type_value(patches) == "match"


def test_patch_ui_for_state_keeps_explicit_phase_from_payload() -> None:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_phase = "P1"
    node._game_screen_index = 0

    payload = _question_payload()
    payload["phase"] = "P3"
    patches = GameControllerNode._patch_ui_for_state(node, "QUESTION_PRESENT", payload)

    assert _answer_type_value(patches) == "button"
