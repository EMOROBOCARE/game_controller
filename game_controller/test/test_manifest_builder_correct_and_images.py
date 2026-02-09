"""Unit tests for manifest option correctness flags and image normalization."""

from __future__ import annotations

from game_controller.ui import manifest_builder as mb


def test_format_options_include_correct_success():
    options = [
        {"id": "1", "label": "uno", "correct": True},
        {"id": "2", "label": "dos", "correct": False},
    ]

    ui_options = mb.format_options_for_ui(options, include_correct=True)

    assert ui_options[0]["correct"] is True
    assert ui_options[1]["correct"] is False


def test_format_options_excludes_correct_failure_case():
    options = [
        {"id": "1", "label": "uno", "correct": True},
    ]

    ui_options = mb.format_options_for_ui(options, include_correct=False)

    assert "correct" not in ui_options[0]


def test_question_present_multi_image_edge_case(monkeypatch):
    monkeypatch.setenv("ASSET_CDN_URL", "http://cdn.test/base")

    payload = {
        "question": {
            "questionId": 7,
            "prompt": "¿Dónde está?",
            "questionType": "multiple_choice",
            "images": [
                {"src": "assets/images/a.png"},
                {"url": "images/b.png"},
            ],
            "imageUrl": "assets/images/c.png",
            "options": [],
        }
    }

    patches = mb.build_state_based_patches("QUESTION_PRESENT", payload)
    question_patch = patches[0]

    assert question_patch["value"]["imgs"] == [
        "http://cdn.test/base/images/a.png",
        "images/b.png",
        "http://cdn.test/base/images/c.png",
    ]
