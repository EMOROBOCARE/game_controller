"""Unit tests for option generation helpers."""

import random

from game_controller.content.option_generation import (
    enrich_question_options,
    generate_color_options,
    generate_generic_options,
    generate_yes_no_options,
)


def test_generate_color_options_basic_count():
    colors = [
        {"label": "rojo", "image": "rojo.svg"},
        {"label": "verde", "image": "verde.svg"},
        {"label": "azul", "image": "azul.svg"},
    ]
    random.seed(0)
    options = generate_color_options(
        "rojo",
        difficulty="basic",
        available_colors=colors,
        shuffle=False,
    )
    assert len(options) == 2
    assert sum(1 for opt in options if opt["correct"]) == 1
    assert any(opt["label"] == "rojo" and opt["correct"] for opt in options)


def test_generate_color_options_unknown_color():
    colors = [
        {"label": "rojo", "image": "rojo.svg"},
        {"label": "verde", "image": "verde.svg"},
    ]
    random.seed(1)
    options = generate_color_options(
        "morado",
        difficulty="basic",
        available_colors=colors,
        shuffle=False,
    )
    correct = next(opt for opt in options if opt["correct"])
    assert correct["label"] == "morado"
    assert correct["imageUrl"] == "assets/color-morado.svg"


def test_generate_yes_no_options_correct_no():
    options = generate_yes_no_options("no")
    assert any(opt["id"] == "no" and opt["correct"] for opt in options)
    assert any(opt["id"] == "si" and not opt["correct"] for opt in options)


def test_generate_generic_options_respects_difficulty():
    items = [
        {"id": "1", "label": "uno", "image": "1.svg"},
        {"id": "2", "label": "dos", "image": "2.svg"},
        {"id": "3", "label": "tres", "image": "3.svg"},
    ]
    random.seed(2)
    options = generate_generic_options(
        items[0],
        items,
        difficulty="basic",
        shuffle=False,
    )
    assert len(options) == 2
    assert options[0]["id"] == "1"
    assert options[0]["correct"] is True


def test_enrich_question_options_yes_no():
    question = {"questionType": "yes_no", "meta": {"correct_answer": "no"}}
    phase_config = {"interactionType": "yes_no"}
    difficulty_config = {"level": "basic"}
    enriched = enrich_question_options(question, phase_config, difficulty_config)
    assert len(enriched["options"]) == 2
    assert any(opt["id"] == "no" and opt["correct"] for opt in enriched["options"])


def test_enrich_question_options_color():
    colors = [
        {"label": "rojo", "image": "rojo.svg"},
        {"label": "verde", "image": "verde.svg"},
    ]
    random.seed(3)
    question = {"questionType": "color", "meta": {"color": "rojo"}}
    phase_config = {"interactionType": "selection"}
    difficulty_config = {"optionsCount": 2}
    enriched = enrich_question_options(
        question,
        phase_config,
        difficulty_config,
        available_colors=colors,
    )
    assert len(enriched["options"]) == 2
    assert any(opt["label"] == "rojo" and opt["correct"] for opt in enriched["options"])
