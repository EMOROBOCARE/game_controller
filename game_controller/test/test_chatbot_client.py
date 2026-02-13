"""Unit tests for chatbot client helpers."""

from game_controller.chatbot_client import normalize_evaluate_label


def test_normalize_evaluate_label_correct_variants():
    assert normalize_evaluate_label("AC") is True
    assert normalize_evaluate_label("answer-correct") is True
    assert normalize_evaluate_label("true") is True


def test_normalize_evaluate_label_incorrect_variants():
    assert normalize_evaluate_label("AI") is False
    assert normalize_evaluate_label("answer-incorrect") is False
    assert normalize_evaluate_label("false") is False


def test_normalize_evaluate_label_unknown():
    assert normalize_evaluate_label("") is None
    assert normalize_evaluate_label("maybe") is None
