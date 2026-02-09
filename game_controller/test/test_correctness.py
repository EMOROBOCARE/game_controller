"""Unit tests for correctness computation."""

import pytest
from game_controller.content.correctness import (
    normalize_text,
    compute_correct_for_question,
    find_correct_option,
)


class TestNormalizeText:
    """Test text normalization."""
    
    def test_lowercase(self):
        assert normalize_text("ROJO") == "rojo"
    
    def test_strip_whitespace(self):
        assert normalize_text("  rojo  ") == "rojo"
    
    def test_remove_accents(self):
        assert normalize_text("José") == "jose"
        assert normalize_text("niño") == "nino"
        assert normalize_text("sí") == "si"
        assert normalize_text("  Sí  ") == "si"
    
    def test_empty_string(self):
        assert normalize_text("") == ""
        assert normalize_text(None) == ""


class TestComputeCorrect:
    """Test correctness computation."""
    
    def test_color_question_correct(self):
        question = {
            "questionType": "color",
            "answer": "rojo",
            "meta": {"color": "rojo"},
        }
        assert compute_correct_for_question(question, "rojo") is True
        assert compute_correct_for_question(question, "ROJO") is True
    
    def test_color_question_incorrect(self):
        question = {
            "questionType": "color",
            "answer": "rojo",
            "meta": {"color": "rojo"},
        }
        assert compute_correct_for_question(question, "verde") is False
    
    def test_yes_no_question(self):
        question = {
            "questionType": "yes_no",
            "meta": {"correct_answer": "si"},
        }
        assert compute_correct_for_question(question, "si") is True
        assert compute_correct_for_question(question, "sí") is True
        assert compute_correct_for_question(question, "no") is False
    
    def test_accepted_answers(self):
        question = {
            "meta": {
                "accepted_answers": ["rojo", "verde", "azul", None],
            },
        }
        assert compute_correct_for_question(question, "rojo") is True
        assert compute_correct_for_question(question, "verde") is True
        assert compute_correct_for_question(question, "ÁZUL") is True
        assert compute_correct_for_question(question, "amarillo") is False
    
    def test_options_with_correct(self):
        question = {
            "questionType": "multiple_choice",
            "options": [
                {"id": "1", "label": "rojo", "correct": True},
                {"id": "2", "label": "verde", "correct": False},
            ],
        }
        assert compute_correct_for_question(question, "rojo") is True
        assert compute_correct_for_question(question, "verde") is False
        assert compute_correct_for_question(question, "1") is True

    def test_multiple_choice_no_match(self):
        question = {
            "questionType": "multiple_choice",
            "options": [
                {"id": "1", "label": "rojo", "correct": True},
                {"id": "2", "label": "verde", "correct": False},
            ],
        }
        assert compute_correct_for_question(question, "2") is False

    def test_multiple_choice_options_without_correct_flag(self):
        """Options from decision_making may lack correct flag; fallback to answer."""
        question = {
            "questionType": "multiple_choice",
            "answer": "blue",
            "options": [
                {"id": "red", "label": "red", "imageUrl": "red_circle"},
                {"id": "blue", "label": "blue", "imageUrl": "blue_circle"},
            ],
        }
        assert compute_correct_for_question(question, "blue") is True
        assert compute_correct_for_question(question, "red") is False

    def test_yes_no_fallback_answer(self):
        question = {
            "questionType": "yes_no",
            "answer": "si",
        }
        assert compute_correct_for_question(question, "si") is True
        assert compute_correct_for_question(question, "no") is False

    def test_yes_no_from_options(self):
        question = {
            "questionType": "yes_no",
            "options": [
                {"label": "sí", "correct": True},
                {"label": "no", "correct": False},
            ],
        }
        assert compute_correct_for_question(question, "si") is True

    def test_tracing_completion(self):
        question = {"questionType": "tracing"}
        assert compute_correct_for_question(question, "completed") is True
        assert compute_correct_for_question(question, "nope") is None

    def test_default_candidates(self):
        question = {
            "answer_text": "rojo",
            "options": [{"label": "verde", "correct": True}],
            "meta": {"color": "azul"},
        }
        assert compute_correct_for_question(question, "verde") is True
        assert compute_correct_for_question(question, "azul") is True

    def test_no_candidates(self):
        question = {"questionType": "freeform"}
        assert compute_correct_for_question(question, "rojo") is None

    def test_non_dict_meta(self):
        question = {
            "questionType": "color",
            "meta": "oops",
            "answer": "rojo",
        }
        assert compute_correct_for_question(question, "rojo") is True
    
    def test_none_value(self):
        question = {"answer": "rojo"}
        assert compute_correct_for_question(question, None) is None
    
    def test_empty_value(self):
        question = {"answer": "rojo"}
        assert compute_correct_for_question(question, "") is None


class TestFindCorrectOption:
    """Test finding correct option."""
    
    def test_find_correct(self):
        options = [
            {"id": "1", "label": "rojo", "correct": True},
            {"id": "2", "label": "verde", "correct": False},
        ]
        result = find_correct_option(options)
        assert result is not None
        assert result["label"] == "rojo"
    
    def test_no_correct(self):
        options = [
            {"id": "1", "label": "rojo", "correct": False},
            {"id": "2", "label": "verde", "correct": False},
        ]
        result = find_correct_option(options)
        assert result is None

    def test_is_correct_variant_key(self):
        options = [
            {"id": "1", "label": "rojo", "isCorrect": True},
            {"id": "2", "label": "verde", "isCorrect": False},
        ]
        result = find_correct_option(options)
        assert result is not None
        assert result["label"] == "rojo"
    
    def test_empty_options(self):
        result = find_correct_option([])
        assert result is None
