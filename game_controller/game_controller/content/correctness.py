"""Correctness computation for user answers.

Ported from refactored_game/game_manager/app/main.py
"""

from __future__ import annotations

import unicodedata
from typing import Any, Dict, List, Optional, Set


def normalize_text(value: str) -> str:
    """Normalize text for comparison.
    
    - Lowercase
    - Strip whitespace
    - Remove diacritics (accents)
    """
    text = (value or "").lower().strip()
    # Remove accents/diacritics
    return "".join(
        c for c in unicodedata.normalize("NFD", text)
        if unicodedata.category(c) != "Mn"
    )


def _get_field(data: Dict[str, Any], *names: str, default: Any = None) -> Any:
    """Get field from dict, trying multiple name variants."""
    if not isinstance(data, dict):
        return default
    for name in names:
        if name in data:
            return data.get(name, default)
    return default


def compute_correct_for_question(
    question: Dict[str, Any],
    raw_value: str | None,
) -> Optional[bool]:
    """Compute correctness of a user answer for a question.
    
    Args:
        question: Question dict with answer, options, meta, etc.
        raw_value: The user's raw answer text
        
    Returns:
        True if correct, False if incorrect, None if cannot determine
    """
    if raw_value is None:
        return None
    
    value = normalize_text(str(raw_value))
    if not value:
        return None
    
    # Get question fields
    question_type = (_get_field(question, "questionType", "question_type", default="") or "").lower()
    meta = _get_field(question, "meta", "metadata", default={}) or {}
    if not isinstance(meta, dict):
        meta = {}
    
    # 1. Check explicit accepted_answers from meta
    accepted = meta.get("accepted_answers") or meta.get("acceptedAnswers")
    if isinstance(accepted, list) and accepted:
        accepted_normalized: Set[str] = {
            normalize_text(str(v)) for v in accepted if v is not None
        }
        return value in accepted_normalized
    
    # Get options
    options = _get_field(question, "options", default=[]) or []
    
    # 2. Multiple choice: match against correct option
    if question_type in ("multiple_choice", "multiple-choice", "mcq"):
        has_correct_flag = any(
            _get_field(opt, "is_correct", "isCorrect", "correct") is not None
            for opt in options
        )
        if has_correct_flag:
            for opt in options:
                is_correct = bool(_get_field(opt, "is_correct", "isCorrect", "correct", default=False))
                if not is_correct:
                    continue
                opt_label = normalize_text(str(_get_field(opt, "label", default="")))
                opt_id = normalize_text(str(_get_field(opt, "id", default="")))
                if value == opt_label or value == opt_id:
                    return True
            return False
        # Fallback: options lack correct flag, match against question.answer
        answer = _get_field(question, "answer", default="")
        if answer:
            return value == normalize_text(str(answer))
    
    # 3. Yes/No questions
    if question_type in ("yes_no", "yesno"):
        correct_answer = meta.get("correct_answer") or meta.get("correctAnswer")
        if isinstance(correct_answer, str) and correct_answer.strip():
            return value == normalize_text(correct_answer)
        
        # Check options for correct one
        for opt in options:
            is_correct = bool(_get_field(opt, "is_correct", "isCorrect", "correct", default=False))
            if is_correct:
                opt_label = normalize_text(str(_get_field(opt, "label", default="")))
                return value == opt_label
        
        # Fallback to answer text
        answer_text = _get_field(question, "answer_text", "answerText", "answer", default="")
        return value == normalize_text(str(answer_text or ""))
    
    # 4. Tracing: accept completion indicators
    if question_type in ("tracing", "trace"):
        if value in ("complete", "completed", "done", "true"):
            return True
        return None
    
    # 5. Color questions: match against color in meta or answer
    if question_type == "color":
        # Check meta.color
        color_answer = meta.get("color")
        if color_answer:
            if value == normalize_text(str(color_answer)):
                return True
        
        # Check direct answer
        answer = _get_field(question, "answer", default="")
        if answer:
            if value == normalize_text(str(answer)):
                return True
        
        # Check options if any
        for opt in options:
            is_correct = bool(_get_field(opt, "is_correct", "isCorrect", "correct", default=False))
            if is_correct:
                opt_label = normalize_text(str(_get_field(opt, "label", default="")))
                if value == opt_label:
                    return True
        
        return False
    
    # 6. Default: free-form speech/text - compare to answer + correct options
    candidates: Set[str] = set()
    
    # Add answer text
    answer_text = _get_field(question, "answer_text", "answerText", "answer")
    if answer_text:
        candidates.add(normalize_text(str(answer_text)))
    
    # Add correct option labels
    for opt in options:
        is_correct = bool(_get_field(opt, "is_correct", "isCorrect", "correct", default=False))
        if is_correct:
            opt_label = normalize_text(str(_get_field(opt, "label", default="")))
            if opt_label:
                candidates.add(opt_label)
    
    # Add color from meta
    color = meta.get("color")
    if color:
        candidates.add(normalize_text(str(color)))
    
    # Filter empty candidates
    candidates = {c for c in candidates if c}
    
    if not candidates:
        return None
    
    return value in candidates


def find_correct_option(options: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Find the correct option from a list of options.
    
    Args:
        options: List of option dicts
        
    Returns:
        The correct option dict or None
    """
    for opt in options:
        is_correct = bool(_get_field(opt, "is_correct", "isCorrect", "correct", default=False))
        if is_correct:
            return opt
    return None


def is_option_correct(option: Dict[str, Any]) -> bool:
    """Check if an option is marked as correct.
    
    Args:
        option: Option dict
        
    Returns:
        True if the option is correct
    """
    return bool(_get_field(option, "is_correct", "isCorrect", "correct", default=False))
