"""Input translation helpers.

Converts UI-like input payloads into decision_making events.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Optional, Tuple

from .content.correctness import compute_correct_for_question

_logger = logging.getLogger("input_translation")


def parse_input_json(raw: str) -> Optional[dict[str, Any]]:
    """Parse a JSON string input into a dict."""
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    return data if isinstance(data, dict) else None


def is_control_command(input_data: dict[str, Any]) -> bool:
    """Check if the input is a control command (PAUSE, RESUME, EXIT, etc.)."""
    label = str(input_data.get("label") or "").upper().strip()
    return label in {"PAUSE", "RESUME", "RESTART", "RESET", "EXIT", "STOP", "BACK", "SKIP_PHASE", "SKIP"}


def extract_control_command(input_data: dict[str, Any]) -> Optional[str]:
    """Extract control command from input data."""
    label = str(input_data.get("label") or "").upper().strip()
    if label in {"PAUSE", "RESUME", "RESTART", "RESET", "EXIT", "STOP", "SKIP_PHASE", "SKIP"}:
        return label
    if label == "BACK":
        return "EXIT"
    return None


def extract_user_answer(input_data: dict[str, Any]) -> Tuple[Optional[str], Optional[bool]]:
    """Extract user answer and correctness from input data."""
    _logger.debug(f"[TRANSLATE] extract_user_answer input_data keys={list(input_data.keys())}")
    value = input_data.get("label")
    if value is None:
        value = input_data.get("value")

    nested_answer = input_data.get("answer")
    if value is None and isinstance(nested_answer, dict):
        value = (
            nested_answer.get("label")
            or nested_answer.get("value")
            or nested_answer.get("id")
            or nested_answer.get("text")
        )
    if value is None:
        value = nested_answer

    # Support MatchingPhase payloads: {leftId, rightId, correct}
    if value is None:
        left_id = input_data.get("leftId")
        if left_id is not None:
            _logger.debug(f"[TRANSLATE] Using leftId fallback: {left_id}")
            value = left_id

    correct = input_data.get("correct")
    if correct is None and isinstance(nested_answer, dict):
        correct = nested_answer.get("correct")
    _logger.debug(f"[TRANSLATE] extract_user_answer → value={value}, correct={correct}")
    return (
        str(value) if value is not None else None,
        bool(correct) if correct is not None else None,
    )


def translate_input_to_user_intent(
    input_data: dict[str, Any],
    transaction_id: int,
    current_question: Optional[dict[str, Any]] = None,
    modality: str = "touch",
) -> Optional[Dict[str, Any]]:
    """Translate input data to USER_INTENT event for decision_making."""
    if is_control_command(input_data):
        return None
    
    value, correct = extract_user_answer(input_data)
    
    if value is None:
        return None
    
    if correct is None and current_question is not None:
        correct = compute_correct_for_question(current_question, value)
    
    payload: dict[str, Any] = {
        "transactionId": transaction_id,
        "value": value,
        "modality": modality,
    }
    
    if correct is not None:
        payload["correct"] = correct
    
    return {
        "type": "USER_INTENT",
        "payload": payload,
    }


def translate_input_to_game_control(
    input_data: dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """Translate input data to GAME_CONTROL event."""
    command = extract_control_command(input_data)
    
    if command is None:
        return None
    
    return {
        "type": "GAME_CONTROL",
        "payload": {
            "command": command,
        },
    }


class InputTranslator:
    """Stateful translator for UI-like input payloads."""
    
    def __init__(self) -> None:
        self._current_transaction_id: Optional[int] = None
        self._current_question: Optional[dict[str, Any]] = None
        self._current_game_state: Optional[str] = None
    
    def update_state(
        self,
        transaction_id: int,
        game_state: Optional[str] = None,
        question: Optional[dict[str, Any]] = None,
    ) -> None:
        """Update translator state from /decision/state payloads."""
        self._current_transaction_id = transaction_id
        self._current_game_state = game_state
        if question is not None:
            self._current_question = question
    
    def translate(
        self,
        json_data: str,
        modality: str = "touch",
    ) -> Optional[Dict[str, Any]]:
        """Translate a JSON string payload to a decision_making event."""
        input_data = parse_input_json(json_data)
        if input_data is None:
            return None

        return self.translate_input_data(input_data, modality=modality)

    def translate_input_data(
        self,
        input_data: dict[str, Any],
        modality: str = "touch",
    ) -> Optional[Dict[str, Any]]:
        """Translate already-parsed input data to a decision_making event."""
        _logger.debug(
            f"[TRANSLATE] translate_input_data: input_data={input_data}, "
            f"game_state={self._current_game_state}, tx={self._current_transaction_id}"
        )
        # Check for control command first
        control_event = translate_input_to_game_control(input_data)
        if control_event is not None:
            _logger.debug(f"[TRANSLATE] Control command detected: {control_event}")
            return control_event

        # decision_making only accepts USER_INTENT while waiting for input.
        # Allowing intents in FAIL_L1 can cancel the retry auto-advance timer
        # and leave gameplay stuck on the failure feedback screen.
        if self._current_game_state != "WAIT_INPUT":
            _logger.debug(
                f"[TRANSLATE] BLOCKED - game_state={self._current_game_state} != WAIT_INPUT"
            )
            return None

        if self._current_transaction_id is None:
            _logger.debug("[TRANSLATE] BLOCKED - transaction_id is None")
            return None

        result = translate_input_to_user_intent(
            input_data,
            self._current_transaction_id,
            self._current_question,
            modality,
        )
        _logger.debug(f"[TRANSLATE] translate_input_to_user_intent → {result}")
        return result
