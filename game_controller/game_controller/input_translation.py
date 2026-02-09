"""Input translation helpers.

Converts UI-like input payloads into decision_making events.
"""

from __future__ import annotations

import json
from typing import Any, Optional, Tuple

from .content.correctness import compute_correct_for_question


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
    value = input_data.get("label")
    if value is None:
        value = input_data.get("value") or input_data.get("answer")

    correct = input_data.get("correct")
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
        # Check for control command first
        control_event = translate_input_to_game_control(input_data)
        if control_event is not None:
            return control_event

        # Only accept user intents in WAIT_INPUT state
        if self._current_game_state not in ("WAIT_INPUT", "FAIL_L1"):
            return None

        if self._current_transaction_id is None:
            return None

        return translate_input_to_user_intent(
            input_data,
            self._current_transaction_id,
            self._current_question,
            modality,
        )
