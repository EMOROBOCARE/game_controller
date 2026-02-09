"""Game state models."""

from __future__ import annotations

from enum import Enum
from typing import Any, Optional

from pydantic import BaseModel, Field


class GameState(str, Enum):
    """Possible game states."""

    IDLE = "IDLE"
    PHASE_INTRO = "PHASE_INTRO"
    ROUND_SETUP = "ROUND_SETUP"
    QUESTION_PRESENT = "QUESTION_PRESENT"
    WAIT_INPUT = "WAIT_INPUT"
    FAIL_L1 = "FAIL_L1"
    FAIL_L2 = "FAIL_L2"
    CORRECT = "CORRECT"
    PHASE_COMPLETE = "PHASE_COMPLETE"
    GAME_COMPLETE = "GAME_COMPLETE"


class StatePayload(BaseModel):
    """Payload for game state updates."""

    transaction_id: int = Field(default=0, alias="transactionId")
    game_state: GameState = Field(alias="gameState")
    session_id: Optional[int] = Field(default=None, alias="sessionId")
    phase: Optional[str] = None
    round_id: Optional[int] = Field(default=None, alias="roundId")
    question: Optional[dict[str, Any]] = None
    introduction: Optional[str] = None
    hint: Optional[str] = None
    feedback: Optional[str] = None

    model_config = {"populate_by_name": True}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> StatePayload:
        """Create from dict with flexible field names."""
        state_str = data.get("gameState", data.get("game_state", "IDLE"))
        try:
            game_state = GameState(state_str.upper())
        except ValueError:
            game_state = GameState.IDLE

        return cls(
            transaction_id=int(data.get("transactionId", data.get("transaction_id", 0))),
            game_state=game_state,
            session_id=data.get("sessionId", data.get("session_id")),
            phase=data.get("phase"),
            round_id=data.get("roundId", data.get("round_id")),
            question=data.get("question"),
            introduction=data.get("introduction"),
            hint=data.get("hint"),
            feedback=data.get("feedback"),
        )


def build_phase_intro_payload(
    introduction: str,
    phase: str,
    config: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Build payload for PHASE_INTRO state."""
    payload: dict[str, Any] = {
        "introduction": introduction,
        "phase": phase,
    }
    if config:
        payload["phaseConfig"] = config
    return payload


def build_question_present_payload(question_dict: dict[str, Any]) -> dict[str, Any]:
    """Build payload for QUESTION_PRESENT state."""
    return {"question": question_dict}


def build_wait_input_payload(round_id: int, question_id: int) -> dict[str, Any]:
    """Build payload for WAIT_INPUT state."""
    return {
        "roundId": round_id,
        "questionId": question_id,
    }


def build_fail_payload(
    hint: str,
    action: str,
    correct_id: Any,
    auto_advance: bool = False,
) -> dict[str, Any]:
    """Build payload for FAIL_L1 or FAIL_L2 state."""
    return {
        "hint": hint,
        "action": action,
        "correctOptionId": correct_id,
        "autoAdvance": auto_advance,
    }


def build_correct_payload(feedback: Optional[str] = None) -> dict[str, Any]:
    """Build payload for CORRECT state."""
    payload: dict[str, Any] = {}
    if feedback:
        payload["feedback"] = feedback
    return payload


def build_phase_complete_payload(phase: str) -> dict[str, Any]:
    """Build payload for PHASE_COMPLETE state."""
    return {"phase": phase}
