"""Phase configuration models."""

from __future__ import annotations

from enum import Enum
from typing import Any, Optional

from pydantic import BaseModel, Field


class InteractionType(str, Enum):
    """Types of user interaction for phases."""

    MATCHING = "matching"
    VOICE = "voice"
    TOUCH = "touch"
    TOUCH_VOICE = "touch_voice"
    YES_NO = "yes_no"
    POINTING = "pointing"
    SELECTION = "selection"


class PhaseType(str, Enum):
    """Phase type identifiers."""

    P1 = "P1"  # Matching/Association
    P2 = "P2"  # Repetition/Pronunciation
    P3 = "P3"  # Discrimination
    P4_YESNO = "P4_YESNO"  # Unified Yes/No (incorrect first, then correct)
    P6 = "P6"  # Pointing (child asks, robot highlights)
    P7 = "P7"  # Choice (Es un ... o ...)
    TRACING = "TRACING"  # Drawing/Tracing


class PhaseConfig(BaseModel):
    """Configuration for a game phase."""

    interaction_type: InteractionType = Field(alias="interactionType")
    fail_l1_action: str = Field(default="highlight", alias="failL1Action")
    fail_l2_action: str = Field(default="highlight_and_solve", alias="failL2Action")
    max_failures: int = Field(default=2, alias="maxFailures")
    phase_introduction: Optional[str] = Field(default=None, alias="phaseIntroduction")
    prompt_template: Optional[str] = Field(default=None, alias="prompt")
    expected_answer: Optional[str] = Field(default=None, alias="expectedAnswer")
    hint_type: str = Field(default="highlight", alias="hint")
    success_response: Optional[str] = Field(default=None, alias="successResponse")
    sub_rounds: int = Field(default=1, alias="subRounds")
    config: dict[str, Any] = Field(default_factory=dict)

    model_config = {"populate_by_name": True}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PhaseConfig:
        """Create from dict with flexible field names."""
        interaction = data.get("interactionType", data.get("interaction_type", ""))
        if isinstance(interaction, list):
            if "matching" in interaction:
                interaction = "matching"
            elif "voice" in interaction and "touch" in interaction:
                interaction = "touch_voice"
            elif "voice" in interaction:
                interaction = "voice"
            elif "touch" in interaction:
                interaction = "touch"
            else:
                interaction = "selection"

        return cls(
            interaction_type=InteractionType(interaction) if interaction else InteractionType.SELECTION,
            fail_l1_action=data.get("failL1Action", data.get("fail_l1_action", "highlight")),
            fail_l2_action=data.get("failL2Action", data.get("fail_l2_action", "highlight_and_solve")),
            max_failures=int(data.get("maxFailures", data.get("max_failures", 2))),
            phase_introduction=data.get("phaseIntroduction", data.get("phase_introduction")),
            prompt_template=data.get("prompt"),
            expected_answer=data.get("expectedAnswer", data.get("expected_answer")),
            hint_type=data.get("hint", "highlight"),
            success_response=data.get("successResponse", data.get("success_response")),
            sub_rounds=int(data.get("subRounds", data.get("sub_rounds", 1))),
            config=data.get("config", {}),
        )


# Default phase configurations
DEFAULT_PHASE_CONFIGS: dict[str, dict[str, Any]] = {
    "P1": {
        "interactionType": "matching",
        "failL1Action": "blink_correct",
        "failL2Action": "highlight_and_select",
        "maxFailures": 2,
    },
    "P2": {
        "interactionType": "voice",
        "failL1Action": "clear_say",
        "failL2Action": "model_and_correct",
        "maxFailures": 2,
    },
    "P3": {
        "interactionType": "touch_voice",
        "failL1Action": "point",
        "failL2Action": "highlight_and_solve",
        "maxFailures": 2,
    },
    "P4_YESNO": {
        "interactionType": "yes_no",
        "failL1Action": "shake_head",
        "failL2Action": "explain",
        "maxFailures": 2,
        "subRounds": 2,
    },
    "P6": {
        "interactionType": "pointing",
        "failL1Action": "skip",
        "failL2Action": "skip",
        "maxFailures": 0,
        "successResponse": "Aquííí",
    },
    "P7": {
        "interactionType": "selection",
        "failL1Action": "suggest",
        "failL2Action": "offer_choices",
        "maxFailures": 2,
    },
}
