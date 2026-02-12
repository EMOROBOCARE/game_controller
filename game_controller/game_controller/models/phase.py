"""Phase configuration models."""

from __future__ import annotations

from enum import Enum
from typing import Any, Optional

from pydantic import BaseModel, Field


class Modality(str, Enum):
    """Fixed modality per phase type."""

    DRAGGING = "dragging"
    VOICE = "voice"
    BUTTON = "button"
    BUTTON_VOICE = "button_voice"
    YES_NO = "yes_no"
    POINTING = "pointing"
    SELECTION = "selection"


# Backward compat alias
InteractionType = Modality


class PhaseType(str, Enum):
    """Phase type identifiers."""

    P1 = "P1"  # Matching/Association
    P2 = "P2"  # Repetition/Pronunciation
    P3 = "P3"  # Discrimination
    P4 = "P4"  # Yes/No (incorrect first, then correct)
    P5 = "P5"  # Pointing (child asks, robot highlights)
    P6 = "P6"  # Choice (Es un ... o ...)
    TRACING = "TRACING"  # Drawing/Tracing


class PhaseConfig(BaseModel):
    """Configuration for a game phase."""

    hints: list[str] = Field(default_factory=lambda: ["highlight", "highlight", "say_answer"])
    max_failures: int = Field(default=2, alias="maxFailures")
    text_instructions: Optional[str] = Field(default=None, alias="textInstructions")
    verbal_instructions: Optional[str] = Field(default=None, alias="verbalInstructions")
    prompt_text: Optional[str] = Field(default=None, alias="promptText")
    prompt_verbal: Optional[str] = Field(default=None, alias="promptVerbal")
    expected_answer: Optional[str] = Field(default=None, alias="expectedAnswer")
    success_response: Optional[str] = Field(default=None, alias="successResponse")
    sub_rounds: int = Field(default=1, alias="subRounds")
    config: dict[str, Any] = Field(default_factory=dict)

    model_config = {"populate_by_name": True}

    @property
    def effective_max_failures(self) -> int:
        """Return min(max_failures, len(hints)) if max_failures > 0, else len(hints)."""
        if self.max_failures <= 0:
            return len(self.hints)
        return min(self.max_failures, len(self.hints))

    # Backward compat properties
    @property
    def fail_l1_action(self) -> str:
        return self.hints[0] if self.hints else "highlight"

    @property
    def fail_l2_action(self) -> str:
        return self.hints[1] if len(self.hints) > 1 else "highlight_and_solve"

    @property
    def phase_introduction(self) -> Optional[str]:
        return self.text_instructions

    @property
    def prompt_template(self) -> Optional[str]:
        return self.prompt_verbal or self.prompt_text

    @property
    def hint_type(self) -> str:
        return self.hints[0] if self.hints else "highlight"

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PhaseConfig:
        """Create from dict with flexible field names."""
        # Hints: prefer 'hints' list, fall back to failL1/L2 actions
        hints = data.get("hints")
        if not isinstance(hints, list):
            l1 = data.get("failL1Action", data.get("fail_l1_action", "highlight"))
            l2 = data.get("failL2Action", data.get("fail_l2_action", "highlight_and_solve"))
            hints = [l1, l2]

        # Instructions: prefer split text/verbal, fall back to phase_introduction
        text_instr = data.get("textInstructions", data.get("text_instructions"))
        verbal_instr = data.get("verbalInstructions", data.get("verbal_instructions"))
        if text_instr is None and verbal_instr is None:
            intro = data.get("phaseIntroduction", data.get("phase_introduction"))
            if intro:
                text_instr = intro
                verbal_instr = intro

        # Prompt: prefer split text/verbal, fall back to single prompt
        prompt_text = data.get("promptText", data.get("prompt_text"))
        prompt_verbal = data.get("promptVerbal", data.get("prompt_verbal"))
        if prompt_text is None and prompt_verbal is None:
            prompt = data.get("prompt")
            if prompt:
                prompt_text = prompt
                prompt_verbal = prompt

        return cls(
            hints=hints,
            max_failures=int(data.get("maxFailures", data.get("max_failures", 2))),
            text_instructions=text_instr,
            verbal_instructions=verbal_instr,
            prompt_text=prompt_text,
            prompt_verbal=prompt_verbal,
            expected_answer=data.get("expectedAnswer", data.get("expected_answer")),
            success_response=data.get("successResponse", data.get("success_response")),
            sub_rounds=int(data.get("subRounds", data.get("sub_rounds", 1))),
            config=data.get("config", {}),
        )


# Default phase configurations
DEFAULT_PHASE_CONFIGS: dict[str, dict[str, Any]] = {
    "P1": {
        "hints": ["highlight", "highlight", "say_answer"],
        "maxFailures": 2,
    },
    "P2": {
        "hints": ["clear_say", "model_and_correct"],
        "maxFailures": 2,
    },
    "P3": {
        "hints": ["highlight", "highlight_and_solve"],
        "maxFailures": 2,
    },
    "P4": {
        "hints": ["head_gesture", "explain"],
        "maxFailures": 2,
        "subRounds": 2,
    },
    "P5": {
        "hints": [],
        "maxFailures": 0,
        "successResponse": "Aquííí",
    },
    "P6": {
        "hints": ["suggest", "offer_choices"],
        "maxFailures": 2,
    },
}
