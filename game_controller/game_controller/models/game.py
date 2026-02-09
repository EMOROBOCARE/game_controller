"""Game configuration models."""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, Field

from .option import Option
from .phase import PhaseConfig


class Question(BaseModel):
    """A game question."""

    question_id: int = Field(alias="questionId")
    prompt: str = ""
    image_url: Optional[str] = Field(default=None, alias="imageUrl")
    answer: Optional[str] = None
    question_type: Optional[str] = Field(default=None, alias="questionType")
    options: list[Option] = Field(default_factory=list)
    tiered_hints: dict[str, list[str]] = Field(default_factory=dict, alias="tieredHints")
    fail_l1_prompt: Optional[str] = Field(default=None, alias="failL1Prompt")
    fail_l2_prompt: Optional[str] = Field(default=None, alias="failL2Prompt")
    meta: dict[str, Any] = Field(default_factory=dict)

    model_config = {"populate_by_name": True}

    def correct_option_id(self) -> Any:
        """Get the ID of the correct option."""
        for opt in self.options:
            if opt.correct:
                return opt.id
        return None

    def correct_option(self) -> Optional[Option]:
        """Get the correct option."""
        for opt in self.options:
            if opt.correct:
                return opt
        return None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Question:
        """Create from dict with flexible field names."""
        options = [Option.from_dict(opt) for opt in data.get("options", [])]
        return cls(
            question_id=int(data.get("questionId", data.get("question_id", 0))),
            prompt=data.get("prompt", ""),
            image_url=data.get("imageUrl", data.get("image_url")),
            answer=data.get("answer"),
            question_type=data.get("questionType", data.get("question_type")),
            options=options,
            tiered_hints=data.get("tieredHints", data.get("tiered_hints", {})),
            fail_l1_prompt=data.get("failL1Prompt", data.get("fail_l1_prompt")),
            fail_l2_prompt=data.get("failL2Prompt", data.get("fail_l2_prompt")),
            meta=data.get("meta", data.get("metadata", {})),
        )

    def to_payload(self) -> dict[str, Any]:
        """Convert to payload for decision_making."""
        return {
            "questionId": self.question_id,
            "prompt": self.prompt,
            "imageUrl": self.image_url,
            "answer": self.answer,
            "questionType": self.question_type,
            "options": [
                {
                    "id": opt.id,
                    "label": opt.label,
                    "imageUrl": opt.image_url,
                    "correct": opt.correct,
                }
                for opt in self.options
            ],
            "tieredHints": self.tiered_hints,
            "failL1Prompt": self.fail_l1_prompt,
            "failL2Prompt": self.fail_l2_prompt,
            "meta": self.meta,
        }


class Round(BaseModel):
    """A game round."""

    id: int
    phase: str
    difficulty: str = ""
    question: Question

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Round:
        """Create from dict."""
        question = Question.from_dict(data.get("question", {}))
        return cls(
            id=int(data.get("id", 0)),
            phase=data.get("phase", ""),
            difficulty=data.get("difficulty", ""),
            question=question,
        )


class GameConfig(BaseModel):
    """Full game configuration."""

    slug: str
    title: str
    introduction: str = ""
    difficulty: str = "basic"
    phase_sequence: list[str] = Field(default_factory=list, alias="phaseSequence")
    phase_configs: dict[str, PhaseConfig] = Field(default_factory=dict, alias="phaseConfigs")
    rounds: list[Round] = Field(default_factory=list)
    game_type: Optional[str] = Field(default=None, alias="gameType")
    special_handler: Optional[str] = Field(default=None, alias="specialHandler")

    model_config = {"populate_by_name": True}

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> GameConfig:
        """Create from GAME_INIT payload."""
        phase_configs_raw = payload.get("phaseConfigs", payload.get("phase_configs", {}))
        phase_configs = {
            key: PhaseConfig.from_dict(val)
            for key, val in phase_configs_raw.items()
        }
        rounds = [Round.from_dict(r) for r in payload.get("rounds", [])]

        return cls(
            slug=payload.get("slug", ""),
            title=payload.get("title", ""),
            introduction=payload.get("introduction", ""),
            difficulty=payload.get("difficulty", "basic"),
            phase_sequence=payload.get("phaseSequence", payload.get("phase_sequence", [])),
            phase_configs=phase_configs,
            rounds=rounds,
            game_type=payload.get("gameType", payload.get("game_type")),
            special_handler=payload.get("specialHandler", payload.get("special_handler")),
        )
