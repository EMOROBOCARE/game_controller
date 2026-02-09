"""Difficulty configuration models."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, field_validator


DifficultyLevel = Literal["basic", "intermediate", "advanced"]

OPTIONS_COUNT_BY_DIFFICULTY: dict[DifficultyLevel, int] = {
    "basic": 2,
    "intermediate": 3,
    "advanced": 4,
}


class DifficultyConfig(BaseModel):
    """Configuration for a difficulty level."""

    level: DifficultyLevel
    options_count: int

    @field_validator("options_count")
    @classmethod
    def validate_options_count(cls, v: int) -> int:
        if v < 2 or v > 6:
            raise ValueError("options_count must be between 2 and 6")
        return v

    @classmethod
    def from_level(cls, level: DifficultyLevel) -> DifficultyConfig:
        """Create config from difficulty level with default options count."""
        return cls(
            level=level,
            options_count=OPTIONS_COUNT_BY_DIFFICULTY[level],
        )

    @classmethod
    def basic(cls) -> DifficultyConfig:
        """Create basic difficulty config."""
        return cls.from_level("basic")

    @classmethod
    def intermediate(cls) -> DifficultyConfig:
        """Create intermediate difficulty config."""
        return cls.from_level("intermediate")

    @classmethod
    def advanced(cls) -> DifficultyConfig:
        """Create advanced difficulty config."""
        return cls.from_level("advanced")


def get_options_count(difficulty: str) -> int:
    """Get options count for a difficulty level string."""
    level = difficulty.lower()
    return OPTIONS_COUNT_BY_DIFFICULTY.get(level, 3)  # type: ignore
