"""Base phase handler."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Optional, Tuple

from ..models.phase import PhaseConfig
from ..models.game import Question
from ..models.option import Option


class BasePhaseHandler(ABC):
    """Base class for all phase handlers."""

    def __init__(self, config: PhaseConfig) -> None:
        self.config = config
        self._failure_count: int = 0

    def reset_failure_count(self) -> None:
        """Reset the failure counter."""
        self._failure_count = 0

    def increment_failure(self) -> None:
        """Increment the failure counter."""
        self._failure_count += 1

    def failure_count(self) -> int:
        """Get current failure count."""
        return self._failure_count

    def can_retry(self) -> bool:
        """Check if user can retry (hasn't exceeded max failures)."""
        if self.config.max_failures <= 0:
            return False
        return self._failure_count < self.config.max_failures

    def setup_round(self, options: list[Option]) -> None:
        """Setup for a new round. Override in subclasses if needed."""
        self.reset_failure_count()

    @abstractmethod
    def evaluate_input(
        self,
        input_data: dict[str, Any],
        question: Question,
    ) -> Tuple[bool, Optional[str]]:
        """Evaluate user input.

        Args:
            input_data: User input data with 'value', 'label', 'correct' fields
            question: Current question

        Returns:
            Tuple of (is_correct, optional_feedback_message)
        """
        raise NotImplementedError

    @abstractmethod
    def handle_failure_l1(self, question: Question) -> dict[str, Any]:
        """Handle first level failure.

        Returns:
            Payload for FAIL_L1 state
        """
        raise NotImplementedError

    @abstractmethod
    def handle_failure_l2(self, question: Question) -> dict[str, Any]:
        """Handle second level failure.

        Returns:
            Payload for FAIL_L2 state with auto_advance=True
        """
        raise NotImplementedError

    def get_hint_for_level(self, question: Question, level: int) -> str:
        """Get hint text for a failure level."""
        level_key = f"level{level}"
        hints = question.tiered_hints.get(level_key, [])

        if hints:
            idx = max(self._failure_count - 1, 0)
            return hints[min(idx, len(hints) - 1)]

        if level == 1 and question.fail_l1_prompt:
            return question.fail_l1_prompt
        if level == 2 and question.fail_l2_prompt:
            return question.fail_l2_prompt

        return "Inténtalo de nuevo."

    def build_fail_l1_payload(
        self,
        hint: str,
        correct_id: Any,
    ) -> dict[str, Any]:
        """Build standard FAIL_L1 payload."""
        return {
            "hint": hint,
            "action": self.config.fail_l1_action,
            "correctOptionId": correct_id,
            "autoAdvance": False,
        }

    def build_fail_l2_payload(
        self,
        hint: str,
        correct_id: Any,
    ) -> dict[str, Any]:
        """Build standard FAIL_L2 payload."""
        return {
            "hint": hint,
            "action": self.config.fail_l2_action,
            "correctOptionId": correct_id,
            "autoAdvance": True,
        }
