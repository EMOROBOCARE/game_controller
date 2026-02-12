"""Discrimination phase handler (P1/P2/P3)."""

from __future__ import annotations

from typing import Any, Optional, Tuple

from ..models.phase import Modality
from ..models.game import Question
from .base import BasePhaseHandler


class DiscriminationPhaseHandler(BasePhaseHandler):
    """Handler for P1/P2/P3 phases.

    User selects the correct option from multiple choices.
    Modality defaults to BUTTON but can be overridden via constructor.
    """

    def __init__(self, config: Any, default_modality: Modality = Modality.BUTTON) -> None:
        super().__init__(config)
        self._modality = default_modality

    @property
    def modality(self) -> Modality:
        return self._modality

    def evaluate_input(
        self,
        input_data: dict[str, Any],
        question: Question,
    ) -> Tuple[bool, Optional[str]]:
        """Evaluate if user selected the correct option."""
        if not input_data:
            return False, None

        # If upstream already computed correctness, use it
        if "correct" in input_data and input_data.get("correct") is not None:
            return bool(input_data["correct"]), None

        value = input_data.get("value") or input_data.get("label")
        if not value:
            return False, None

        # Check against correct options
        for opt in question.options:
            if not opt.correct:
                continue
            if value == opt.id or str(value).lower() == str(opt.label).lower():
                return True, None

        return False, None

    def handle_failure_l1(self, question: Question) -> dict[str, Any]:
        """Handle first failure with hint."""
        hint = self.get_hint_for_level(question, 1)
        return self.build_fail_l1_payload(hint, question.correct_option_id())

    def handle_failure_l2(self, question: Question) -> dict[str, Any]:
        """Handle second failure with solution."""
        hint = self.get_hint_for_level(question, 2)
        return self.build_fail_l2_payload(hint, question.correct_option_id())
