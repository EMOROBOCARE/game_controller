"""Pointing phase handler (P5)."""

from __future__ import annotations

from typing import Any, Optional, Tuple

from ..models.phase import Modality
from ..models.game import Question
from ..models.option import Option
from .base import BasePhaseHandler


class PointingPhaseHandler(BasePhaseHandler):
    """Handler for P5 Pointing phase.

    Child asks "where is X?" among options.
    No correct answer - robot highlights what child says
    and responds "Aquííí" (success_response).

    On timeout, skips to next phase.
    """

    @property
    def modality(self) -> Modality:
        return Modality.POINTING

    def __init__(self, config: Any) -> None:
        super().__init__(config)
        self._highlighted: set[str] = set()
        self._selected_items: list[str] = []

    def setup_round(self, options: list[Option]) -> None:
        """Reset highlighting for new round."""
        super().setup_round(options)
        self._highlighted = set()
        self._selected_items = []

    def evaluate_input(
        self,
        input_data: dict[str, Any],
        question: Question,
    ) -> Tuple[bool, Optional[str]]:
        """Evaluate pointing input.

        Any valid option is accepted - we highlight it and respond.
        """
        if not input_data:
            return False, None

        value = input_data.get("value") or input_data.get("label")
        if not value:
            return False, None

        value_lower = str(value).lower()

        # Check if value matches any option
        for opt in question.options:
            if str(opt.id).lower() == value_lower or str(opt.label).lower() == value_lower:
                self._highlighted.add(str(opt.id))
                self._selected_items.append(opt.label)

                # Return success with "Aquííí" response
                response = self.config.success_response or "Aquííí"
                return True, response

        return False, None

    def get_highlighted_ids(self) -> set[str]:
        """Get set of highlighted option IDs."""
        return self._highlighted.copy()

    def get_ui_updates(self, options: list[Option]) -> list[Option]:
        """Get options with highlighting/hiding applied."""
        result = []
        for opt in options:
            new_opt = opt.model_copy()

            if str(opt.id) in self._highlighted:
                new_opt.highlighted = True
                new_opt.hidden = False
            else:
                new_opt.hidden = True
                new_opt.highlighted = False

            result.append(new_opt)
        return result

    def handle_failure_l1(self, question: Question) -> dict[str, Any]:
        """P6 doesn't really fail - just skip hint."""
        return {
            "hint": "",
            "action": "skip",
            "correctOptionId": None,
            "autoAdvance": False,
        }

    def handle_failure_l2(self, question: Question) -> dict[str, Any]:
        """P6 doesn't fail - skip to next."""
        return {
            "hint": "",
            "action": "skip",
            "correctOptionId": None,
            "autoAdvance": True,
        }
