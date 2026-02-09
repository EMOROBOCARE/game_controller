"""Unified Yes/No phase handler (P4_YESNO)."""

from __future__ import annotations

import random
from typing import Any, Optional, Tuple

from ..models.game import Question
from ..models.option import Option
from .base import BasePhaseHandler


class UnifiedYesNoHandler(BasePhaseHandler):
    """Handler for unified P4+P5 Yes/No phase.

    This phase:
    1. Samples one correct and one incorrect answer randomly
    2. First asks about the incorrect one (expected answer: No)
    3. Then asks about the correct one (expected answer: Yes)

    Counts as 1 round with 2 internal sub-rounds.
    """

    def __init__(self, config: Any) -> None:
        super().__init__(config)
        self._sub_round: int = 0  # 0 = asking incorrect, 1 = asking correct
        self._sampled_correct: Optional[Option] = None
        self._sampled_incorrect: Optional[Option] = None

    def setup_round(self, options: list[Option]) -> None:
        """Sample one correct and one incorrect option for this round."""
        super().setup_round(options)
        self._sub_round = 0

        correct_opts = [o for o in options if o.correct]
        incorrect_opts = [o for o in options if not o.correct]

        self._sampled_correct = random.choice(correct_opts) if correct_opts else None
        self._sampled_incorrect = random.choice(incorrect_opts) if incorrect_opts else None

    def get_current_item(self) -> Optional[Option]:
        """Get the item currently being asked about."""
        if self._sub_round == 0:
            return self._sampled_incorrect
        return self._sampled_correct

    def get_expected_answer(self) -> str:
        """Get the expected yes/no answer for current sub-round."""
        return "no" if self._sub_round == 0 else "si"

    def build_prompt(self, template: str, item: Optional[Option]) -> str:
        """Build the prompt for the current item."""
        if not item:
            return template

        return template.replace("{word}", item.label).replace(
            "{colour}", item.label
        ).replace("{color}", item.label)

    def evaluate_input(
        self,
        input_data: dict[str, Any],
        question: Question,
    ) -> Tuple[bool, Optional[str]]:
        """Evaluate yes/no answer."""
        if not input_data:
            return False, None

        answer = str(input_data.get("value", input_data.get("label", ""))).lower()
        expected = self.get_expected_answer()

        # Normalize yes variations
        yes_variants = {"si", "sí", "yes", "s", "1", "true"}
        no_variants = {"no", "n", "0", "false"}

        if expected == "si":
            is_correct = answer in yes_variants
        else:
            is_correct = answer in no_variants

        return is_correct, None

    def advance_sub_round(self) -> bool:
        """Advance to next sub-round.

        Returns:
            True if phase complete (both sub-rounds done),
            False if continuing to next sub-round.
        """
        if self._sub_round == 0:
            self._sub_round = 1
            self.reset_failure_count()
            return False
        return True

    def is_first_sub_round(self) -> bool:
        """Check if we're in the first sub-round."""
        return self._sub_round == 0

    def handle_failure_l1(self, question: Question) -> dict[str, Any]:
        """Handle first failure."""
        expected = self.get_expected_answer()
        item = self.get_current_item()
        item_label = item.label if item else ""

        if expected == "no":
            hint = f"No, esto no es {item_label}."
        else:
            hint = f"Sí, esto es {item_label}."

        return self.build_fail_l1_payload(hint, expected)

    def handle_failure_l2(self, question: Question) -> dict[str, Any]:
        """Handle second failure."""
        expected = self.get_expected_answer()
        item = self.get_current_item()
        item_label = item.label if item else ""

        if expected == "no":
            hint = f"La respuesta es no. Esto no es {item_label}."
        else:
            hint = f"La respuesta es sí. Esto es {item_label}."

        return self.build_fail_l2_payload(hint, expected)
