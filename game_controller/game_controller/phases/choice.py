"""Choice phase handler (P7)."""

from __future__ import annotations

import random
from typing import Any, Optional, Tuple

from ..models.game import Question
from ..models.option import Option
from ..models.difficulty import DifficultyLevel
from .base import BasePhaseHandler


class ChoicePhaseHandler(BasePhaseHandler):
    """Handler for P7 Choice phase.

    Robot asks "¿Es un ... o ...?" with two options.
    The correct answer position depends on difficulty:
    - basic: second option is correct
    - intermediate: first option is correct
    - advanced: random position

    User responds with the actual name (e.g., "rojo"), not position.
    """

    def __init__(self, config: Any, difficulty: DifficultyLevel = "basic") -> None:
        super().__init__(config)
        self._difficulty: DifficultyLevel = difficulty
        self._option1: Optional[Option] = None
        self._option2: Optional[Option] = None
        self._correct_position: int = 0  # 0=first, 1=second

    def set_difficulty(self, difficulty: DifficultyLevel) -> None:
        """Set the difficulty level."""
        self._difficulty = difficulty

    def setup_round(self, options: list[Option]) -> None:
        """Setup the two options with correct position by difficulty."""
        super().setup_round(options)

        correct_opts = [o for o in options if o.correct]
        incorrect_opts = [o for o in options if not o.correct]

        if not correct_opts or not incorrect_opts:
            self._option1 = correct_opts[0] if correct_opts else None
            self._option2 = incorrect_opts[0] if incorrect_opts else None
            return

        correct = random.choice(correct_opts)
        incorrect = random.choice(incorrect_opts)

        # Determine correct position by difficulty
        if self._difficulty == "basic":
            self._correct_position = 1  # Second option
        elif self._difficulty == "intermediate":
            self._correct_position = 0  # First option
        else:  # advanced
            self._correct_position = random.randint(0, 1)

        # Assign options based on position
        if self._correct_position == 0:
            self._option1 = correct
            self._option2 = incorrect
        else:
            self._option1 = incorrect
            self._option2 = correct

    def get_options_pair(self) -> Tuple[Optional[Option], Optional[Option]]:
        """Get the two options in display order."""
        return (self._option1, self._option2)

    def get_correct_option(self) -> Optional[Option]:
        """Get the correct option."""
        if self._correct_position == 0:
            return self._option1
        return self._option2

    def build_prompt(self, template: str) -> str:
        """Build '¿Es un ... o ...?' prompt."""
        opt1_label = self._option1.label if self._option1 else "?"
        opt2_label = self._option2.label if self._option2 else "?"

        prompt = template.replace("{option1}", opt1_label).replace("{option2}", opt2_label)
        return prompt

    def evaluate_input(
        self,
        input_data: dict[str, Any],
        question: Question,
    ) -> Tuple[bool, Optional[str]]:
        """Evaluate if user answered with the correct option name."""
        if not input_data:
            return False, None

        value = str(input_data.get("value", input_data.get("label", ""))).lower()
        if not value:
            return False, None

        correct_opt = self.get_correct_option()
        if not correct_opt:
            return False, None

        # Check if answer matches the correct option
        if value == str(correct_opt.id).lower() or value == str(correct_opt.label).lower():
            return True, None

        return False, None

    def handle_failure_l1(self, question: Question) -> dict[str, Any]:
        """Handle first failure with hint."""
        correct = self.get_correct_option()
        hint = f"Piensa bien, ¿cuál es?"
        return self.build_fail_l1_payload(hint, correct.id if correct else None)

    def handle_failure_l2(self, question: Question) -> dict[str, Any]:
        """Handle second failure with solution."""
        correct = self.get_correct_option()
        label = correct.label if correct else ""
        hint = f"La respuesta correcta es {label}."
        return self.build_fail_l2_payload(hint, correct.id if correct else None)
