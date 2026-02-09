"""Phase handler registry and factory."""

from __future__ import annotations

from typing import Any, Optional

from ..models.phase import PhaseConfig, DEFAULT_PHASE_CONFIGS
from ..models.difficulty import DifficultyLevel
from .base import BasePhaseHandler
from .discrimination import DiscriminationPhaseHandler
from .yes_no import UnifiedYesNoHandler
from .pointing import PointingPhaseHandler
from .choice import ChoicePhaseHandler


def get_phase_handler(
    phase_code: str,
    config: PhaseConfig,
    difficulty: DifficultyLevel = "basic",
) -> BasePhaseHandler:
    """Get the appropriate phase handler for a phase code.

    Args:
        phase_code: Phase identifier (P1, P2, P3, P4_YESNO, P6, P7, etc.)
        config: Phase configuration
        difficulty: Difficulty level (used by P7)

    Returns:
        Appropriate phase handler instance
    """
    code_upper = phase_code.upper()

    if code_upper in ("P1", "MATCHING", "ASSOCIATION"):
        return DiscriminationPhaseHandler(config)

    if code_upper in ("P2", "REPETITION", "VOICE"):
        return DiscriminationPhaseHandler(config)

    if code_upper in ("P3", "DISCRIMINATION"):
        return DiscriminationPhaseHandler(config)

    if code_upper in ("P4_YESNO", "P4", "P5", "YES_NO", "YESNO"):
        return UnifiedYesNoHandler(config)

    if code_upper in ("P6", "POINTING"):
        return PointingPhaseHandler(config)

    if code_upper in ("P7", "CHOICE"):
        handler = ChoicePhaseHandler(config, difficulty)
        return handler

    if code_upper == "TRACING":
        return DiscriminationPhaseHandler(config)

    # Default to discrimination handler
    return DiscriminationPhaseHandler(config)


def create_phase_handlers(
    phase_sequence: list[str],
    phase_configs: dict[str, PhaseConfig],
    difficulty: DifficultyLevel = "basic",
) -> dict[str, BasePhaseHandler]:
    """Create handlers for all phases in a sequence.

    Args:
        phase_sequence: List of phase codes
        phase_configs: Phase configurations by phase code
        difficulty: Difficulty level

    Returns:
        Dict of phase code -> handler instance
    """
    handlers: dict[str, BasePhaseHandler] = {}

    for phase_code in phase_sequence:
        config = phase_configs.get(phase_code)

        if not config:
            # Use default config
            default_data = DEFAULT_PHASE_CONFIGS.get(phase_code, {})
            config = PhaseConfig.from_dict(default_data) if default_data else None

        if config:
            handlers[phase_code] = get_phase_handler(phase_code, config, difficulty)

    return handlers


def get_default_phase_config(phase_code: str) -> Optional[PhaseConfig]:
    """Get the default configuration for a phase."""
    data = DEFAULT_PHASE_CONFIGS.get(phase_code.upper())
    if data:
        return PhaseConfig.from_dict(data)
    return None
