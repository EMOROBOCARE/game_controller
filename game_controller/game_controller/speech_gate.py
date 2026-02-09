"""Speech gating helpers for UI-facing state transitions.

In this architecture, decision_making advances the session only after receiving
an ON_COMPLETE event for the current transactionId. For states that require
"presentation" (e.g., QUESTION_PRESENT), game_controller is responsible for:
1) triggering the robot/UI presentation (e.g., expressive TTS action), then
2) emitting ON_COMPLETE when that presentation finishes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Set

DoneCallback = Callable[[bool], None]
SpeakFn = Callable[[str, str, DoneCallback], bool]
OnCompleteFn = Callable[[int], None]


@dataclass(frozen=True)
class SpeechRequest:
    transaction_id: int
    text: str
    language: str


class SpeechGate:
    """Ensure each transaction triggers at most one presentation+completion."""

    def __init__(
        self,
        speak: SpeakFn,
        on_complete: OnCompleteFn,
        logger: Optional[Any] = None,
    ) -> None:
        self._speak = speak
        self._on_complete = on_complete
        self._logger = logger

        self._inflight_tx: Optional[int] = None
        self._completed: Set[int] = set()

    def reset(self) -> None:
        self._inflight_tx = None
        self._completed.clear()

    def maybe_speak(self, request: SpeechRequest) -> bool:
        """Start speaking for this tx, and publish ON_COMPLETE when finished.

        Returns True if a speech action was started (and ON_COMPLETE will be
        emitted from its completion callback). Returns False if this request
        is not applicable, duplicated, or speech could not be started.
        """
        tx = int(request.transaction_id)
        if tx <= 0:
            return False

        text = str(request.text or "").strip()
        if not text:
            return False

        if tx in self._completed or self._inflight_tx == tx:
            return False

        language = str(request.language or "").strip() or "es"

        def _done(success: bool) -> None:
            self._completed.add(tx)
            if self._inflight_tx == tx:
                self._inflight_tx = None
            if self._logger:
                self._logger.info(f"Speech done (tx={tx}, success={bool(success)})")
            try:
                self._on_complete(tx)
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Failed to publish ON_COMPLETE for tx={tx}: {exc}")

        started = False
        try:
            started = bool(self._speak(text, language, _done))
        except Exception as exc:
            if self._logger:
                self._logger.error(f"Speech start failed for tx={tx}: {exc}")
            started = False

        if not started:
            return False

        self._inflight_tx = tx
        if self._logger:
            self._logger.info(f"Speech started (tx={tx})")
        return True

