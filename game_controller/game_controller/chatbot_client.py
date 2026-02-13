"""Optional chatbot service client wrappers.

This module integrates with:
- `/chatbot/rephrase` (`chatbot_msgs/srv/Rephrase`)
- `/chatbot/evaluate_answer` (`chatbot_msgs/srv/EvaluateAnswer`)

The implementation is best-effort. If message/service types are unavailable,
or services are down, callers can fallback to deterministic local logic.
"""

from __future__ import annotations

from typing import Any, Callable, Optional, Sequence

try:  # pragma: no cover - depends on ROS environment
    from chatbot_msgs.srv import Rephrase, EvaluateAnswer
except Exception:  # pragma: no cover - depends on ROS environment
    Rephrase = None  # type: ignore[assignment]
    EvaluateAnswer = None  # type: ignore[assignment]


RephraseDone = Callable[[str], None]
EvaluateDone = Callable[[Optional[bool], str], None]


def normalize_evaluate_label(label: str) -> Optional[bool]:
    """Map chatbot EvaluateAnswer labels to booleans.

    Returns:
        True for correct, False for incorrect, None for unknown labels.
    """
    raw = str(label or "").strip().lower()
    if raw in {"ac", "answer-correct", "correct", "true", "1"}:
        return True
    if raw in {"ai", "answer-incorrect", "incorrect", "false", "0"}:
        return False
    return None


class ChatbotClient:
    """Thin async wrapper around optional chatbot ROS services."""

    def __init__(
        self,
        node: Any,
        rephrase_service: str = "/chatbot/rephrase",
        evaluate_service: str = "/chatbot/evaluate_answer",
        service_wait_timeout_sec: float = 0.05,
        callback_group: Optional[Any] = None,
        logger: Optional[Any] = None,
    ) -> None:
        self._node = node
        self._logger = logger
        self._service_wait_timeout_sec = float(service_wait_timeout_sec or 0.05)
        self._rephrase_client = None
        self._evaluate_client = None

        if Rephrase is not None:
            self._rephrase_client = node.create_client(
                Rephrase,
                str(rephrase_service or "/chatbot/rephrase"),
                callback_group=callback_group,
            )

        if EvaluateAnswer is not None:
            self._evaluate_client = node.create_client(
                EvaluateAnswer,
                str(evaluate_service or "/chatbot/evaluate_answer"),
                callback_group=callback_group,
            )

    @property
    def rephrase_available(self) -> bool:
        client = self._rephrase_client
        if client is None:
            return False
        try:
            return bool(client.wait_for_service(timeout_sec=0.0))
        except Exception:
            return False

    @property
    def evaluate_available(self) -> bool:
        client = self._evaluate_client
        if client is None:
            return False
        try:
            return bool(client.wait_for_service(timeout_sec=0.0))
        except Exception:
            return False

    def rephrase(
        self,
        sentence: str,
        done: RephraseDone,
        forbidden_expressions: Optional[Sequence[str]] = None,
        n_alternatives: int = 3,
    ) -> bool:
        """Request sentence rephrasing asynchronously.

        Calls `done(text)` with either a selected alternative or a safe fallback.
        Returns True if async request started; False if unavailable.
        """
        if self._rephrase_client is None or Rephrase is None:
            return False

        try:
            if not self._rephrase_client.wait_for_service(
                timeout_sec=self._service_wait_timeout_sec
            ):
                return False
        except Exception:
            return False

        request = Rephrase.Request()
        request.sentence = str(sentence or "")
        request.forbidden_expressions = [
            str(expr) for expr in (forbidden_expressions or []) if str(expr).strip()
        ]
        request.n_alternatives = int(max(1, n_alternatives))

        try:
            future = self._rephrase_client.call_async(request)
        except Exception:
            return False

        original = str(sentence or "")

        def _on_response(fut: Any) -> None:  # pragma: no cover - ROS callback
            fallback = original
            try:
                result = fut.result()
                if bool(getattr(result, "success", False)):
                    alternatives = list(getattr(result, "alternatives", []) or [])
                    chosen = None
                    for alt in alternatives:
                        candidate = str(alt or "").strip()
                        if candidate and candidate.lower() != original.strip().lower():
                            chosen = candidate
                            break
                    done(chosen or fallback)
                    return

                alternatives = list(getattr(result, "alternatives", []) or [])
                if alternatives:
                    fallback = str(alternatives[0] or fallback)
            except Exception as exc:
                if self._logger:
                    self._logger.debug(f"Rephrase call failed: {exc}")

            done(fallback)

        future.add_done_callback(_on_response)
        return True

    def evaluate_answer(
        self,
        question: str,
        expected_answer: str,
        answer: str,
        done: EvaluateDone,
    ) -> bool:
        """Request semantic answer evaluation asynchronously.

        Calls `done(is_correct, raw_label)` where `is_correct` can be None for
        unknown labels / failures.
        """
        if self._evaluate_client is None or EvaluateAnswer is None:
            return False

        try:
            if not self._evaluate_client.wait_for_service(
                timeout_sec=self._service_wait_timeout_sec
            ):
                return False
        except Exception:
            return False

        request = EvaluateAnswer.Request()
        request.question = str(question or "")
        request.expected_answer = str(expected_answer or "")
        request.answer = str(answer or "")

        try:
            future = self._evaluate_client.call_async(request)
        except Exception:
            return False

        def _on_response(fut: Any) -> None:  # pragma: no cover - ROS callback
            raw_label = ""
            try:
                result = fut.result()
                raw_label = str(getattr(result, "label", "") or "")
                done(normalize_evaluate_label(raw_label), raw_label)
                return
            except Exception as exc:
                if self._logger:
                    self._logger.debug(f"EvaluateAnswer call failed: {exc}")
            done(None, raw_label)

        future.add_done_callback(_on_response)
        return True
