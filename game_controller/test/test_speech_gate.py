"""Unit tests for SpeechGate (TTS-driven ON_COMPLETE)."""

from __future__ import annotations

from typing import Callable, List, Tuple

from game_controller.speech_gate import SpeechGate, SpeechRequest


def test_speech_gate_waits_for_done_callback():
    completions: List[Tuple[int, bool]] = []
    pending_done: List[Callable[[bool], None]] = []

    def speak(text: str, language: str, done: Callable[[bool], None]) -> bool:
        assert text == "hola"
        assert language == "es"
        pending_done.append(done)
        return True

    gate = SpeechGate(
        speak=speak,
        on_complete=lambda tx: completions.append((tx, True)),
    )

    assert gate.maybe_speak(SpeechRequest(transaction_id=7, text="hola", language="es")) is True
    assert completions == []
    assert len(pending_done) == 1

    pending_done[0](True)
    assert completions == [(7, True)]


def test_speech_gate_deduplicates_by_transaction():
    fired: List[int] = []
    pending_done: List[Callable[[bool], None]] = []

    def speak(text: str, language: str, done: Callable[[bool], None]) -> bool:
        pending_done.append(done)
        return True

    gate = SpeechGate(speak=speak, on_complete=lambda tx: fired.append(tx))

    assert gate.maybe_speak(SpeechRequest(transaction_id=3, text="a", language="es")) is True
    # Duplicate while inflight is ignored.
    assert gate.maybe_speak(SpeechRequest(transaction_id=3, text="a", language="es")) is False
    pending_done[0](True)
    assert fired == [3]
    # Duplicate after completion is ignored.
    assert gate.maybe_speak(SpeechRequest(transaction_id=3, text="a", language="es")) is False


def test_speech_gate_skips_empty_text():
    fired: List[int] = []

    def speak(text: str, language: str, done: Callable[[bool], None]) -> bool:
        raise AssertionError("speak() should not be called for empty text")

    gate = SpeechGate(speak=speak, on_complete=lambda tx: fired.append(tx))
    assert gate.maybe_speak(SpeechRequest(transaction_id=1, text="  ", language="es")) is False
    assert fired == []


def test_speech_gate_returns_false_if_speak_cannot_start():
    fired: List[int] = []

    def speak(text: str, language: str, done: Callable[[bool], None]) -> bool:
        return False

    gate = SpeechGate(speak=speak, on_complete=lambda tx: fired.append(tx))
    assert gate.maybe_speak(SpeechRequest(transaction_id=9, text="hola", language="es")) is False
    assert fired == []

