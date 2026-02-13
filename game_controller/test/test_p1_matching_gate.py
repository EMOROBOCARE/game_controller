"""Unit tests for P1 matching aggregation and feedback gating."""

from __future__ import annotations

from game_controller.node import GameControllerNode


class _DummyLogger:
    def debug(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return None

    def info(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return None

    def warn(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return None

    def warning(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return None

    def error(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return None


def _build_node_stub() -> GameControllerNode:
    node = GameControllerNode.__new__(GameControllerNode)
    node._current_system_state = "GAME"
    node._current_phase = "P1"
    node._latest_transaction_id = 42
    node._latest_question_payload = {
        "questionId": 7,
        "options": [
            {"id": "rojo", "label": "Rojo"},
            {"id": "azul", "label": "Azul"},
        ],
    }
    node._p1_match_tracking_tx = None
    node._p1_expected_match_ids = set()
    node._p1_completed_match_ids = set()
    node._p1_match_finalized_tx = None
    node._tts_enabled = False
    node._tts_client = None
    node._tts_language = "es"
    node._ui_translator = type("TranslatorStub", (), {"_current_game_state": "WAIT_INPUT"})()
    node.get_logger = lambda: _DummyLogger()
    return node


def test_p1_matching_waits_until_all_matches_then_publishes() -> None:
    node = _build_node_stub()
    published: list[dict] = []
    spoken: list[str] = []

    node._publish_translated_input_event = lambda data, modality: published.append(dict(data))
    node._speak_p1_match_label = lambda match_id: spoken.append(match_id)

    handled = GameControllerNode._handle_p1_matching_input(
        node,
        {"leftId": "rojo", "rightId": "rojo", "correct": True},
        "touch",
    )
    assert handled is True
    assert published == []
    assert spoken == ["rojo"]

    handled = GameControllerNode._handle_p1_matching_input(
        node,
        {"leftId": "azul", "rightId": "azul", "correct": True},
        "touch",
    )
    assert handled is True
    assert len(published) == 1
    assert spoken == ["rojo", "azul"]


def test_p1_matching_incorrect_match_is_forwarded_immediately() -> None:
    node = _build_node_stub()
    published: list[dict] = []
    spoken: list[str] = []

    node._publish_translated_input_event = lambda data, modality: published.append(dict(data))
    node._speak_p1_match_label = lambda match_id: spoken.append(match_id)

    handled = GameControllerNode._handle_p1_matching_input(
        node,
        {"leftId": "rojo", "rightId": "azul", "correct": False},
        "touch",
    )
    assert handled is True
    assert len(published) == 1
    assert spoken == []


def test_p1_matching_honors_explicit_completion_flag() -> None:
    node = _build_node_stub()
    published: list[dict] = []

    node._publish_translated_input_event = lambda data, modality: published.append(dict(data))
    node._speak_p1_match_label = lambda match_id: None

    handled = GameControllerNode._handle_p1_matching_input(
        node,
        {
            "leftId": "rojo",
            "rightId": "rojo",
            "correct": True,
            "allMatched": True,
        },
        "touch",
    )
    assert handled is True
    assert len(published) == 1


def test_p1_matching_falls_back_when_expected_count_is_unknown() -> None:
    node = _build_node_stub()
    node._latest_question_payload = {"questionId": 7, "options": []}
    published: list[dict] = []

    node._publish_translated_input_event = lambda data, modality: published.append(dict(data))
    node._speak_p1_match_label = lambda match_id: None

    handled = GameControllerNode._handle_p1_matching_input(
        node,
        {"leftId": "rojo", "rightId": "rojo", "correct": True},
        "touch",
    )
    assert handled is True
    assert len(published) == 1
