"""Unit tests for input translation helpers."""

from game_controller.input_translation import (
    InputTranslator,
    extract_control_command,
    extract_user_answer,
    is_control_command,
    parse_input_json,
    translate_input_to_game_control,
    translate_input_to_user_intent,
)


def test_parse_input_json_valid():
    assert parse_input_json('{"label": "rojo"}') == {"label": "rojo"}


def test_parse_input_json_invalid():
    assert parse_input_json("{") is None
    assert parse_input_json("[]") is None
    assert parse_input_json(None) is None


def test_control_command_detection():
    assert is_control_command({"label": " pause "}) is True
    assert is_control_command({"label": "RESUME"}) is True
    assert is_control_command({"label": "exit"}) is True
    assert is_control_command({"label": "nope"}) is False


def test_extract_control_command():
    assert extract_control_command({"label": "PAUSE"}) == "PAUSE"
    assert extract_control_command({"label": "BACK"}) == "EXIT"
    assert extract_control_command({"label": "other"}) is None


def test_extract_user_answer_prefers_label_then_value():
    value, correct = extract_user_answer({"label": "rojo", "correct": True})
    assert value == "rojo"
    assert correct is True

    value, correct = extract_user_answer({"value": "verde", "correct": False})
    assert value == "verde"
    assert correct is False

    value, correct = extract_user_answer({"answer": "azul"})
    assert value == "azul"
    assert correct is None


def test_translate_input_to_user_intent_uses_question_correctness():
    question = {
        "questionType": "yes_no",
        "meta": {"correct_answer": "si"},
    }
    event = translate_input_to_user_intent(
        {"label": "si"},
        transaction_id=7,
        current_question=question,
    )
    assert event is not None
    assert event["type"] == "USER_INTENT"
    assert event["payload"]["transactionId"] == 7
    assert event["payload"]["correct"] is True


def test_translate_input_to_user_intent_ignores_control():
    event = translate_input_to_user_intent({"label": "PAUSE"}, transaction_id=1)
    assert event is None


def test_translate_input_to_game_control():
    event = translate_input_to_game_control({"label": "PAUSE"})
    assert event == {"type": "GAME_CONTROL", "payload": {"command": "PAUSE"}}


def test_input_translator_gates_on_state():
    translator = InputTranslator()
    translator.update_state(10, game_state="PHASE_INTRO")
    assert translator.translate_input_data({"label": "rojo"}) is None

    translator.update_state(
        10,
        game_state="WAIT_INPUT",
        question={"questionType": "yes_no", "meta": {"correct_answer": "si"}},
    )
    event = translator.translate_input_data({"label": "si"})
    assert event is not None
    assert event["type"] == "USER_INTENT"
    assert event["payload"]["transactionId"] == 10


def test_input_translator_allows_control_any_state():
    translator = InputTranslator()
    translator.update_state(5, game_state="PHASE_INTRO")
    event = translator.translate_input_data({"label": "EXIT"})
    assert event == {"type": "GAME_CONTROL", "payload": {"command": "EXIT"}}
