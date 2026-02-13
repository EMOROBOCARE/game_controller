"""Unit tests for GAME_INIT payload builder."""

import pytest
from game_controller.content.builder import (
    normalize_phase_list,
    build_phase_configs,
    build_game_init_payload,
)


class TestNormalizePhaseList:
    """Test phase list normalization."""
    
    def test_list_input(self):
        assert normalize_phase_list(["P1", "P2"]) == ["P1", "P2"]
    
    def test_string_input(self):
        assert normalize_phase_list("P1") == ["P1"]
    
    def test_lowercase_uppercase(self):
        assert normalize_phase_list(["p1", "p2"]) == ["P1", "P2"]
    
    def test_empty_input(self):
        assert normalize_phase_list([]) == []
        assert normalize_phase_list(None) == []

    def test_modern_p6_is_preserved(self):
        assert normalize_phase_list(["P5", "P6"]) == ["P5", "P6"]

    def test_legacy_numbering_maps_when_p7_present(self):
        assert normalize_phase_list(["P4_YESNO", "P6", "P7"]) == ["P4", "P5", "P6"]


class TestBuildPhaseConfigs:
    """Test phase config building."""

    def test_default_configs(self):
        configs = build_phase_configs(["P1", "P2"])
        assert "P1" in configs
        assert "P2" in configs
        assert "hints" in configs["P1"]
        assert "hints" in configs["P2"]

    def test_merge_with_game_configs(self):
        game_configs = {
            "P1": {"maxFailures": 3},
        }
        configs = build_phase_configs(["P1"], game_configs)
        assert configs["P1"]["maxFailures"] == 3
        assert "hints" in configs["P1"]

    def test_merge_with_lowercase_phase_key(self):
        game_configs = {
            "p2": {"maxFailures": 5},
        }
        configs = build_phase_configs(["p2"], game_configs)
        assert configs["P2"]["maxFailures"] == 5

    def test_legacy_phase_names_in_game_configs(self):
        """Legacy phase config keys (P4_YESNO, P6, P7) are normalized."""
        game_configs = {
            "P4_YESNO": {"maxFailures": 3},
            "P7": {"maxFailures": 4},
        }
        configs = build_phase_configs(["P4", "P6"], game_configs)
        assert configs["P4"]["maxFailures"] == 3
        assert configs["P6"]["maxFailures"] == 4


class TestBuildGameInitPayload:
    """Test GAME_INIT payload building."""
    
    def test_basic_payload(self):
        game_content = {
            "slug": "colores",
            "title": "Colores",
            "intro": "Vamos a jugar",
            "supportedPhases": ["P1", "P2"],
            "phaseConfig": {"P1": {"prompt": "Señala {colour}"}},
            "answerType": "colours",
        }
        
        payload = build_game_init_payload(
            game_content,
            phases=["P1"],
            difficulty="basic",
            rounds_per_phase=1,
        )
        
        assert payload["slug"] == "colores"
        assert payload["title"] == "Colores"
        assert payload["phaseSequence"] == ["P1"]
        assert len(payload["rounds"]) == 1
        assert len(payload["rounds"][0]["question"]["options"]) == 2
    
    def test_session_id_included(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "answerType": "animals",
        }
        
        payload = build_game_init_payload(
            game_content,
            session_id=42,
        )
        
        assert payload["sessionId"] == 42

    def test_default_phase_sequence(self):
        game_content = {
            "slug": "default",
            "title": "Default",
            "intro": "Default",
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content)
        assert payload["phaseSequence"] == ["P1", "P2", "P3"]

    def test_phase_config_override(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "phaseConfig": {"P1": {"maxFailures": 7}},
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content, phases=["P1"])
        assert payload["phaseConfigs"]["P1"]["maxFailures"] == 7

    def test_requested_phases_filtered_by_supported_phases(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P1"],
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content, phases=["P6"])
        assert payload["phaseSequence"] == ["P1"]

    def test_modern_p5_and_p6_are_not_collapsed(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P5", "P6"],
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content)
        assert payload["phaseSequence"] == ["P5", "P6"]

    def test_p5_rounds_always_include_answer_option(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P5"],
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content, phases=["P5"], rounds_per_phase=6)
        for round_entry in payload["rounds"]:
            question = round_entry["question"]
            option_ids = {str(opt.get("id")) for opt in question.get("options", [])}
            assert str(question.get("answer")) in option_ids

    def test_p4_generates_two_subrounds(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P4"],
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content, phases=["P4"], rounds_per_phase=2)
        assert len(payload["rounds"]) == 4

    def test_legacy_p4_yesno_maps_to_p4(self):
        """Legacy P4_YESNO in supportedPhases maps to P4."""
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P4_YESNO"],
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content, phases=["P4_YESNO"], rounds_per_phase=1)
        assert payload["phaseSequence"] == ["P4"]

    def test_round_question_has_prompt_fields(self):
        """Rounds include promptText and promptVerbal."""
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P3"],
            "answerType": "animals",
            "phaseConfig": {
                "P3": {
                    "text_instructions": "Mira bien.",
                    "verbal_instructions": "Escucha bien.",
                    "prompt": "Señala {animal}",
                }
            },
        }
        payload = build_game_init_payload(game_content, phases=["P3"], rounds_per_phase=1)
        question = payload["rounds"][0]["question"]
        assert "promptText" in question
        assert "promptVerbal" in question
        assert question["prompt"] == question["promptVerbal"]
        assert question["promptText"].startswith("Mira bien.")
        assert question["promptVerbal"].startswith("Escucha bien.")
