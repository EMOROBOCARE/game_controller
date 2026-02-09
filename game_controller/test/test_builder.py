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


class TestBuildPhaseConfigs:
    """Test phase config building."""
    
    def test_default_configs(self):
        configs = build_phase_configs(["P1", "P2"])
        assert "P1" in configs
        assert "P2" in configs
        assert configs["P1"]["interactionType"] == "matching"
        assert configs["P2"]["interactionType"] == "voice"
    
    def test_merge_with_game_configs(self):
        game_configs = {
            "P1": {"maxFailures": 3},
        }
        configs = build_phase_configs(["P1"], game_configs)
        assert configs["P1"]["maxFailures"] == 3
        assert configs["P1"]["interactionType"] == "matching"

    def test_merge_with_lowercase_phase_key(self):
        game_configs = {
            "p2": {"maxFailures": 5},
        }
        configs = build_phase_configs(["p2"], game_configs)
        assert configs["P2"]["maxFailures"] == 5
        assert "config" in configs["P2"]


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
        payload = build_game_init_payload(game_content, phases=["P7"])
        assert payload["phaseSequence"] == ["P1"]

    def test_p4_yesno_generates_two_subrounds(self):
        game_content = {
            "slug": "test",
            "title": "Test",
            "intro": "Test",
            "supportedPhases": ["P4_YESNO"],
            "answerType": "animals",
        }
        payload = build_game_init_payload(game_content, phases=["P4_YESNO"], rounds_per_phase=2)
        assert len(payload["rounds"]) == 4
