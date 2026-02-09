"""Test complete state coverage - verify all game states generate patches.

Phase 1 test: Ensure every game state from decision_making generates valid
manifest patches with correct structure.
"""

import pytest
from game_controller.ui.manifest_builder import build_state_based_patches

# All valid game states from decision_making
GAME_STATES = [
    "PHASE_INTRO",
    "ROUND_SETUP",
    "QUESTION_PRESENT",
    "WAIT_INPUT",
    "FAIL_L1",
    "FAIL_L2",
    "CORRECT",
    "PHASE_COMPLETE",
    "P6_HIGHLIGHT",
]


@pytest.mark.parametrize("game_state", GAME_STATES)
def test_state_generates_patches(game_state):
    """Test that each game state generates valid patches."""

    # Create appropriate payload for each state
    payload = create_test_payload_for_state(game_state)

    # Generate patches
    patches = build_state_based_patches(game_state, payload)

    # Verify patches is a list
    assert isinstance(patches, list), f"{game_state} should return list"

    # Verify each patch has required fields
    for i, patch in enumerate(patches):
        assert "op" in patch, f"{game_state} patch {i} missing 'op'"
        assert "path" in patch, f"{game_state} patch {i} missing 'path'"
        assert patch["op"] in ["replace", "add", "remove"], f"Invalid op: {patch['op']}"
        assert patch["path"].startswith("/instances/"), f"Invalid path: {patch['path']}"

        print(f"  {game_state} patch {i}: {patch['op']} {patch['path']}")

    print(f"✓ {game_state}: {len(patches)} patches generated")


def create_test_payload_for_state(state):
    """Create appropriate test payload for each state."""

    if state == "PHASE_INTRO":
        return {
            "phase": "P1",
            "introduction": "Test intro"
        }

    elif state == "QUESTION_PRESENT":
        return {
            "question": {
                "questionId": 1,
                "prompt": "Test question",
                "imageUrl": "assets/images/test.png",
                "questionType": "multiple_choice",
                "options": [
                    {"id": "1", "label": "Option 1", "imageUrl": "assets/opt1.png", "correct": True},
                    {"id": "2", "label": "Option 2", "imageUrl": "assets/opt2.png", "correct": False}
                ]
            }
        }

    elif state == "WAIT_INPUT":
        return {
            "roundId": 1,
            "questionId": 1
        }

    elif state == "FAIL_L1":
        return {
            "hint": "Try again",
            "action": "point",
            "correctOptionId": "1"
        }

    elif state == "FAIL_L2":
        return {
            "hint": "The answer is Option 1",
            "action": "highlight_and_solve",
            "correctOptionId": "1"
        }

    elif state == "CORRECT":
        return {}

    elif state == "PHASE_COMPLETE":
        return {
            "phase": "P1"
        }

    elif state == "P6_HIGHLIGHT":
        return {
            "highlighted_ids": ["2"],
            "options": [
                {"id": "1", "label": "Option 1"},
                {"id": "2", "label": "Option 2"}
            ],
            "response": "Aquí"
        }

    elif state == "ROUND_SETUP":
        return {}

    return {}


def test_patch_paths_are_consistent():
    """Verify all patches use consistent instance paths."""

    all_patches = []

    for state in GAME_STATES:
        payload = create_test_payload_for_state(state)
        patches = build_state_based_patches(state, payload)
        all_patches.extend(patches)

    # Extract all paths
    paths = [p["path"] for p in all_patches]

    # Verify all paths start with /instances/
    for path in paths:
        assert path.startswith("/instances/"), f"Invalid path: {path}"

    # Verify instance index is consistent
    instance_indices = set()
    for path in paths:
        parts = path.split("/")
        if len(parts) >= 3 and parts[1] == "instances":
            instance_indices.add(parts[2])

    print(f"Instance indices used: {instance_indices}")

    # Current implementation uses instance 0 (game_screen)
    assert "0" in instance_indices, "Should patch instance 0 (game_screen)"

    print("✓ All patch paths are consistent")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
