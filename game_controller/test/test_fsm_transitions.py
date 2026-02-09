"""Phase 2: FSM State Transition Validation.

Verifies that all expected state transitions occur correctly in the FSM.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest


def load_recorded_states() -> list[dict[str, Any]]:
    """Load states from integration test results."""
    # Path relative to game_controller/game_controller/test/
    log_path = Path(__file__).parent.parent.parent / "test" / "results" / "decision_state_log.jsonl"

    if not log_path.exists():
        pytest.skip("No decision state log found. Run integration tests first.")

    with open(log_path, "r") as f:
        states = []
        for line in f:
            if line.strip():
                record = json.loads(line)
                # Extract the state object from the log record
                if "state" in record:
                    states.append(record["state"])
                else:
                    states.append(record)
        return states


def find_transitions(states: list[dict[str, Any]], from_state: str, to_state: str) -> list[tuple[int, int]]:
    """Find all transitions from one state to another."""
    transitions = []

    for i in range(len(states) - 1):
        current = states[i]
        next_s = states[i + 1]

        if current.get("gameState") == from_state and next_s.get("gameState") == to_state:
            transitions.append((i, i + 1))

    return transitions


def test_phase_intro_to_question_present():
    """Verify PHASE_INTRO transitions to QUESTION_PRESENT."""
    states = load_recorded_states()

    transitions = find_transitions(states, "PHASE_INTRO", "QUESTION_PRESENT")

    assert len(transitions) > 0, "No PHASE_INTRO -> QUESTION_PRESENT transitions found"

    print(f"✓ Found {len(transitions)} PHASE_INTRO -> QUESTION_PRESENT transitions")


def test_question_present_to_wait_input():
    """Verify QUESTION_PRESENT transitions to WAIT_INPUT."""
    states = load_recorded_states()

    transitions = find_transitions(states, "QUESTION_PRESENT", "WAIT_INPUT")

    assert len(transitions) > 0, "No QUESTION_PRESENT -> WAIT_INPUT transitions found"

    print(f"✓ Found {len(transitions)} QUESTION_PRESENT -> WAIT_INPUT transitions")


def test_wait_input_to_correct_or_fail():
    """Verify WAIT_INPUT transitions to CORRECT or FAIL_L1."""
    states = load_recorded_states()

    wait_states = [s for s in states if s.get("gameState") == "WAIT_INPUT"]

    for i, wait_state in enumerate(wait_states):
        # Find next state
        wait_idx = states.index(wait_state)
        if wait_idx + 1 < len(states):
            next_state = states[wait_idx + 1]
            next_game_state = next_state.get("gameState")

            assert next_game_state in ["CORRECT", "FAIL_L1", "WAIT_INPUT"], \
                f"WAIT_INPUT should transition to CORRECT, FAIL_L1, or stay in WAIT_INPUT, got {next_game_state}"

            print(f"  WAIT_INPUT -> {next_game_state}")


def test_fail_l1_allows_retry():
    """Verify FAIL_L1 can transition back to WAIT_INPUT."""
    states = load_recorded_states()

    transitions = find_transitions(states, "FAIL_L1", "WAIT_INPUT")

    if len(transitions) == 0:
        pytest.skip("No FAIL_L1 -> WAIT_INPUT transitions found (no failures in test?)")

    print(f"✓ Found {len(transitions)} FAIL_L1 -> WAIT_INPUT transitions (retry)")


def test_correct_advances_to_next_question():
    """Verify CORRECT transitions to next question or phase complete."""
    states = load_recorded_states()

    correct_states = [s for s in states if s.get("gameState") == "CORRECT"]

    for correct_state in correct_states:
        correct_idx = states.index(correct_state)
        if correct_idx + 1 < len(states):
            next_state = states[correct_idx + 1]
            next_game_state = next_state.get("gameState")

            assert next_game_state in ["QUESTION_PRESENT", "PHASE_COMPLETE", "ROUND_SETUP"], \
                f"CORRECT should transition to QUESTION_PRESENT, PHASE_COMPLETE, or ROUND_SETUP, got {next_game_state}"

            print(f"  CORRECT -> {next_game_state}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
