"""Phase 2: Decision Making State Payload Validation.

Verifies that decision_making publishes complete state payloads with all required fields.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


def load_recorded_states():
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


def test_state_envelope_structure():
    """Verify all states have correct envelope structure."""
    states = load_recorded_states()

    assert len(states) > 0, "No states recorded"

    for i, state in enumerate(states):
        # Check envelope fields
        assert "state" in state, f"State {i} missing 'state'"
        assert "gameState" in state, f"State {i} missing 'gameState'"
        assert "sessionId" in state, f"State {i} missing 'sessionId'"
        assert "transactionId" in state, f"State {i} missing 'transactionId'"
        assert "payload" in state, f"State {i} missing 'payload'"

        print(f"✓ State {i}: {state['state']}/{state.get('gameState', 'N/A')}")

    print(f"\n✅ Validated {len(states)} state envelopes")


def test_question_present_payload():
    """Verify QUESTION_PRESENT payloads have complete data."""
    states = load_recorded_states()

    question_states = [s for s in states if s.get("gameState") == "QUESTION_PRESENT"]

    assert len(question_states) > 0, "No QUESTION_PRESENT states found"

    for state in question_states:
        payload = state.get("payload", {})

        assert "question" in payload, "Missing question object"

        question = payload["question"]
        assert "questionId" in question, "Missing questionId"
        assert "prompt" in question, "Missing prompt"
        assert "questionType" in question, "Missing questionType"
        assert "options" in question, "Missing options"

        # Verify options structure
        options = question["options"]
        assert isinstance(options, list), "options should be a list"

        include_correct = os.environ.get("DECISION_MAKING_INCLUDE_CORRECT_OPTIONS", "").strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

        for opt in options:
            assert "id" in opt, "Option missing id"
            assert "label" in opt, "Option missing label"
            if include_correct:
                assert "correct" in opt, "Option missing correct flag"

        print(f"✓ Question {question['questionId']}: {len(options)} options")

    print(f"\n✅ Validated {len(question_states)} QUESTION_PRESENT payloads")


def test_fail_l1_payload():
    """Verify FAIL_L1 payloads have hint data."""
    states = load_recorded_states()

    fail_states = [s for s in states if s.get("gameState") == "FAIL_L1"]

    if len(fail_states) == 0:
        pytest.skip("No FAIL_L1 states found")

    for state in fail_states:
        payload = state.get("payload", {})

        assert "hint" in payload, "Missing hint"
        assert "action" in payload, "Missing action"
        # correctOptionId is optional but recommended

        print(f"✓ FAIL_L1: hint='{payload['hint'][:30]}...', action={payload['action']}")

    print(f"\n✅ Validated {len(fail_states)} FAIL_L1 payloads")


def test_transaction_id_increments():
    """Verify transaction IDs increment correctly."""
    states = load_recorded_states()

    transaction_ids = [s.get("transactionId") for s in states]

    # Remove None values
    transaction_ids = [tid for tid in transaction_ids if tid is not None]

    assert len(transaction_ids) > 0, "No transaction IDs found"

    # Check for duplicates (same transaction ID in consecutive states is OK)
    print(f"Transaction ID range: {min(transaction_ids)} - {max(transaction_ids)}")
    print(f"Unique transaction IDs: {len(set(transaction_ids))}")

    # Verify IDs are incrementing (allowing repeats)
    prev_tid = -1
    for tid in transaction_ids:
        assert tid >= prev_tid, f"Transaction ID should not decrease: {prev_tid} -> {tid}"
        prev_tid = tid

    print("✅ Transaction IDs increment correctly")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
