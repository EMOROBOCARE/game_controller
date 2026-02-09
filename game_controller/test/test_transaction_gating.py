"""Phase 2: Transaction ID Gating Validation.

Verifies that transaction IDs are properly tracked and gating works correctly.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest


def load_event_log() -> list[dict[str, Any]]:
    """Load decision event log."""
    # Path relative to game_controller/game_controller/test/
    log_path = Path(__file__).parent.parent.parent / "test" / "results" / "decision_event_log.jsonl"

    if not log_path.exists():
        pytest.skip("No event log found. Run integration tests first.")

    with open(log_path, "r") as f:
        events = []
        for line in f:
            if line.strip():
                record = json.loads(line)
                # Extract the event object from the log record
                if "event" in record:
                    events.append(record["event"])
                else:
                    events.append(record)
        return events


def load_state_log() -> list[dict[str, Any]]:
    """Load decision state log."""
    # Path relative to game_controller/game_controller/test/
    log_path = Path(__file__).parent.parent.parent / "test" / "results" / "decision_state_log.jsonl"

    if not log_path.exists():
        pytest.skip("No state log found. Run integration tests first.")

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


def test_user_intent_includes_transaction_id():
    """Verify USER_INTENT events include transactionId."""
    events = load_event_log()

    user_intents = [e for e in events if e.get("type") == "USER_INTENT"]

    assert len(user_intents) > 0, "No USER_INTENT events found"

    for i, event in enumerate(user_intents):
        payload = event.get("payload", {})

        assert "transactionId" in payload, f"USER_INTENT {i} missing transactionId"

        tid = payload["transactionId"]
        assert isinstance(tid, int), f"transactionId should be int, got {type(tid)}"
        assert tid >= 0, f"transactionId should be non-negative, got {tid}"

        print(f"✓ USER_INTENT {i}: transactionId={tid}")

    print(f"\n✅ All {len(user_intents)} USER_INTENT events have transactionId")


def test_on_complete_includes_transaction_id():
    """Verify ON_COMPLETE events include transactionId."""
    events = load_event_log()

    on_complete = [e for e in events if e.get("type") == "ON_COMPLETE"]

    assert len(on_complete) > 0, "No ON_COMPLETE events found"

    for i, event in enumerate(on_complete):
        payload = event.get("payload", {})

        assert "transactionId" in payload, f"ON_COMPLETE {i} missing transactionId"
        print(f"✓ ON_COMPLETE {i}: transactionId={payload['transactionId']}")

    print(f"\n✅ All {len(on_complete)} ON_COMPLETE events have transactionId")


def test_transaction_ids_are_sequential():
    """Verify transaction IDs increment sequentially."""
    states = load_state_log()

    transaction_ids = []
    for state in states:
        tid = state.get("transactionId")
        if tid is not None:
            transaction_ids.append(tid)

    assert len(transaction_ids) > 0, "No transaction IDs found"

    # Check for monotonic increase (allowing repeats for same state)
    prev = -1
    for tid in transaction_ids:
        assert tid >= prev, f"Transaction ID decreased: {prev} -> {tid}"
        prev = tid

    print(f"✓ Transaction IDs range from {min(transaction_ids)} to {max(transaction_ids)}")
    print(f"✓ {len(set(transaction_ids))} unique transaction IDs")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
