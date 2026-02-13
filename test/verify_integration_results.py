"""Verify integration test generated expected outputs.

This script validates that integration tests produce all required output files
and that the data meets minimum quality thresholds.
"""

import json
import os
from pathlib import Path


def verify_integration_results():
    """Verify integration test generated expected outputs."""

    results_dir = Path("test/results")

    # Check required files exist
    required_files = [
        "decision_state_log.jsonl",
        "decision_event_log.jsonl",
        "manifest_log.jsonl",
        "topic_audit_log.jsonl",
        "topic_action_stats.json",
        "expressive_say_log.jsonl",
        "manifest_analysis.md",
        "report.md"
    ]

    print("Checking required files...")
    for filename in required_files:
        filepath = results_dir / filename
        assert filepath.exists(), f"Missing: {filepath}"

        size = filepath.stat().st_size
        assert size > 0, f"Empty file: {filepath}"

        print(f"✓ {filename} ({size:,} bytes)")

    # Count records
    print("\nCounting records...")

    with open(results_dir / "decision_state_log.jsonl") as f:
        state_count = sum(1 for line in f if line.strip())
    print(f"✓ Decision states: {state_count}")

    with open(results_dir / "decision_event_log.jsonl") as f:
        event_count = sum(1 for line in f if line.strip())
    print(f"✓ Decision events: {event_count}")

    with open(results_dir / "manifest_log.jsonl") as f:
        manifest_count = sum(1 for line in f if line.strip())
    print(f"✓ Manifest updates: {manifest_count}")

    with open(results_dir / "topic_audit_log.jsonl") as f:
        topic_audit_count = sum(1 for line in f if line.strip())
    print(f"✓ Topic audit rows: {topic_audit_count}")

    with open(results_dir / "expressive_say_log.jsonl") as f:
        tts_goal_count = sum(1 for line in f if line.strip())
    print(f"✓ expressive_say goals: {tts_goal_count}")

    # Verify minimum counts
    assert state_count >= 50, f"Expected at least 50 states, got {state_count}"
    assert event_count >= 40, f"Expected at least 40 events, got {event_count}"
    assert manifest_count >= 40, f"Expected at least 40 manifest updates, got {manifest_count}"
    assert topic_audit_count >= 40, f"Expected at least 40 topic audit rows, got {topic_audit_count}"
    assert tts_goal_count >= 5, f"Expected at least 5 expressive_say goals, got {tts_goal_count}"

    with open(results_dir / "topic_action_stats.json") as f:
        topic_stats = json.load(f)
    observed_event_types = set(topic_stats.get("decision_event_types", []))
    for expected in ("GAME_INIT", "USER_INTENT", "ON_COMPLETE", "GAME_CONTROL"):
        assert expected in observed_event_types, f"Missing expected decision event type in stats: {expected}"

    tts_stats = topic_stats.get("tts_action", {})
    assert int(tts_stats.get("goals_count", 0)) >= int(tts_stats.get("question_present_count", 0)), (
        "expressive_say goals should cover QUESTION_PRESENT states"
    )

    # Load analysis
    print("\nLoading analysis...")
    analysis_path = results_dir / "manifest_analysis.json"
    if analysis_path.exists():
        with open(analysis_path) as f:
            analysis = json.load(f)

        print(f"✓ Unique manifest hashes: {analysis.get('unique_manifest_hashes', 0)}")
        print(f"✓ States covered: {len(analysis.get('states_by_type', {}))}")

        # Check for anomalies
        anomalies = analysis.get("anomalies", [])
        if anomalies:
            print(f"\n⚠ Found {len(anomalies)} anomalies:")
            for anomaly in anomalies:
                print(f"  - {anomaly}")
        else:
            print("\n✅ No anomalies detected")
    else:
        print("⚠ manifest_analysis.json not found (optional)")

    print("\n✅ Integration test results validated")


if __name__ == "__main__":
    verify_integration_results()
