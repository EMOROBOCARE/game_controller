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

    # Verify minimum counts
    assert state_count >= 50, f"Expected at least 50 states, got {state_count}"
    assert event_count >= 40, f"Expected at least 40 events, got {event_count}"
    assert manifest_count >= 40, f"Expected at least 40 manifest updates, got {manifest_count}"

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
