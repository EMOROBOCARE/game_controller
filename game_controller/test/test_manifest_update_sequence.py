"""Validate manifest snapshot log sequence.

The integration harness logs full snapshots in each JSONL row:
{ ts, step, decision, manifest_hash, manifest }.
"""

import json
import pytest
from pathlib import Path


def load_manifest_log():
    """Load manifest update log."""
    # Try multiple possible locations for the log
    possible_paths = [
        Path("test/results/manifest_log.jsonl"),
        Path("../test/results/manifest_log.jsonl"),
        Path("../../test/results/manifest_log.jsonl"),
    ]

    for log_path in possible_paths:
        if log_path.exists():
            with open(log_path) as f:
                return [json.loads(line) for line in f if line.strip()]

    pytest.skip("No manifest log found. Run integration test first.")


def test_initial_manifest_is_set():
    """Verify first snapshot contains a full manifest structure."""
    records = load_manifest_log()

    assert len(records) > 0, "No manifest updates found"

    first = records[0]
    manifest = first.get("manifest")

    assert isinstance(first.get("manifest_hash"), str), "First record missing manifest_hash"
    assert isinstance(manifest, dict), "First record missing manifest snapshot"

    assert "version" in manifest, "Initial manifest missing version"
    assert "componentRegistry" in manifest, "Initial manifest missing componentRegistry"
    assert "instances" in manifest, "Initial manifest missing instances"

    print("✓ Initial record has complete manifest structure")


def test_snapshots_include_hash_and_manifest():
    """Every logged record should include hash + snapshot payload."""
    records = load_manifest_log()
    for idx, rec in enumerate(records):
        assert isinstance(rec.get("manifest_hash"), str), f"Record {idx} missing manifest_hash"
        assert isinstance(rec.get("manifest"), dict), f"Record {idx} missing manifest object"


def test_hash_changes_exist():
    """Integration run should capture at least one manifest state transition."""
    records = load_manifest_log()
    hashes = [str(rec.get("manifest_hash") or "") for rec in records]
    unique_hashes = {h for h in hashes if h}
    assert len(unique_hashes) >= 2, "Expected at least two unique manifest hashes"
    print(f"✓ Unique manifest hashes: {len(unique_hashes)}")

def test_instances_stay_on_two_instance_contract():
    """Manifest snapshots should keep exactly user_panel + game_screen."""
    records = load_manifest_log()
    for idx, rec in enumerate(records):
        manifest = rec.get("manifest")
        assert isinstance(manifest, dict), f"Record {idx} has invalid manifest payload"
        instances = manifest.get("instances")
        assert isinstance(instances, list), f"Record {idx} missing instances list"
        ids = [inst.get("id") for inst in instances if isinstance(inst, dict)]
        assert ids == ["game_screen", "user_panel"], f"Record {idx} invalid instances order/content: {ids}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
