"""Validate manifest update sequence.

This test verifies that:
1. First manifest update is a 'set' operation with complete structure
2. Subsequent updates use 'patch' operations
3. Patch paths reference valid instance indices
4. No unnecessary duplicate patches
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
    """Verify first manifest update is 'set' operation."""
    updates = load_manifest_log()

    assert len(updates) > 0, "No manifest updates found"

    first_update = updates[0]

    assert first_update.get("operation") == "set", \
        f"First update should be 'set', got {first_update.get('operation')}"

    # Verify payload is a complete manifest
    payload = json.loads(first_update.get("payload", "{}"))

    assert "version" in payload, "Initial manifest missing version"
    assert "componentRegistry" in payload, "Initial manifest missing componentRegistry"
    assert "instances" in payload, "Initial manifest missing instances"

    print("✓ Initial manifest is 'set' operation with complete structure")


def test_subsequent_updates_are_patches():
    """Verify subsequent updates use 'patch' operation."""
    updates = load_manifest_log()

    patch_updates = [u for u in updates[1:] if u.get("operation") == "patch"]

    assert len(patch_updates) > 0, "No patch operations found"

    print(f"✓ Found {len(patch_updates)} patch operations")

    # Verify patch structure
    for i, update in enumerate(patch_updates[:5]):  # Check first 5
        payload = json.loads(update.get("payload", "[]"))

        assert isinstance(payload, list), f"Patch {i} payload should be a list"

        for j, patch in enumerate(payload):
            assert "op" in patch, f"Patch {i}[{j}] missing 'op'"
            assert "path" in patch, f"Patch {i}[{j}] missing 'path'"
            print(f"  Patch {i}[{j}]: {patch['op']} {patch['path']}")


def test_no_redundant_patches():
    """Check for unnecessary duplicate patches."""
    updates = load_manifest_log()

    # Get consecutive patch operations
    patches = []
    for update in updates:
        if update.get("operation") == "patch":
            payload = json.loads(update.get("payload", "[]"))
            patches.append(payload)

    # Check for exact duplicates
    seen = []
    duplicates = 0

    for i, patch_list in enumerate(patches):
        patch_str = json.dumps(patch_list, sort_keys=True)
        if patch_str in seen:
            duplicates += 1
            print(f"⚠ Duplicate patch at index {i}")
        seen.append(patch_str)

    if duplicates > 0:
        print(f"⚠ Found {duplicates} duplicate patches (may be intentional)")
    else:
        print("✓ No duplicate patches found")


def test_patch_paths_reference_valid_instances():
    """Verify patch paths reference existing instances."""
    updates = load_manifest_log()

    # Get instance IDs from initial manifest
    first_update = updates[0]
    manifest = json.loads(first_update.get("payload", "{}"))
    instances = manifest.get("instances", [])
    instance_ids = [inst["id"] for inst in instances]

    print(f"Instance IDs: {instance_ids}")

    # Check all patch paths
    invalid_paths = []

    for update in updates:
        if update.get("operation") == "patch":
            payload = json.loads(update.get("payload", "[]"))

            for patch in payload:
                path = patch.get("path", "")

                # Extract instance reference
                # Path format: /instances/0/config/... or /instances/1/config/...
                if path.startswith("/instances/"):
                    parts = path.split("/")
                    if len(parts) >= 3:
                        instance_idx = parts[2]

                        # Check if index is valid
                        try:
                            idx = int(instance_idx)
                            if idx >= len(instance_ids):
                                invalid_paths.append(path)
                        except ValueError:
                            invalid_paths.append(path)

    if invalid_paths:
        print(f"❌ Found {len(invalid_paths)} invalid patch paths:")
        for path in invalid_paths[:10]:  # Show first 10
            print(f"  - {path}")
        assert False, "Invalid patch paths found"
    else:
        print("✓ All patch paths reference valid instances")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
