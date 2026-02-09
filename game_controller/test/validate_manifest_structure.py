"""Validate manifest structure conforms to expected schema.

Phase 1 test: Verify that the manifest builder generates valid manifest structures
with all required fields and proper component registry setup.
"""

import json
from game_controller.ui.manifest_builder import build_initial_manifest


def validate_manifest_structure():
    """Validate manifest conforms to expected schema."""
    manifest = build_initial_manifest(
        games=[{"slug": "test", "title": "Test"}],
        users=[{"id": "1", "name": "User"}]
    )

    # Check required top-level keys
    assert "version" in manifest
    assert "componentRegistry" in manifest
    assert "ops" in manifest
    assert "layout" in manifest
    assert "instances" in manifest
    assert "ui" in manifest

    # Check component registry
    registry = manifest["componentRegistry"]
    for comp_name, comp_def in registry.items():
        assert "url" in comp_def
        assert "scope" in comp_def
        assert "module" in comp_def
        print(f"✓ {comp_name}: {comp_def['module']}")

    # Check instances
    instances = manifest["instances"]
    for instance in instances:
        assert "id" in instance
        assert "component" in instance
        assert "config" in instance
        print(f"✓ Instance: {instance['id']} ({instance['component']})")

    # Check operations
    ops = manifest["ops"]
    for op_name, op_def in ops.items():
        assert "kind" in op_def
        assert "rosType" in op_def
        assert "topic" in op_def
        print(f"✓ Operation: {op_name} -> {op_def['topic']}")

    print("\n✅ Manifest structure validation passed")
    return manifest


if __name__ == "__main__":
    validate_manifest_structure()
