"""Mock Generic UI service for isolated testing.

This mock simulates the generic_ui backend by:
1. Providing the UpdateManifest and GetManifest services
2. Validating manifest structure
3. Applying JSON patch operations
4. Computing manifest hashes
"""

import hashlib
import json
from typing import Any, Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from generic_ui_interfaces.srv import GetManifest, UpdateManifest
    HAS_MANIFEST_SERVICES = True
except ImportError:
    # Define a minimal mock service for testing
    HAS_MANIFEST_SERVICES = False

    class UpdateManifest:
        """Mock UpdateManifest service definition."""

        class Request:
            def __init__(self):
                self.operation = ""
                self.json_payload = ""

        class Response:
            def __init__(self):
                self.success = False
                self.message = ""
                self.manifest_hash = ""

    class GetManifest:
        """Mock GetManifest service definition."""

        class Request:
            pass

        class Response:
            def __init__(self):
                self.manifest_json = "{}"
                self.manifest_hash = ""


class MockGenericUI(Node):
    """Mock UI service that validates manifest operations."""

    def __init__(self):
        super().__init__("mock_generic_ui")

        # Current manifest
        self._manifest: Dict[str, Any] = {}

        # Services
        self._update_service = self.create_service(
            UpdateManifest,
            "/generic_ui/update_manifest",
            self._handle_update_manifest,
        )
        self._get_service = self.create_service(
            GetManifest,
            "/generic_ui/get_manifest",
            self._handle_get_manifest,
        )

        self.get_logger().info("Mock Generic UI service initialized")

    def _handle_update_manifest(
        self,
        request: UpdateManifest.Request,
        response: UpdateManifest.Response,
    ) -> UpdateManifest.Response:
        """Handle UpdateManifest service requests."""
        try:
            payload = json.loads(request.json_payload)
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f"Invalid JSON: {e}"
            response.manifest_hash = ""
            return response

        operation = request.operation.lower()

        self.get_logger().info(f"Received {operation} request")

        if operation == "set":
            return self._handle_set(payload, response)
        elif operation == "patch":
            return self._handle_patch(payload, response)
        else:
            response.success = False
            response.message = f"Unknown operation: {operation}"
            response.manifest_hash = ""
            return response

    def _handle_get_manifest(
        self,
        request: GetManifest.Request,
        response: GetManifest.Response,
    ) -> GetManifest.Response:
        """Return current manifest and hash."""
        _ = request
        response.manifest_json = json.dumps(self._manifest, ensure_ascii=False)
        response.manifest_hash = self._compute_manifest_hash(self._manifest)
        return response

    def _handle_set(
        self,
        manifest: Dict[str, Any],
        response: UpdateManifest.Response,
    ) -> UpdateManifest.Response:
        """Handle 'set' operation - replace entire manifest."""
        # Validate manifest structure
        if not isinstance(manifest, dict):
            response.success = False
            response.message = "Manifest must be a dict"
            response.manifest_hash = ""
            return response

        # Store manifest
        self._manifest = manifest

        # Compute hash
        hash_hex = self._compute_manifest_hash(manifest)

        response.success = True
        response.message = "Manifest set successfully"
        response.manifest_hash = hash_hex

        self.get_logger().info(f"Manifest set. Hash: {hash_hex}")

        return response

    def _handle_patch(
        self,
        patches: Any,
        response: UpdateManifest.Response,
    ) -> UpdateManifest.Response:
        """Handle 'patch' operation - apply JSON patches."""
        if not isinstance(patches, list):
            response.success = False
            response.message = "Patches must be a list"
            response.manifest_hash = ""
            return response

        # Validate patches (basic validation)
        for patch in patches:
            if not isinstance(patch, dict):
                response.success = False
                response.message = "Each patch must be a dict"
                response.manifest_hash = ""
                return response

            if "op" not in patch or "path" not in patch:
                response.success = False
                response.message = "Patch must have 'op' and 'path'"
                response.manifest_hash = ""
                return response

        # Apply patches
        try:
            for patch in patches:
                self._apply_patch(patch)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            response.success = False
            response.message = f"Patch failed: {exc}"
            response.manifest_hash = self._compute_manifest_hash(self._manifest)
            return response

        # Compute hash
        hash_hex = self._compute_manifest_hash(self._manifest)

        response.success = True
        response.message = f"Applied {len(patches)} patch(es)"
        response.manifest_hash = hash_hex

        return response

    def _compute_manifest_hash(self, manifest: Dict[str, Any]) -> str:
        """Compute stable short hash for manifest."""
        manifest_json = json.dumps(manifest, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
        return hashlib.sha256(manifest_json.encode()).hexdigest()[:16]

    def _apply_patch(self, patch: Dict[str, Any]) -> None:
        """Apply a single JSON patch operation to the in-memory manifest."""
        op = str(patch["op"]).lower()
        path = str(patch["path"])
        value = patch.get("value")
        tokens = self._decode_pointer(path)

        self.get_logger().info(f"Applying patch: {op} {path}")

        if op == "add":
            self._set_pointer(tokens, value, create_missing=True, allow_insert=True)
            return
        if op == "replace":
            self._set_pointer(tokens, value, create_missing=False, allow_insert=False)
            return
        if op == "remove":
            self._remove_pointer(tokens)
            return
        raise ValueError(f"Unsupported patch op: {op}")

    def _decode_pointer(self, path: str) -> List[str]:
        """Decode RFC-6901 JSON pointer path."""
        if path == "":
            return []
        if not path.startswith("/"):
            raise ValueError(f"Invalid patch path: {path}")
        raw_tokens = path.lstrip("/").split("/")
        return [token.replace("~1", "/").replace("~0", "~") for token in raw_tokens]

    def _resolve_parent(self, tokens: List[str], create_missing: bool) -> tuple[Any, str]:
        """Resolve parent container and leaf token for a pointer."""
        if not tokens:
            return None, ""

        current: Any = self._manifest
        for token in tokens[:-1]:
            if isinstance(current, list):
                index = int(token)
                if index < 0 or index >= len(current):
                    raise IndexError(f"List index out of bounds: {index}")
                current = current[index]
                continue

            if not isinstance(current, dict):
                raise TypeError(f"Cannot traverse into type {type(current).__name__}")

            if token not in current:
                if not create_missing:
                    raise KeyError(f"Missing path segment: {token}")
                current[token] = {}
            current = current[token]

        return current, tokens[-1]

    def _set_pointer(
        self,
        tokens: List[str],
        value: Any,
        create_missing: bool,
        allow_insert: bool,
    ) -> None:
        """Set/add value at JSON pointer."""
        if not tokens:
            if not isinstance(value, dict):
                raise ValueError("Root manifest must be a dict")
            self._manifest = value
            return

        parent, leaf = self._resolve_parent(tokens, create_missing=create_missing)
        if isinstance(parent, list):
            if leaf == "-" and allow_insert:
                parent.append(value)
                return
            index = int(leaf)
            if allow_insert:
                if index < 0 or index > len(parent):
                    raise IndexError(f"List insert index out of bounds: {index}")
                parent.insert(index, value)
            else:
                if index < 0 or index >= len(parent):
                    raise IndexError(f"List index out of bounds: {index}")
                parent[index] = value
            return

        if not isinstance(parent, dict):
            raise TypeError(f"Cannot assign into type {type(parent).__name__}")
        if not allow_insert and leaf not in parent:
            raise KeyError(f"Missing key for replace: {leaf}")
        parent[leaf] = value

    def _remove_pointer(self, tokens: List[str]) -> None:
        """Remove value at JSON pointer."""
        if not tokens:
            self._manifest = {}
            return

        parent, leaf = self._resolve_parent(tokens, create_missing=False)
        if isinstance(parent, list):
            index = int(leaf)
            if index < 0 or index >= len(parent):
                raise IndexError(f"List index out of bounds: {index}")
            del parent[index]
            return
        if not isinstance(parent, dict):
            raise TypeError(f"Cannot remove from type {type(parent).__name__}")
        if leaf not in parent:
            raise KeyError(f"Missing key for remove: {leaf}")
        del parent[leaf]


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MockGenericUI()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
