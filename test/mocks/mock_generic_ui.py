"""Mock Generic UI service for isolated testing.

This mock simulates the generic_ui backend by:
1. Providing the UpdateManifest service
2. Validating manifest structure
3. Computing manifest hashes
"""

import hashlib
import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node

try:
    from generic_ui_interfaces.srv import UpdateManifest
    HAS_MANIFEST_SERVICE = True
except ImportError:
    # Define a minimal mock service for testing
    HAS_MANIFEST_SERVICE = False

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


class MockGenericUI(Node):
    """Mock UI service that validates manifest operations."""

    def __init__(self):
        super().__init__("mock_generic_ui")

        # Current manifest
        self._manifest: Dict[str, Any] = {}

        # Service
        self._service = self.create_service(
            UpdateManifest,
            "/generic_ui/update_manifest",
            self._handle_update_manifest,
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
        manifest_json = json.dumps(manifest, sort_keys=True)
        hash_hex = hashlib.sha256(manifest_json.encode()).hexdigest()[:16]

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

        # Apply patches (simplified - just log them)
        for patch in patches:
            op = patch.get("op")
            path = patch.get("path")
            self.get_logger().info(f"Applying patch: {op} {path}")

        # Compute hash
        manifest_json = json.dumps(self._manifest, sort_keys=True)
        hash_hex = hashlib.sha256(manifest_json.encode()).hexdigest()[:16]

        response.success = True
        response.message = f"Applied {len(patches)} patch(es)"
        response.manifest_hash = hash_hex

        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MockGenericUI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
