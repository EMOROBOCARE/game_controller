"""Generic UI manifest client.

Wrapper for calling the UpdateManifest service.
"""

from __future__ import annotations

import json
from typing import Any, Callable, Dict, List, Optional

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

try:
    from generic_ui_interfaces.srv import UpdateManifest
    HAS_MANIFEST_SERVICE = True
except ImportError:
    HAS_MANIFEST_SERVICE = False
    UpdateManifest = None  # type: ignore


class ManifestClient:
    """Client for updating generic_ui manifests."""
    
    def __init__(
        self,
        node: Node,
        service_name: str = "/generic_ui/update_manifest",
        timeout_sec: float = 5.0,
        callback_group: Optional[ReentrantCallbackGroup] = None,
    ) -> None:
        """Initialize the manifest client.
        
        Args:
            node: ROS2 node to create client on
            service_name: Name of UpdateManifest service
            timeout_sec: Timeout for service calls
            callback_group: Optional callback group for async operations
        """
        self._node = node
        self._service_name = service_name
        self._timeout_sec = timeout_sec
        self._client: Optional[Any] = None
        self._manifest_sent = False
        
        if not HAS_MANIFEST_SERVICE:
            node.get_logger().warn(
                "generic_ui_interfaces not available. "
                "Manifest updates will be disabled."
            )
            return
        
        self._client = node.create_client(
            UpdateManifest,
            service_name,
            callback_group=callback_group,
        )
    
    @property
    def available(self) -> bool:
        """Check if the manifest service is available."""
        return HAS_MANIFEST_SERVICE and self._client is not None
    
    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for the manifest service to become available.
        
        Args:
            timeout_sec: Timeout in seconds (uses default if None)
            
        Returns:
            True if service is available, False if timed out
        """
        if not self.available:
            return False
        
        timeout = timeout_sec if timeout_sec is not None else self._timeout_sec
        return self._client.wait_for_service(timeout_sec=timeout)
    
    def set_manifest(
        self,
        manifest: Dict[str, Any],
        on_response: Optional[Callable[[bool, str, str], None]] = None,
    ) -> bool:
        """Set the complete manifest (replaces current).
        
        Args:
            manifest: Full manifest dict
            on_response: Callback(success, message, hash) when response received
            
        Returns:
            True if request was sent, False if service unavailable
        """
        return self._send_request("set", manifest, on_response)
    
    def patch_manifest(
        self,
        patches: List[Dict[str, Any]],
        on_response: Optional[Callable[[bool, str, str], None]] = None,
    ) -> bool:
        """Apply JSON patches to the manifest.
        
        Args:
            patches: List of JSON patch operations
            on_response: Callback(success, message, hash) when response received
            
        Returns:
            True if request was sent, False if service unavailable
        """
        return self._send_request("patch", patches, on_response)
    
    def _send_request(
        self,
        operation: str,
        payload: Any,
        on_response: Optional[Callable[[bool, str, str], None]] = None,
    ) -> bool:
        """Send a manifest update request.
        
        Args:
            operation: "set" or "patch"
            payload: Manifest dict or patch array
            on_response: Optional callback for response
            
        Returns:
            True if request was sent
        """
        if not self.available:
            self._node.get_logger().warn(
                "Cannot send manifest update: service not available"
            )
            return False
        
        if not self._client.wait_for_service(timeout_sec=self._timeout_sec):
            self._node.get_logger().warn(
                f"Manifest service {self._service_name} not available "
                f"after {self._timeout_sec}s"
            )
            return False
        
        request = UpdateManifest.Request()
        request.operation = operation
        request.json_payload = json.dumps(payload, ensure_ascii=False)
        
        future = self._client.call_async(request)
        
        if on_response:
            future.add_done_callback(
                lambda f: self._handle_response(f, on_response)
            )
        else:
            future.add_done_callback(self._default_response_handler)
        
        self._node.get_logger().info(
            f"Sent manifest {operation} request"
        )
        return True
    
    def _handle_response(
        self,
        future: Any,
        callback: Callable[[bool, str, str], None],
    ) -> None:
        """Handle service response with callback."""
        try:
            response = future.result()
            callback(
                response.success,
                response.message,
                response.manifest_hash,
            )
        except Exception as e:
            self._node.get_logger().error(
                f"Error getting manifest response: {e}"
            )
            callback(False, str(e), "")
    
    def _default_response_handler(self, future: Any) -> None:
        """Default response handler that logs result."""
        try:
            response = future.result()
            if response.success:
                self._node.get_logger().info(
                    f"Manifest update successful. Hash: {response.manifest_hash}"
                )
            else:
                self._node.get_logger().error(
                    f"Manifest update failed: {response.message}"
                )
        except Exception as e:
            self._node.get_logger().error(
                f"Error sending manifest: {e}"
            )
