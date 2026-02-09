"""ROS2 capability profiles for games.

Profiles define the ROS2 interactions (topics, services, actions) that games
require. Each profile represents a capability provided by external components
(chatbot, vision, robot hardware, etc.).

Games declare their requirements via the `requires` field, and the controller
checks availability and applies fallbacks when services are unavailable.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Set


class FallbackStrategy(Enum):
    """How to handle unavailable profile capabilities."""
    NONE = "none"           # No fallback, profile is required
    SKIP = "skip"           # Skip commands silently
    STATIC = "static"       # Use static content from game JSON
    UI_ONLY = "ui_only"     # Use UI-based alternative
    UI_TEXT = "ui_text"     # Display text in UI instead
    LOG = "log"             # Log locally instead of publishing


@dataclass
class ROS2Endpoint:
    """A single ROS2 endpoint (topic, service, or action)."""
    name: str
    type: str
    purpose: str = ""


@dataclass
class ROS2Interactions:
    """Collection of ROS2 interactions for a profile."""
    subscribe: List[ROS2Endpoint] = field(default_factory=list)
    publish: List[ROS2Endpoint] = field(default_factory=list)
    service_clients: List[ROS2Endpoint] = field(default_factory=list)
    service_servers: List[ROS2Endpoint] = field(default_factory=list)
    action_clients: List[ROS2Endpoint] = field(default_factory=list)
    action_servers: List[ROS2Endpoint] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> ROS2Interactions:
        """Parse from profile JSON ros2 section."""
        def parse_endpoints(items: List[Dict]) -> List[ROS2Endpoint]:
            return [
                ROS2Endpoint(
                    name=item["name"],
                    type=item["type"],
                    purpose=item.get("purpose", "")
                )
                for item in items
            ]

        topics = data.get("topics", {})
        services = data.get("services", {})
        actions = data.get("actions", {})

        return cls(
            subscribe=parse_endpoints(topics.get("subscribe", [])),
            publish=parse_endpoints(topics.get("publish", [])),
            service_clients=parse_endpoints(services.get("client", [])),
            service_servers=parse_endpoints(services.get("server", [])),
            action_clients=parse_endpoints(actions.get("client", [])),
            action_servers=parse_endpoints(actions.get("server", [])),
        )


@dataclass
class Profile:
    """A capability profile defining required ROS2 interactions."""
    name: str
    description: str
    required: bool
    ros2: ROS2Interactions
    fallback_strategy: FallbackStrategy
    fallback_description: str = ""

    @classmethod
    def from_dict(cls, name: str, data: Dict[str, Any]) -> Profile:
        """Parse from profile JSON."""
        fallback = data.get("fallback") or {}
        strategy_str = fallback.get("strategy", "none") if fallback else "none"

        try:
            strategy = FallbackStrategy(strategy_str)
        except ValueError:
            strategy = FallbackStrategy.SKIP

        return cls(
            name=name,
            description=data.get("description", ""),
            required=data.get("required", False),
            ros2=ROS2Interactions.from_dict(data.get("ros2", {})),
            fallback_strategy=strategy,
            fallback_description=fallback.get("description", "") if fallback else "",
        )


@dataclass
class ProfileStatus:
    """Status of a profile's availability."""
    profile: Profile
    available: bool
    unavailable_endpoints: List[str] = field(default_factory=list)
    using_fallback: bool = False


class ProfileRegistry:
    """Registry for loading and managing capability profiles."""

    def __init__(self):
        self._profiles: Dict[str, Profile] = {}
        self._loaded = False

    def get_profiles_directory(self) -> str:
        """Get the path to the profiles directory."""
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory("game_controller")
            return os.path.join(pkg_share, "games", "profiles")
        except Exception:
            # Fallback for development
            current_dir = os.path.dirname(os.path.abspath(__file__))
            return os.path.join(current_dir, "..", "..", "games", "profiles")

    def load_profiles(self) -> None:
        """Load all profile definitions from JSON files."""
        if self._loaded:
            return

        profiles_dir = self.get_profiles_directory()
        if not os.path.isdir(profiles_dir):
            self._loaded = True
            return

        for filename in os.listdir(profiles_dir):
            if not filename.endswith(".json"):
                continue

            filepath = os.path.join(profiles_dir, filename)
            try:
                with open(filepath, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    name = data.get("profile", filename[:-5])
                    self._profiles[name] = Profile.from_dict(name, data)
            except (json.JSONDecodeError, OSError, KeyError):
                continue

        self._loaded = True

    def get_profile(self, name: str) -> Optional[Profile]:
        """Get a profile by name."""
        self.load_profiles()
        return self._profiles.get(name)

    def get_all_profiles(self) -> Dict[str, Profile]:
        """Get all loaded profiles."""
        self.load_profiles()
        return self._profiles.copy()

    def list_profile_names(self) -> List[str]:
        """List all available profile names."""
        self.load_profiles()
        return list(self._profiles.keys())


# Global registry instance
_registry = ProfileRegistry()


def get_profile(name: str) -> Optional[Profile]:
    """Get a profile by name."""
    return _registry.get_profile(name)


def get_all_profiles() -> Dict[str, Profile]:
    """Get all loaded profiles."""
    return _registry.get_all_profiles()


def list_profile_names() -> List[str]:
    """List all available profile names."""
    return _registry.list_profile_names()


def get_game_profiles(game_data: Dict[str, Any]) -> List[Profile]:
    """Get profiles required by a game.

    Args:
        game_data: Game JSON data with optional 'requires' field

    Returns:
        List of Profile objects (always includes 'general')
    """
    _registry.load_profiles()

    # Always include general profile
    required_names: Set[str] = {"general"}

    # Add game-specific requirements
    requires = game_data.get("requires", [])
    if isinstance(requires, list):
        required_names.update(requires)

    profiles = []
    for name in required_names:
        profile = _registry.get_profile(name)
        if profile:
            profiles.append(profile)

    return profiles


def get_all_ros2_endpoints(profiles: List[Profile]) -> ROS2Interactions:
    """Merge ROS2 interactions from multiple profiles.

    Args:
        profiles: List of profiles to merge

    Returns:
        Combined ROS2Interactions
    """
    combined = ROS2Interactions()

    for profile in profiles:
        combined.subscribe.extend(profile.ros2.subscribe)
        combined.publish.extend(profile.ros2.publish)
        combined.service_clients.extend(profile.ros2.service_clients)
        combined.service_servers.extend(profile.ros2.service_servers)
        combined.action_clients.extend(profile.ros2.action_clients)
        combined.action_servers.extend(profile.ros2.action_servers)

    return combined


class ProfileAvailabilityChecker:
    """Check availability of profile capabilities at runtime."""

    def __init__(self, node):
        """Initialize with ROS2 node for service/topic discovery.

        Args:
            node: ROS2 node instance
        """
        self._node = node
        self._available_services: Set[str] = set()
        self._available_topics: Set[str] = set()
        self._checked = False

    def refresh(self) -> None:
        """Refresh the list of available services and topics."""
        try:
            # Get available services
            service_names_and_types = self._node.get_service_names_and_types()
            self._available_services = {name for name, _ in service_names_and_types}

            # Get available topics
            topic_names_and_types = self._node.get_topic_names_and_types()
            self._available_topics = {name for name, _ in topic_names_and_types}

            self._checked = True
        except Exception:
            self._checked = False

    def check_profile(self, profile: Profile) -> ProfileStatus:
        """Check if a profile's capabilities are available.

        Args:
            profile: Profile to check

        Returns:
            ProfileStatus with availability info
        """
        if not self._checked:
            self.refresh()

        unavailable = []

        # Check service clients
        for endpoint in profile.ros2.service_clients:
            if endpoint.name not in self._available_services:
                unavailable.append(f"service:{endpoint.name}")

        # Check subscriptions (topics we need to exist)
        for endpoint in profile.ros2.subscribe:
            if endpoint.name not in self._available_topics:
                unavailable.append(f"topic:{endpoint.name}")

        available = len(unavailable) == 0
        using_fallback = not available and profile.fallback_strategy != FallbackStrategy.NONE

        return ProfileStatus(
            profile=profile,
            available=available,
            unavailable_endpoints=unavailable,
            using_fallback=using_fallback,
        )

    def check_game_profiles(
        self, game_data: Dict[str, Any]
    ) -> Dict[str, ProfileStatus]:
        """Check all profiles required by a game.

        Args:
            game_data: Game JSON data

        Returns:
            Dict mapping profile name to ProfileStatus
        """
        profiles = get_game_profiles(game_data)
        return {p.name: self.check_profile(p) for p in profiles}
