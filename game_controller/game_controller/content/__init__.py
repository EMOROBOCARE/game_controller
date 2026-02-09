"""Content package for game loading and building."""

from .loaders import (
    load_game_content,
    list_available_games,
    get_game_metadata,
    get_all_games_metadata,
)
from .builder import build_game_init_payload
from .option_generation import (
    generate_color_options,
    generate_yes_no_options,
    generate_generic_options,
    get_options_count_for_difficulty,
)
from .profiles import (
    Profile,
    ProfileRegistry,
    ProfileStatus,
    ProfileAvailabilityChecker,
    FallbackStrategy,
    ROS2Interactions,
    get_profile,
    get_all_profiles,
    list_profile_names,
    get_game_profiles,
    get_all_ros2_endpoints,
)

__all__ = [
    "load_game_content",
    "list_available_games",
    "get_game_metadata",
    "get_all_games_metadata",
    "build_game_init_payload",
    "generate_color_options",
    "generate_yes_no_options",
    "generate_generic_options",
    "get_options_count_for_difficulty",
    # Profiles
    "Profile",
    "ProfileRegistry",
    "ProfileStatus",
    "ProfileAvailabilityChecker",
    "FallbackStrategy",
    "ROS2Interactions",
    "get_profile",
    "get_all_profiles",
    "list_profile_names",
    "get_game_profiles",
    "get_all_ros2_endpoints",
]
