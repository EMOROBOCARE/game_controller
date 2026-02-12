"""Game content loaders.

Load game configuration from YAML or JSON files.
"""

from __future__ import annotations

import json
import os
from functools import lru_cache
from typing import Any, Dict, List, Optional

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


def get_games_directory() -> str:
    """Get the path to the games directory."""
    try:
        from ament_index_python.packages import get_package_share_directory

        pkg_share = get_package_share_directory("game_controller")
        return os.path.join(pkg_share, "games")
    except Exception:
        # Fallback for development
        current_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(current_dir, "..", "..", "games")


def get_answers_directory() -> str:
    """Get the path to the answers directory."""
    return os.path.join(get_games_directory(), "answers")


def get_phases_directory() -> str:
    """Get the path to the phases directory."""
    return os.path.join(get_games_directory(), "phases")


def _load_json_file(filepath: str) -> Optional[Any]:
    """Load a JSON file safely."""
    if not os.path.isfile(filepath):
        return None
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return None


def _load_yaml_file(filepath: str) -> Optional[Any]:
    """Load a YAML file safely."""
    if yaml is None or not os.path.isfile(filepath):
        return None
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except (yaml.YAMLError, OSError):
        return None


def _load_config_file(base_path: str) -> Optional[Any]:
    """Load a config file, trying .yaml first then .json."""
    yaml_path = base_path + ".yaml"
    data = _load_yaml_file(yaml_path)
    if data is not None:
        return data
    json_path = base_path + ".json"
    return _load_json_file(json_path)


def load_answer_set(answer_type: str) -> Optional[List[Dict[str, Any]]]:
    """Load an answer set by type from games/answers/.

    Args:
        answer_type: Answer set identifier (e.g., "colours")

    Returns:
        List of answer dicts or None if not found/invalid.
    """
    if not answer_type:
        return None
    answers_dir = get_answers_directory()
    filepath = os.path.join(answers_dir, f"{answer_type}.json")
    data = _load_json_file(filepath)
    if isinstance(data, list):
        return data
    return None


@lru_cache(maxsize=8)
def load_phase_definitions(name: str = "generalPhases") -> Dict[str, Any]:
    """Load shared phase definitions from games/phases/.

    Args:
        name: Phase definitions file stem (without extension)

    Returns:
        Dict of phaseCode -> phase definition dict.
    """
    phases_dir = get_phases_directory()
    base_path = os.path.join(phases_dir, name)
    data = _load_config_file(base_path)
    if isinstance(data, dict):
        return data
    return {}


def list_available_games() -> List[str]:
    """List available game slugs (from .yaml and .json, deduplicated)."""
    games_dir = get_games_directory()
    if not os.path.isdir(games_dir):
        return []

    slugs: set[str] = set()
    for filename in os.listdir(games_dir):
        if filename.endswith(".yaml"):
            slugs.add(filename[:-5])
        elif filename.endswith(".json"):
            slugs.add(filename[:-5])
    return sorted(slugs)


def load_game_content(slug: str) -> Optional[Dict[str, Any]]:
    """Load game content by slug (.yaml preferred over .json).

    Args:
        slug: Game identifier (e.g., "colores")

    Returns:
        Game data dict or None if not found
    """
    games_dir = get_games_directory()
    base_path = os.path.join(games_dir, slug)
    data = _load_config_file(base_path)
    if isinstance(data, dict):
        return data
    return None


def get_game_metadata(slug: str) -> Optional[Dict[str, Any]]:
    """Get game metadata without loading full content.

    Args:
        slug: Game identifier

    Returns:
        Dict with slug, title, image, supportedPhases, difficulties
    """
    content = load_game_content(slug)
    if not content:
        return None

    return {
        "slug": content.get("slug", slug),
        "title": content.get("title", slug.title()),
        "image": content.get("image"),
        "intro": content.get("intro"),
        # UI menu metadata
        "available": bool(content.get("available", True)),
        "levels": content.get("levels") or [{"name": "Nivel 1", "id": 1}],
        "supportedPhases": content.get("supportedPhases", []),
        "difficulties": content.get("difficulties", {}),
        "gameType": content.get("gameType"),
        "specialHandler": content.get("specialHandler"),
        "requires": content.get("requires", []),
    }


def get_all_games_metadata() -> List[Dict[str, Any]]:
    """Get metadata for all available games."""
    metadata = []
    for slug in list_available_games():
        meta = get_game_metadata(slug)
        if meta:
            metadata.append(meta)
    return metadata
