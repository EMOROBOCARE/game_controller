"""Decision event builders and publish helpers.

This module provides functions to build properly formatted events
for the /decision/events topic consumed by decision_making node.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional


def build_game_init_event(
    slug: str,
    title: str,
    introduction: str,
    difficulty: str,
    phase_sequence: List[str],
    phase_configs: Dict[str, Dict[str, Any]],
    rounds: List[Dict[str, Any]],
    session_id: Optional[int] = None,
    game_type: Optional[str] = None,
    special_handler: Optional[str] = None,
) -> Dict[str, Any]:
    """Build a GAME_INIT event payload.
    
    Args:
        slug: Game identifier (e.g., "colores")
        title: Game display title
        introduction: Introduction text spoken at game start
        difficulty: Difficulty level ("basic", "intermediate", "advanced")
        phase_sequence: List of phase codes in order (e.g., ["P1", "P2", "P3"])
        phase_configs: Dict of phase code -> phase configuration
        rounds: List of round dictionaries with question data
        session_id: Optional session ID (auto-generated if not provided)
        game_type: Optional game type identifier
        special_handler: Optional special handler name
        
    Returns:
        Dict containing the full GAME_INIT event structure
    """
    payload = {
        "slug": slug,
        "title": title,
        "introduction": introduction,
        "difficulty": difficulty,
        "phaseSequence": phase_sequence,
        "phaseConfigs": phase_configs,
        "rounds": rounds,
    }
    
    if session_id is not None:
        payload["sessionId"] = session_id
    if game_type:
        payload["gameType"] = game_type
    if special_handler:
        payload["specialHandler"] = special_handler
        
    return {
        "type": "GAME_INIT",
        "payload": payload,
    }


def build_on_complete_event(transaction_id: int) -> Dict[str, Any]:
    """Build an ON_COMPLETE event.
    
    This event signals to decision_making that the current state's
    required actions are complete and it should advance to the next state.
    
    Args:
        transaction_id: The transaction ID from the last /decision/state message
        
    Returns:
        Dict containing the ON_COMPLETE event structure
    """
    return {
        "type": "ON_COMPLETE",
        "payload": {
            "transactionId": transaction_id,
        },
    }


def build_user_intent_event(
    transaction_id: int,
    value: str,
    correct: Optional[bool] = None,
    modality: str = "touch",
    metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build a USER_INTENT event for user input.
    
    Args:
        transaction_id: The transaction ID from the last /decision/state message
        value: The user's input value (e.g., selected option label)
        correct: Pre-computed correctness (if controller knows the answer)
        modality: Input modality ("touch", "speech", "gesture")
        metadata: Optional additional metadata
        
    Returns:
        Dict containing the USER_INTENT event structure
    """
    payload: Dict[str, Any] = {
        "transactionId": transaction_id,
        "value": value,
        "modality": modality,
    }
    
    if correct is not None:
        payload["correct"] = correct
    if metadata:
        payload["metadata"] = metadata
        
    return {
        "type": "USER_INTENT",
        "payload": payload,
    }


def build_game_control_event(command: str) -> Dict[str, Any]:
    """Build a GAME_CONTROL event for session control.
    
    Args:
        command: Control command ("PAUSE", "RESUME", "RESTART", "EXIT")
        
    Returns:
        Dict containing the GAME_CONTROL event structure
    """
    return {
        "type": "GAME_CONTROL",
        "payload": {
            "command": command.upper(),
        },
    }


def event_to_json(event: Dict[str, Any]) -> str:
    """Serialize event to JSON string for ROS message.
    
    Args:
        event: Event dictionary to serialize
        
    Returns:
        JSON string representation
    """
    return json.dumps(event, ensure_ascii=False)


def parse_decision_state(json_data: str) -> Optional[Dict[str, Any]]:
    """Parse a /decision/state message JSON.
    
    Args:
        json_data: JSON string from decision/state topic
        
    Returns:
        Parsed dict or None if parsing fails
    """
    try:
        return json.loads(json_data)
    except (json.JSONDecodeError, TypeError):
        return None
