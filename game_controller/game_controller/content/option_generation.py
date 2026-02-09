"""Option generation for game questions.

Generates distractor options for selection-based questions.
Supports difficulty-based option counts.
"""

from __future__ import annotations

import random
from typing import Any, Dict, List, Optional

from ..models.difficulty import get_options_count


# Default colors for the Colors game (Spanish labels)
DEFAULT_COLORS = [
    {"label": "rojo", "image": "assets/color-rojo.svg"},
    {"label": "verde", "image": "assets/color-verde.svg"},
    {"label": "azul", "image": "assets/color-azul.svg"},
    {"label": "amarillo", "image": "assets/color-amarillo.svg"},
    {"label": "naranja", "image": "assets/color-naranja.svg"},
    {"label": "morado", "image": "assets/color-morado.svg"},
    {"label": "rosa", "image": "assets/color-rosa.svg"},
    {"label": "blanco", "image": "assets/color-blanco.svg"},
    {"label": "negro", "image": "assets/color-negro.svg"},
    {"label": "marrón", "image": "assets/color-marron.svg"},
]


def get_options_count_for_difficulty(difficulty: str) -> int:
    """Get number of options based on difficulty level.

    Args:
        difficulty: Difficulty level string

    Returns:
        Number of options (2 for basic, 3 for intermediate, 4 for advanced)
    """
    return get_options_count(difficulty)


def generate_color_options(
    correct_color: str,
    difficulty: str = "intermediate",
    available_colors: Optional[List[Dict[str, str]]] = None,
    shuffle: bool = True,
) -> List[Dict[str, Any]]:
    """Generate options for a color question.

    Args:
        correct_color: The correct answer (color label)
        difficulty: Difficulty level (basic/intermediate/advanced)
        available_colors: List of color dicts with 'label' and 'image'
        shuffle: Whether to shuffle the options

    Returns:
        List of option dicts with id, label, imageUrl, and correct
    """
    if available_colors is None:
        available_colors = DEFAULT_COLORS

    num_options = get_options_count_for_difficulty(difficulty)

    # Find the correct color
    correct_entry = None
    for color in available_colors:
        if color["label"].lower() == correct_color.lower():
            correct_entry = color
            break

    # If correct color not in list, create an entry
    if correct_entry is None:
        correct_entry = {
            "label": correct_color,
            "image": f"assets/color-{correct_color.lower()}.svg",
        }

    # Get distractors (colors that aren't the correct one)
    distractors = [
        c for c in available_colors
        if c["label"].lower() != correct_color.lower()
    ]

    # Select random distractors
    num_distractors = min(num_options - 1, len(distractors))
    selected_distractors = random.sample(distractors, num_distractors)

    # Build options list
    options = []

    # Add correct option
    options.append({
        "id": correct_entry["label"],
        "label": correct_entry["label"],
        "imageUrl": correct_entry["image"],
        "correct": True,
    })

    # Add distractors
    for d in selected_distractors:
        options.append({
            "id": d["label"],
            "label": d["label"],
            "imageUrl": d["image"],
            "correct": False,
        })

    # Shuffle if requested
    if shuffle:
        random.shuffle(options)

    return options


def generate_yes_no_options(
    correct_answer: str = "si",
) -> List[Dict[str, Any]]:
    """Generate yes/no options.

    Args:
        correct_answer: "si" or "no"

    Returns:
        List of yes/no options
    """
    yes_correct = correct_answer.lower() in ("si", "sí", "yes", "true", "1")

    return [
        {
            "id": "si",
            "label": "Sí",
            "imageUrl": "assets/yes-button.svg",
            "correct": yes_correct,
        },
        {
            "id": "no",
            "label": "No",
            "imageUrl": "assets/no-button.svg",
            "correct": not yes_correct,
        },
    ]


def generate_generic_options(
    correct_item: Dict[str, Any],
    all_items: List[Dict[str, Any]],
    difficulty: str = "intermediate",
    shuffle: bool = True,
) -> List[Dict[str, Any]]:
    """Generate options from a list of items with difficulty-based count.

    Args:
        correct_item: The correct item dict (must have 'id' or 'label')
        all_items: List of all available items
        difficulty: Difficulty level
        shuffle: Whether to shuffle the options

    Returns:
        List of option dicts
    """
    num_options = get_options_count_for_difficulty(difficulty)

    correct_id = correct_item.get("id", correct_item.get("label", ""))

    # Get distractors
    distractors = [
        item for item in all_items
        if item.get("id", item.get("label", "")) != correct_id
    ]

    # Select random distractors
    num_distractors = min(num_options - 1, len(distractors))
    selected_distractors = random.sample(distractors, num_distractors)

    # Build options
    options = [
        {
            "id": correct_item.get("id", correct_item.get("label", "")),
            "label": correct_item.get("label", ""),
            "imageUrl": correct_item.get("image", correct_item.get("imageUrl", "")),
            "correct": True,
        }
    ]

    for d in selected_distractors:
        options.append({
            "id": d.get("id", d.get("label", "")),
            "label": d.get("label", ""),
            "imageUrl": d.get("image", d.get("imageUrl", "")),
            "correct": False,
        })

    if shuffle:
        random.shuffle(options)

    return options


def enrich_question_options(
    question: Dict[str, Any],
    phase_config: Dict[str, Any],
    difficulty_config: Dict[str, Any],
    available_colors: Optional[List[Dict[str, str]]] = None,
) -> Dict[str, Any]:
    """Enrich a question with generated options if needed.

    Args:
        question: Question dict (may have empty options)
        phase_config: Phase configuration with interactionType
        difficulty_config: Difficulty config with optionsCount or level

    Returns:
        Question dict with options populated
    """
    # Make a copy to avoid mutating original
    q = dict(question)

    # If options already exist and are non-empty, return as-is
    existing_options = q.get("options", [])
    if existing_options and len(existing_options) > 0:
        return q

    interaction_type = phase_config.get("interactionType", "selection")
    question_type = q.get("questionType", "").lower()

    # Determine difficulty level from config
    difficulty = difficulty_config.get("level", "intermediate")
    if "optionsCount" in difficulty_config:
        count = difficulty_config["optionsCount"]
        if count == 2:
            difficulty = "basic"
        elif count >= 4:
            difficulty = "advanced"
        else:
            difficulty = "intermediate"

    # Get the correct answer from question or meta
    meta = q.get("meta", {}) or {}
    correct_answer = q.get("answer") or meta.get("color") or meta.get("correct_answer")

    if interaction_type in ("yes_no", "yesno"):
        q["options"] = generate_yes_no_options(correct_answer or "si")
    elif question_type == "color" or "color" in meta:
        if correct_answer:
            q["options"] = generate_color_options(
                correct_answer,
                difficulty=difficulty,
                available_colors=available_colors,
            )
    elif interaction_type == "selection" and correct_answer:
        if "color" in meta:
            q["options"] = generate_color_options(
                correct_answer,
                difficulty=difficulty,
                available_colors=available_colors,
            )

    return q
