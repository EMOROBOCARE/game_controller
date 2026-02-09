"""GAME_INIT payload builder.

Builds the payload consumed by decision_making node.
Supports difficulty-based option counts and unified phase structure.
"""

from __future__ import annotations

import random
from typing import Any, Dict, List, Optional

from .loaders import load_answer_set, load_phase_definitions
from .option_generation import generate_generic_options, generate_yes_no_options
from ..models.difficulty import get_options_count


# Default phase configurations
PHASE_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "P1": {
        "interactionType": "matching",
        "failL1Action": "blink_correct",
        "failL2Action": "highlight_and_select",
        "maxFailures": 2,
        "config": {},
    },
    "P2": {
        "interactionType": "voice",
        "failL1Action": "clear_say",
        "failL2Action": "model_and_correct",
        "maxFailures": 2,
        "config": {},
    },
    "P3": {
        "interactionType": "touch_voice",
        "failL1Action": "point",
        "failL2Action": "highlight_and_solve",
        "maxFailures": 2,
        "config": {},
    },
    "P4_YESNO": {
        "interactionType": "yes_no",
        "failL1Action": "shake_head",
        "failL2Action": "explain",
        "maxFailures": 2,
        "subRounds": 2,
        "config": {},
    },
    "P6": {
        "interactionType": "pointing",
        "failL1Action": "skip",
        "failL2Action": "skip",
        "maxFailures": 0,
        "successResponse": "Aquííí",
        "config": {},
    },
    "P7": {
        "interactionType": "selection",
        "failL1Action": "suggest",
        "failL2Action": "offer_choices",
        "maxFailures": 2,
        "config": {},
    },
    "TRACING": {
        "interactionType": "drawing",
        "failL1Action": "illuminate",
        "failL2Action": "guide",
        "maxFailures": 2,
        "config": {},
    },
}


def _get_phase_defaults() -> Dict[str, Dict[str, Any]]:
    """Get default phase configurations from games/phases with code fallbacks."""
    merged: Dict[str, Dict[str, Any]] = {k: dict(v) for k, v in PHASE_DEFAULTS.items()}
    for phase, cfg in load_phase_definitions().items():
        if not isinstance(cfg, dict):
            continue
        upper = str(phase).strip().upper()
        merged[upper] = {**merged.get(upper, {}), **cfg}
    for cfg in merged.values():
        cfg.setdefault("config", {})
    return merged


def _extract_determinant_and_word(label: str) -> tuple[str, str]:
    """Extract Spanish determinant/article and base word for templates.

    Examples:
        "el agua" -> ("el", "agua")
        "la manzana" -> ("la", "manzana")
        "círculo" -> ("", "círculo")
    """
    text = (label or "").strip()
    if not text:
        return "", ""
    parts = text.split()
    if len(parts) >= 2 and parts[0].lower() in {"el", "la", "los", "las", "un", "una"}:
        return parts[0], " ".join(parts[1:])
    return "", text


class _SafeFormatDict(dict):
    def __missing__(self, key: str) -> str:  # type: ignore[override]
        return "{" + key + "}"


def _safe_format(template: Optional[str], values: Dict[str, Any]) -> str:
    """Format prompt templates without failing on missing keys."""
    if not template:
        return ""
    try:
        return str(template).format_map(_SafeFormatDict(values))
    except Exception:
        return str(template)


def _fallback_prompt(phase: str, phase_cfg: Dict[str, Any], values: Dict[str, Any]) -> str:
    """Return a non-empty prompt when templates are missing.

    game_controller uses question.prompt for EmorobCare expressive TTS in QUESTION_PRESENT, so prompts should not be empty.
    """
    intro = phase_cfg.get("phase_introduction") or phase_cfg.get("phaseIntroduction")
    if isinstance(intro, str) and intro.strip():
        return intro.strip()

    expr = str(values.get("expr") or "").strip()
    if phase in {"P1", "P3"} and expr:
        return f"Señala {expr}"

    if phase == "P4_YESNO" and expr:
        return f"¿Es esto {expr}?"

    option1 = str(values.get("option1") or "").strip()
    option2 = str(values.get("option2") or "").strip()
    if phase == "P7" and option1 and option2:
        return f"¿Es un {option1} o {option2}?"

    return "Elige una opción."


def _normalize_answer_items(raw: Any) -> List[Dict[str, Any]]:
    """Normalize answer items into a consistent shape."""
    if not isinstance(raw, list):
        return []

    normalized: List[Dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        label = item.get("label") or item.get("value") or ""
        value = item.get("value") or label
        image = (
            item.get("image")
            or item.get("imageUrl")
            or item.get("image_url")
            or item.get("img")
        )
        normalized.append(
            {
                **item,
                "label": label,
                "value": value,
                "image": image,
            }
        )

    return normalized


def _load_game_answers(game_content: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Load answer items for a game from its answerType reference."""
    answer_type = game_content.get("answerType")
    loaded = load_answer_set(str(answer_type)) if answer_type else None
    return _normalize_answer_items(loaded)


def _as_item_for_options(item: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "id": item.get("value", item.get("label", "")),
        "label": item.get("label", ""),
        "image": item.get("image"),
    }


def _build_choice_options(
    correct_item: Dict[str, Any],
    all_items: List[Dict[str, Any]],
    difficulty: str,
    correct_position_by_difficulty: Optional[Dict[str, str]] = None,
) -> List[Dict[str, Any]]:
    """Build exactly 2 options with correct position driven by difficulty."""
    correct_position_by_difficulty = correct_position_by_difficulty or {}
    position = str(correct_position_by_difficulty.get(difficulty, "random")).lower()

    correct_opt = {
        "id": correct_item.get("value", correct_item.get("label", "")),
        "label": correct_item.get("label", ""),
        "imageUrl": correct_item.get("image"),
        "correct": True,
    }

    distractors = [i for i in all_items if i.get("value") != correct_item.get("value")]
    if distractors:
        d = random.choice(distractors)
        wrong_opt = {
            "id": d.get("value", d.get("label", "")),
            "label": d.get("label", ""),
            "imageUrl": d.get("image"),
            "correct": False,
        }
    else:
        wrong_opt = {
            "id": "__wrong__",
            "label": "No",
            "imageUrl": None,
            "correct": False,
        }

    if position in {"first", "1"}:
        return [correct_opt, wrong_opt]
    if position in {"second", "2"}:
        return [wrong_opt, correct_opt]

    options = [correct_opt, wrong_opt]
    random.shuffle(options)
    return options


def _generate_rounds_from_answers(
    phase_sequence: List[str],
    difficulty: str,
    rounds_per_phase: int,
    answer_items: List[Dict[str, Any]],
    phase_configs: Dict[str, Dict[str, Any]],
    answer_type: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """Generate rounds from answer pool + phase templates."""
    if not answer_items:
        return []

    rng = random.Random()
    rounds: List[Dict[str, Any]] = []
    round_id = 1
    question_id = 1

    all_items = list(answer_items)

    for phase in phase_sequence:
        phase_upper = str(phase).upper()
        phase_cfg = phase_configs.get(phase_upper, {})
        prompt_template = phase_cfg.get("prompt")
        interaction_type = str(phase_cfg.get("interactionType", "") or "").lower()

        # Shuffle per phase to reduce repetition.
        phase_items = list(all_items)
        rng.shuffle(phase_items)

        if phase_upper == "P4_YESNO":
            sub_rounds = int(phase_cfg.get("subRounds") or 2)
            sequence = phase_cfg.get("subRoundSequence") or ["incorrect", "correct"]

            for i in range(max(0, rounds_per_phase)):
                correct_item = phase_items[i % len(phase_items)]
                incorrect_candidates = [it for it in all_items if it.get("value") != correct_item.get("value")]
                incorrect_item = rng.choice(incorrect_candidates) if incorrect_candidates else correct_item

                image_url = correct_item.get("image")

                for kind in list(sequence)[:sub_rounds]:
                    asked_item = correct_item if str(kind).lower() == "correct" else incorrect_item
                    statement_true = asked_item.get("value") == correct_item.get("value")
                    yes_no = "si" if statement_true else "no"

                    asked_label = asked_item.get("label", "")
                    determinant, word = _extract_determinant_and_word(asked_label)
                    item_type = asked_item.get("type")

                    values: Dict[str, Any] = {
                        "expr": asked_label,
                        "value": asked_item.get("value", asked_label),
                        "determinant": determinant,
                        "word": word,
                    }
                    if item_type:
                        values[str(item_type)] = asked_label
                    # Common Spanish placeholders used across game configs.
                    values.setdefault("animal", asked_label)
                    values.setdefault("fruta", asked_label)
                    values.setdefault("emocion", asked_label)
                    values.setdefault("forma", asked_label)
                    values.setdefault("objeto", asked_label)
                    values.setdefault("lugar", asked_label)
                    # Colours game uses English placeholder.
                    values.setdefault("colour", asked_label)

                    prompt = _safe_format(prompt_template, values)
                    if not str(prompt).strip():
                        prompt = _fallback_prompt(phase_upper, phase_cfg, values)
                    options = generate_yes_no_options(yes_no)

                    rounds.append(
                        {
                            "id": round_id,
                            "phase": phase_upper,
                            "difficulty": difficulty,
                            "question": {
                                "questionId": question_id,
                                "prompt": prompt,
                                "imageUrl": image_url,
                                "answer": yes_no,
                                "questionType": "yes_no",
                                "options": options,
                                "meta": {
                                    "asked": asked_item.get("value", asked_label),
                                    "shown": correct_item.get("value", correct_item.get("label", "")),
                                },
                            },
                        }
                    )
                    round_id += 1
                    question_id += 1

            continue

        for i in range(max(0, rounds_per_phase)):
            correct_item = phase_items[i % len(phase_items)]
            correct_label = correct_item.get("label", "")
            determinant, word = _extract_determinant_and_word(correct_label)
            item_type = correct_item.get("type")

            values = {
                "expr": correct_label,
                "value": correct_item.get("value", correct_label),
                "determinant": determinant,
                "word": word,
            }
            if item_type:
                values[str(item_type)] = correct_label
            values.setdefault("animal", correct_label)
            values.setdefault("fruta", correct_label)
            values.setdefault("emocion", correct_label)
            values.setdefault("forma", correct_label)
            values.setdefault("objeto", correct_label)
            values.setdefault("lugar", correct_label)
            values.setdefault("colour", correct_label)

            image_url = correct_item.get("image")

            # Voice-only phases typically don't render options.
            if interaction_type == "voice" and phase_upper == "P2":
                options: List[Dict[str, Any]] = []
                question_type = "speech"

            elif phase_upper == "P7":
                options = _build_choice_options(
                    correct_item,
                    all_items,
                    difficulty=difficulty,
                    correct_position_by_difficulty=phase_cfg.get("correctPositionByDifficulty"),
                )
                values["option1"] = options[0].get("label", "")
                values["option2"] = options[1].get("label", "")
                question_type = "multiple_choice"

            elif phase_upper == "P6":
                # P6: no single correct answer - any selection can be accepted.
                # Keep a reduced set to match difficulty option count.
                count = get_options_count(difficulty)
                sample = rng.sample(all_items, min(count, len(all_items)))
                options = [
                    {
                        "id": it.get("value", it.get("label", "")),
                        "label": it.get("label", ""),
                        "imageUrl": it.get("image"),
                        "correct": True,
                    }
                    for it in sample
                ]
                question_type = "pointing"

            else:
                options = generate_generic_options(
                    _as_item_for_options(correct_item),
                    [_as_item_for_options(it) for it in all_items],
                    difficulty=difficulty,
                )
                question_type = "multiple_choice"

            prompt = _safe_format(prompt_template, values)
            if not str(prompt).strip():
                prompt = _fallback_prompt(phase_upper, phase_cfg, values)

            rounds.append(
                {
                    "id": round_id,
                    "phase": phase_upper,
                    "difficulty": difficulty,
                    "question": {
                        "questionId": question_id,
                        "prompt": prompt,
                        "imageUrl": image_url,
                        "answer": correct_item.get("value", correct_label),
                        "questionType": question_type,
                        "options": options,
                        "meta": {"answerType": answer_type} if answer_type else {},
                    },
                }
            )

            round_id += 1
            question_id += 1

    return rounds


def normalize_phase_list(raw: Any) -> List[str]:
    """Normalize phase list input."""
    if isinstance(raw, str):
        raw = [raw]
    if not isinstance(raw, list):
        return []
    return [str(item).strip().upper() for item in raw if str(item).strip()]


def build_phase_configs(
    phases: List[str],
    game_phase_configs: Optional[Dict[str, Any]] = None,
) -> Dict[str, Dict[str, Any]]:
    """Build phase configurations for the given phases."""
    configs: Dict[str, Dict[str, Any]] = {}
    phase_defaults = _get_phase_defaults()

    for phase in phases:
        phase_upper = phase.upper()

        # Start with defaults
        default_config = phase_defaults.get(phase_upper, {})

        # Merge with game-specific config
        if game_phase_configs and phase_upper in game_phase_configs:
            game_config = game_phase_configs[phase_upper]
            configs[phase_upper] = {**default_config, **game_config}
        elif game_phase_configs and phase in game_phase_configs:
            game_config = game_phase_configs[phase]
            configs[phase_upper] = {**default_config, **game_config}
        else:
            configs[phase_upper] = dict(default_config)

    return configs

def build_game_init_payload(
    game_content: Dict[str, Any],
    phases: Optional[List[str]] = None,
    difficulty: str = "basic",
    rounds_per_phase: int = 2,
    session_id: Optional[int] = None,
) -> Dict[str, Any]:
    """Build a complete GAME_INIT payload from game content."""
    slug = game_content.get("slug", "game")
    title = game_content.get("title", slug.title())
    intro = game_content.get("intro", f"Vamos a jugar a {title}")

    # Determine phases
    supported_phases = normalize_phase_list(game_content.get("supportedPhases", []))
    requested_phases = normalize_phase_list(phases) if phases else []
    if requested_phases:
        if supported_phases:
            filtered = [p for p in requested_phases if p in supported_phases]
            phase_sequence = filtered or supported_phases
        else:
            phase_sequence = requested_phases
    elif supported_phases:
        phase_sequence = supported_phases
    else:
        phase_sequence = ["P1", "P2", "P3"]

    # Build phase configs (defaults + per-game overrides)
    overrides = game_content.get("phaseConfig", {})
    if not isinstance(overrides, dict):
        overrides = {}
    phase_configs = build_phase_configs(phase_sequence, overrides)

    answers = _load_game_answers(game_content)
    rounds = _generate_rounds_from_answers(
        phase_sequence,
        difficulty=difficulty,
        rounds_per_phase=rounds_per_phase,
        answer_items=answers,
        phase_configs=phase_configs,
        answer_type=game_content.get("answerType"),
    )

    # Build payload
    payload: Dict[str, Any] = {
        "slug": slug,
        "title": title,
        "introduction": intro,
        "difficulty": difficulty,
        "phaseSequence": phase_sequence,
        "phaseConfigs": phase_configs,
        "rounds": rounds,
    }

    if session_id is not None:
        payload["sessionId"] = session_id

    # Copy any special handlers
    if game_content.get("gameType"):
        payload["gameType"] = game_content["gameType"]
    if game_content.get("specialHandler"):
        payload["specialHandler"] = game_content["specialHandler"]

    return payload
