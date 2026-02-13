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
        "hints": ["highlight", "highlight", "say_answer"],
        "maxFailures": 2,
        "config": {},
    },
    "P2": {
        "hints": ["clear_say", "model_and_correct"],
        "maxFailures": 2,
        "config": {},
    },
    "P3": {
        "hints": ["highlight", "highlight_and_solve"],
        "maxFailures": 2,
        "config": {},
    },
    "P4": {
        "hints": ["head_gesture", "explain"],
        "maxFailures": 2,
        "subRounds": 2,
        "config": {},
    },
    "P5": {
        "hints": [],
        "maxFailures": 0,
        "successResponse": "Aquííí",
        "config": {},
    },
    "P6": {
        "hints": ["suggest", "offer_choices"],
        "maxFailures": 2,
        "config": {},
    },
    "TRACING": {
        "hints": ["illuminate", "guide"],
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

def _coerce_bool_like(value: Any) -> Optional[bool]:
    """Coerce common bool-like values used in game config."""
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if not text:
        return None
    if text in {"true", "1", "yes", "on", "si", "sí"}:
        return True
    if text in {"false", "0", "no", "off"}:
        return False
    return None


def _build_question_payload_flags(phase_cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Build per-question flags from phase config."""
    if not isinstance(phase_cfg, dict):
        return {}

    flags: Dict[str, Any] = {}
    say_prompt_raw = phase_cfg.get("say_prompt")
    if say_prompt_raw is None:
        say_prompt_raw = phase_cfg.get("sayPrompt")
    say_prompt = _coerce_bool_like(say_prompt_raw)
    if say_prompt is not None:
        flags["say_prompt"] = bool(say_prompt)

    expected_question = phase_cfg.get("expected_question")
    if expected_question is None:
        expected_question = phase_cfg.get("expectedQuestion")
    if expected_question is not None:
        text = str(expected_question).strip()
        if text:
            flags["expected_question"] = text

    return flags


def _fallback_prompt(phase: str, phase_cfg: Dict[str, Any], values: Dict[str, Any]) -> tuple[str, str]:
    """Return (prompt_text, prompt_verbal) when templates are missing.

    game_controller uses question.prompt for EmorobCare expressive TTS in QUESTION_PRESENT, so prompts should not be empty.
    """
    intro = (
        phase_cfg.get("verbal_instructions")
        or phase_cfg.get("verbalInstructions")
        or phase_cfg.get("text_instructions")
        or phase_cfg.get("textInstructions")
        or phase_cfg.get("phase_introduction")
        or phase_cfg.get("phaseIntroduction")
    )
    if isinstance(intro, str) and intro.strip():
        return intro.strip(), intro.strip()

    expr = str(values.get("expr") or "").strip()
    if phase in {"P1", "P3"} and expr:
        return f"Señala {expr}", f"Señala {expr}"

    if phase == "P4" and expr:
        return f"¿Es esto {expr}?", f"¿Es esto {expr}?"

    option1 = str(values.get("option1") or "").strip()
    option2 = str(values.get("option2") or "").strip()
    if phase == "P6" and option1 and option2:
        text = f"¿Es un {option1} o {option2}?"
        return text, text

    return "Elige una opción.", "Elige una opción."


def _phase_instruction(phase_cfg: Dict[str, Any], verbal: bool) -> str:
    if not isinstance(phase_cfg, dict):
        return ""
    if verbal:
        value = (
            phase_cfg.get("verbal_instructions")
            or phase_cfg.get("verbalInstructions")
            or phase_cfg.get("text_instructions")
            or phase_cfg.get("textInstructions")
        )
    else:
        value = (
            phase_cfg.get("text_instructions")
            or phase_cfg.get("textInstructions")
            or phase_cfg.get("verbal_instructions")
            or phase_cfg.get("verbalInstructions")
        )
    if value is None:
        return ""
    return str(value).strip()


def _compose_instruction_prompt(instruction: str, question_prompt: str) -> str:
    intro = str(instruction or "").strip()
    prompt = str(question_prompt or "").strip()
    if not intro:
        return prompt
    if not prompt:
        return intro
    intro_lower = intro.lower()
    prompt_lower = prompt.lower()
    if prompt_lower == intro_lower or prompt_lower.startswith(intro_lower):
        return prompt
    if intro.endswith((".", "?", "!", ":", ";")):
        return f"{intro} {prompt}".strip()
    return f"{intro}. {prompt}".strip()


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
        prompt_verbal_tmpl = phase_cfg.get("promptVerbal") or phase_cfg.get("prompt_verbal") or phase_cfg.get("prompt")
        prompt_text_tmpl = phase_cfg.get("promptText") or phase_cfg.get("prompt_text") or prompt_verbal_tmpl
        verbal_instruction = _phase_instruction(phase_cfg, verbal=True)
        text_instruction = _phase_instruction(phase_cfg, verbal=False)

        # Shuffle per phase to reduce repetition.
        phase_items = list(all_items)
        rng.shuffle(phase_items)

        if phase_upper == "P4":
            sub_rounds = int(phase_cfg.get("subRounds") or 2)
            sequence = phase_cfg.get("subRoundSequence") or ["incorrect", "correct"]

            # P4 difficulty-dependent answer ordering
            answer_order = phase_cfg.get("answerOrderByDifficulty", {})
            if isinstance(answer_order, dict) and difficulty in answer_order:
                order = answer_order[difficulty]
                if isinstance(order, list):
                    sequence = order

            for i in range(max(0, rounds_per_phase)):
                correct_item = phase_items[i % len(phase_items)]
                incorrect_candidates = [
                    it for it in all_items if it.get("value") != correct_item.get("value")
                ]
                incorrect_item = (
                    rng.choice(incorrect_candidates)
                    if incorrect_candidates
                    else correct_item
                )
                image_url = correct_item.get("image")

                for kind in list(sequence)[:sub_rounds]:
                    asked_item = (
                        correct_item if str(kind).lower() == "correct" else incorrect_item
                    )
                    statement_true = asked_item.get("value") == correct_item.get("value")
                    yes_no = "si" if statement_true else "no"
                    question_flags = _build_question_payload_flags(phase_cfg)

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
                    values.setdefault("animal", asked_label)
                    values.setdefault("fruta", asked_label)
                    values.setdefault("emocion", asked_label)
                    values.setdefault("forma", asked_label)
                    values.setdefault("objeto", asked_label)
                    values.setdefault("lugar", asked_label)
                    values.setdefault("colour", asked_label)

                    prompt_verbal = _safe_format(prompt_verbal_tmpl, values)
                    prompt_text = _safe_format(prompt_text_tmpl, values)
                    if not str(prompt_verbal).strip():
                        prompt_text, prompt_verbal = _fallback_prompt(
                            phase_upper,
                            phase_cfg,
                            values,
                        )
                    prompt_text = _compose_instruction_prompt(text_instruction, prompt_text)
                    prompt_verbal = _compose_instruction_prompt(verbal_instruction, prompt_verbal)
                    options = generate_yes_no_options(yes_no)
                    question_payload = {
                        "questionId": question_id,
                        "prompt": prompt_verbal,
                        "promptText": prompt_text,
                        "promptVerbal": prompt_verbal,
                        "imageUrl": image_url,
                        "answer": yes_no,
                        "questionType": "yes_no",
                        "options": options,
                        "meta": {
                            "asked": asked_item.get("value", asked_label),
                            "shown": correct_item.get("value", correct_item.get("label", "")),
                        },
                    }
                    question_payload.update(question_flags)

                    rounds.append(
                        {
                            "id": round_id,
                            "phase": phase_upper,
                            "difficulty": difficulty,
                            "question": question_payload,
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

            # P2 is voice-only: no options
            if phase_upper == "P2":
                options: List[Dict[str, Any]] = []
                question_type = "speech"

            elif phase_upper == "P6":
                options = _build_choice_options(
                    correct_item,
                    all_items,
                    difficulty=difficulty,
                    correct_position_by_difficulty=phase_cfg.get("correctPositionByDifficulty"),
                )
                values["option1"] = options[0].get("label", "")
                values["option2"] = options[1].get("label", "")
                question_type = "multiple_choice"

            elif phase_upper == "P5":
                # P5 is evaluated with a single expected answer in decision_making,
                # so always include the answer option in the rendered choices.
                count = get_options_count(difficulty)
                distractors = [
                    it
                    for it in all_items
                    if it.get("value", it.get("label", "")) != correct_item.get("value", correct_label)
                ]
                distractor_count = min(max(0, count - 1), len(distractors))
                sample = [correct_item] + rng.sample(distractors, distractor_count)
                rng.shuffle(sample)
                options = [
                    {
                        "id": it.get("value", it.get("label", "")),
                        "label": it.get("label", ""),
                        "imageUrl": it.get("image"),
                        "correct": it.get("value", it.get("label", "")) == correct_item.get("value", correct_label),
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

            prompt_verbal = _safe_format(prompt_verbal_tmpl, values)
            prompt_text = _safe_format(prompt_text_tmpl, values)
            if not str(prompt_verbal).strip():
                prompt_text, prompt_verbal = _fallback_prompt(phase_upper, phase_cfg, values)
            prompt_text = _compose_instruction_prompt(text_instruction, prompt_text)
            prompt_verbal = _compose_instruction_prompt(verbal_instruction, prompt_verbal)

            question_flags = _build_question_payload_flags(phase_cfg)
            question_payload = {
                "questionId": question_id,
                "prompt": prompt_verbal,
                "promptText": prompt_text,
                "promptVerbal": prompt_verbal,
                "imageUrl": image_url,
                "answer": correct_item.get("value", correct_label),
                "questionType": question_type,
                "options": options,
                "meta": {"answerType": answer_type} if answer_type else {},
            }
            question_payload.update(question_flags)

            rounds.append(
                {
                    "id": round_id,
                    "phase": phase_upper,
                    "difficulty": difficulty,
                    "question": question_payload,
                }
            )

            round_id += 1
            question_id += 1

    return rounds


_LEGACY_PHASE_RENAME_MAP: Dict[str, str] = {
    "P4_YESNO": "P4",
    "P6": "P5",
    "P7": "P6",
}
_MODERN_PHASE_RENAME_MAP: Dict[str, str] = {
    "P4_YESNO": "P4",
}


def _uses_legacy_phase_numbering(names: List[str]) -> bool:
    """Detect legacy numbering where old P7 exists."""
    normalized = {str(name).strip().upper() for name in names if str(name).strip()}
    return "P7" in normalized


def _normalize_phase_name(name: str, legacy_numbering: bool = False) -> str:
    """Map phase names to current codes, preserving modern P5/P6 numbering."""
    upper = name.strip().upper()
    mapping = _LEGACY_PHASE_RENAME_MAP if legacy_numbering else _MODERN_PHASE_RENAME_MAP
    return mapping.get(upper, upper)


def normalize_phase_list(raw: Any) -> List[str]:
    """Normalize phase list input, mapping legacy phase names."""
    if isinstance(raw, str):
        raw = [raw]
    if not isinstance(raw, list):
        return []
    cleaned = [str(item) for item in raw if str(item).strip()]
    legacy_numbering = _uses_legacy_phase_numbering(cleaned)
    normalized: List[str] = []
    for item in cleaned:
        phase = _normalize_phase_name(item, legacy_numbering=legacy_numbering)
        if phase not in normalized:
            normalized.append(phase)
    return normalized


def _normalize_phase_config_keys(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize phase config dict keys from legacy to current names."""
    normalized: Dict[str, Any] = {}
    legacy_numbering = _uses_legacy_phase_numbering(list(raw.keys()))
    for key, value in raw.items():
        new_key = _normalize_phase_name(key, legacy_numbering=legacy_numbering)
        normalized[new_key] = value
    return normalized


def build_phase_configs(
    phases: List[str],
    game_phase_configs: Optional[Dict[str, Any]] = None,
) -> Dict[str, Dict[str, Any]]:
    """Build phase configurations for the given phases."""
    configs: Dict[str, Dict[str, Any]] = {}
    phase_defaults = _get_phase_defaults()

    # Normalize game config keys (e.g. P4_YESNO → P4)
    normalized_game_configs = _normalize_phase_config_keys(game_phase_configs) if game_phase_configs else {}

    for phase in phases:
        phase_upper = phase.upper()

        # Start with defaults
        default_config = phase_defaults.get(phase_upper, {})

        # Merge with game-specific config
        if phase_upper in normalized_game_configs:
            game_config = normalized_game_configs[phase_upper]
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
