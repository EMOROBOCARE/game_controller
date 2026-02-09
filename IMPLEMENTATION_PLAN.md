# Implementation Plan: Game Controller Restructuring

## Overview

This plan adapts the existing game system to follow the colores game structure with unified phases, difficulty-based options, and proper integration with generic_ui.

---

## 1. Architecture Changes

### 1.1 Phase System Modifications

**Current Phases (P1-P7):**
- P1: Matching/Association
- P2: Voice/Repetition
- P3: Discrimination (touch/voice)
- P4: Yes/No (correct = no)
- P5: Yes/No (correct = yes)
- P6: Voice request ("where is")
- P7: Two-option selection

**New Phase Structure:**

| Phase | Purpose | Interaction | Key Changes |
|-------|---------|-------------|-------------|
| P1 | Matching images | touch | No change |
| P2 | Pronunciation | voice | No change |
| P3 | Discrimination | touch/voice | Difficulty-based options (2/3/4) |
| P4_YESNO | Unified Yes/No | touch/voice | Sample 1 correct + 1 incorrect, ask incorrect first, then correct |
| P6 | Child asks "where" | voice | No correct answer, highlight selections, robot says "Aquííí" |
| P7 | "Es un...o..." | touch/voice | Answer position by difficulty: basic=2nd, intermediate=1st, advanced=random |

### 1.2 Difficulty System

```
basic:       2 options
intermediate: 3 options
advanced:    4 options
```

---

## 2. File Structure (All files ≤200 lines)

```
game_controller/
├── game_controller/
│   ├── __init__.py
│   ├── node.py                    # Main ROS2 node (~150 lines)
│   ├── auto_advance.py            # Auto-advance scheduler
│   ├── decision_events.py         # Event utilities
│   ├── input_translation.py       # Input translation
│   │
│   ├── models/                    # NEW: Pydantic models
│   │   ├── __init__.py
│   │   ├── game.py               # GameConfig, Round, Question
│   │   ├── phase.py              # PhaseConfig, PhaseType
│   │   ├── option.py             # Option, OptionSet
│   │   └── difficulty.py         # DifficultyConfig
│   │
│   ├── phases/                    # NEW: Phase handlers
│   │   ├── __init__.py
│   │   ├── base.py               # BasePhaseHandler
│   │   ├── matching.py           # P1 MatchingPhaseHandler
│   │   ├── repetition.py         # P2 RepetitionPhaseHandler
│   │   ├── discrimination.py     # P3 DiscriminationPhaseHandler
│   │   ├── yes_no.py             # P4_YESNO UnifiedYesNoHandler
│   │   ├── pointing.py           # P6 PointingPhaseHandler
│   │   ├── choice.py             # P7 ChoicePhaseHandler
│   │   └── registry.py           # Phase handler factory
│   │
│   ├── content/
│   │   ├── __init__.py
│   │   ├── loaders.py            # Game content loading
│   │   ├── builder.py            # GAME_INIT payload builder
│   │   ├── option_generation.py  # Option generation with difficulty
│   │   └── correctness.py        # Answer validation
│   │
│   └── ui/
│       ├── __init__.py
│       ├── manifest_builder.py   # UI manifest construction
│       └── manifest_client.py    # UpdateManifest service client
│
└── games/
    ├── colores.json              # Colors game config
    ├── phases/
    │   └── generalPhases.json    # Phase definitions
    └── answers/
        └── colours.json          # Color answer options
```

---

## 3. Pydantic Models

### 3.1 models/option.py

```python
from pydantic import BaseModel
from typing import Optional, Any

class Option(BaseModel):
    id: Any
    label: str
    image_url: Optional[str] = None
    correct: bool = False
    hidden: bool = False
    highlighted: bool = False

class OptionSet(BaseModel):
    options: list[Option]

    def correct_option(self) -> Optional[Option]:
        return next((o for o in self.options if o.correct), None)
```

### 3.2 models/difficulty.py

```python
from pydantic import BaseModel
from typing import Literal

DifficultyLevel = Literal["basic", "intermediate", "advanced"]

class DifficultyConfig(BaseModel):
    level: DifficultyLevel
    options_count: int  # 2, 3, or 4

    @classmethod
    def from_level(cls, level: DifficultyLevel) -> "DifficultyConfig":
        counts = {"basic": 2, "intermediate": 3, "advanced": 4}
        return cls(level=level, options_count=counts[level])
```

### 3.3 models/phase.py

```python
from pydantic import BaseModel
from typing import Optional, Literal, Any
from enum import Enum

class InteractionType(str, Enum):
    MATCHING = "matching"
    VOICE = "voice"
    TOUCH = "touch"
    TOUCH_VOICE = "touch_voice"
    YES_NO = "yes_no"
    POINTING = "pointing"
    CHOICE = "choice"

class PhaseConfig(BaseModel):
    interaction_type: InteractionType
    fail_l1_action: str
    fail_l2_action: str
    max_failures: int = 2
    phase_introduction: Optional[str] = None
    prompt_template: Optional[str] = None
    expected_answer: Optional[str] = None  # For yes/no phases
    hint_type: str = "highlight"
    config: dict[str, Any] = {}
```

---

## 4. Phase Handler Implementations

### 4.1 phases/yes_no.py (Unified P4+P5)

```python
"""Unified Yes/No phase handler.

Combines old P4 (no) and P5 (yes) into single phase that:
1. Samples one correct and one incorrect answer randomly
2. First asks about the incorrect one (expected: No)
3. Then asks about the correct one (expected: Yes)
"""

class UnifiedYesNoHandler(BasePhaseHandler):
    def __init__(self, config: PhaseConfig):
        super().__init__(config)
        self._sub_round: int = 0  # 0 = asking incorrect, 1 = asking correct
        self._sampled_correct: Optional[Option] = None
        self._sampled_incorrect: Optional[Option] = None

    def setup_round(self, options: list[Option]) -> None:
        """Sample one correct and one incorrect option."""
        correct_opts = [o for o in options if o.correct]
        incorrect_opts = [o for o in options if not o.correct]
        self._sampled_correct = random.choice(correct_opts)
        self._sampled_incorrect = random.choice(incorrect_opts)
        self._sub_round = 0

    def get_current_question_item(self) -> Option:
        """Get the item being asked about."""
        if self._sub_round == 0:
            return self._sampled_incorrect  # First ask incorrect
        return self._sampled_correct  # Then ask correct

    def expected_answer(self) -> str:
        """Return expected yes/no answer."""
        return "no" if self._sub_round == 0 else "si"

    def evaluate_input(self, input_data: dict, question: Question) -> tuple[bool, str | None]:
        answer = input_data.get("value", "").lower()
        expected = self.expected_answer()
        is_correct = answer in (expected, "sí") if expected == "si" else answer == expected
        return is_correct, None

    def advance_sub_round(self) -> bool:
        """Advance to next sub-round. Returns True if phase complete."""
        if self._sub_round == 0:
            self._sub_round = 1
            self.reset_failure_count()
            return False  # Continue to asking correct
        return True  # Phase complete
```

### 4.2 phases/pointing.py (P6)

```python
"""P6 Pointing phase handler.

Child asks "where is X?" among options. No correct answer -
robot highlights what child says and responds "Aquííí".
"""

class PointingPhaseHandler(BasePhaseHandler):
    def __init__(self, config: PhaseConfig):
        super().__init__(config)
        self._highlighted: set[str] = set()

    def evaluate_input(self, input_data: dict, question: Question) -> tuple[bool, str | None]:
        # Any valid option is accepted
        value = input_data.get("value")
        if value and any(o.id == value or o.label == value for o in question.options):
            self._highlighted.add(value)
            return True, "Aquííí"
        return False, None

    def get_ui_updates(self, options: list[Option]) -> list[Option]:
        """Return options with highlighting/hiding applied."""
        result = []
        for opt in options:
            new_opt = opt.model_copy()
            if opt.id in self._highlighted or opt.label in self._highlighted:
                new_opt.highlighted = True
            else:
                new_opt.hidden = True
            result.append(new_opt)
        return result

    def handle_failure_l1(self, question: Question) -> dict:
        # P6 doesn't really fail - skip hint
        return {"action": "skip", "hint": ""}
```

### 4.3 phases/choice.py (P7)

```python
"""P7 Choice phase handler.

Robot asks "Es un ... o ..." with two options.
Correct answer position depends on difficulty:
- basic: second option
- intermediate: first option
- advanced: random
"""

class ChoicePhaseHandler(BasePhaseHandler):
    def __init__(self, config: PhaseConfig, difficulty: DifficultyLevel):
        super().__init__(config)
        self._difficulty = difficulty
        self._correct_position: int = 0  # 0=first, 1=second

    def setup_round(self, options: list[Option]) -> tuple[Option, Option]:
        """Setup the two options with correct position by difficulty."""
        correct = next(o for o in options if o.correct)
        incorrect = random.choice([o for o in options if not o.correct])

        if self._difficulty == "basic":
            self._correct_position = 1  # Second
        elif self._difficulty == "intermediate":
            self._correct_position = 0  # First
        else:  # advanced
            self._correct_position = random.randint(0, 1)

        if self._correct_position == 0:
            return (correct, incorrect)
        return (incorrect, correct)

    def build_prompt(self, opt1: Option, opt2: Option) -> str:
        """Build 'Es un ... o ...' prompt."""
        return f"¿Es un {opt1.label} o {opt2.label}?"
```

---

## 5. generalPhases.json Updates

```json
{
    "P1": {
        "interaction": ["matching"],
        "purpose": "Associate images to words",
        "phase_introduction": "Une imágenes iguales para aprender nuevas palabras.",
        "prompt": null,
        "hint": "highlight"
    },
    "P2": {
        "interaction": ["voice"],
        "purpose": "Learning how to pronounce words",
        "phase_introduction": "Repite después de mí.",
        "prompt": "¿Qué es? Es {determinant} {word}.",
        "hint": "say_answer"
    },
    "P3": {
        "interaction": ["touch", "voice"],
        "purpose": "Discriminate between different images",
        "phase_introduction": "Señala la imagen correcta.",
        "prompt": "Game dependent",
        "hint": "highlight"
    },
    "P4_YESNO": {
        "interaction": ["touch", "voice"],
        "purpose": "Yes/No discrimination with incorrect first, then correct",
        "phase_introduction": "Responde sí o no.",
        "prompt": "¿Es esto {determinant} {word}?",
        "sub_rounds": 2,
        "sub_round_sequence": ["incorrect", "correct"],
        "hint": "highlight"
    },
    "P6": {
        "interaction": ["voice"],
        "purpose": "Child requests items, robot highlights and confirms",
        "phase_introduction": "¿Qué quieres señalar?",
        "prompt": "¿Dónde está?",
        "expected_answer": null,
        "success_response": "Aquííí",
        "hint": "skip"
    },
    "P7": {
        "interaction": ["touch", "voice"],
        "purpose": "Two-option choice with difficulty-based correct position",
        "phase_introduction": "¿Cuál es la respuesta correcta?",
        "prompt": "¿Es un {option1} o {option2}?",
        "correct_position_by_difficulty": {
            "basic": "second",
            "intermediate": "first",
            "advanced": "random"
        },
        "hint": "highlight"
    }
}
```

---

## 6. option_generation.py Updates

```python
def get_options_count_for_difficulty(difficulty: str) -> int:
    """Get number of options based on difficulty."""
    return {"basic": 2, "intermediate": 3, "advanced": 4}.get(difficulty, 3)

def generate_options(
    correct_item: dict,
    all_items: list[dict],
    difficulty: str,
    shuffle: bool = True,
) -> list[Option]:
    """Generate options with difficulty-based count."""
    count = get_options_count_for_difficulty(difficulty)

    # Get distractors
    distractors = [i for i in all_items if i["id"] != correct_item["id"]]
    selected = random.sample(distractors, min(count - 1, len(distractors)))

    # Build options
    options = [Option(id=correct_item["id"], label=correct_item["label"],
                      image_url=correct_item.get("image"), correct=True)]
    for d in selected:
        options.append(Option(id=d["id"], label=d["label"],
                             image_url=d.get("image"), correct=False))

    if shuffle:
        random.shuffle(options)
    return options
```

---

## 7. colores.json Structure

```json
{
  "slug": "colores",
  "title": "Colores",
  "image": "assets/colores.png",
  "intro": "Vamos a jugar a los colores.",
  "correct_effects": ["confetti", "sparkles", "emoji"],
  "positive_feedback": [
    "¡Genial! Es el color {expr}",
    "¡Muy bien! Es el color {expr}",
    "¡Perfecto! Es el color {expr}"
  ],
  "hint_timeout": 8,
  "supportedPhases": ["P1", "P2", "P3", "P4_YESNO", "P6", "P7"],
  "difficulties": {
    "basic": {"optionsCount": 2},
    "intermediate": {"optionsCount": 3},
    "advanced": {"optionsCount": 4}
  },
  "phaseConfig": {
    "P3": {
      "prompt": "Señala el color {colour}"
    },
    "P4_YESNO": {
      "prompt": "¿Es esto el color {colour}?"
    },
    "P6": {
      "prompt": "¿Dónde está el color?"
    },
    "P7": {
      "prompt": "¿Es un {option1} o {option2}?"
    }
  },
  "answerType": "colours"
}
```

---

## 8. Integration Points

### 8.1 ROS2 Topics (via communication_hub)

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/decision/state` | Subscribe | Game state from decision_making |
| `/decision/events` | Publish | GAME_INIT, USER_INTENT, ON_COMPLETE |
| `/intents` | Subscribe | User input (converted from /ui/input) |
| `/game/game_selector` | Subscribe | Game selection |
| `/game/user_selector` | Subscribe | User selection |

### 8.2 generic_ui Service

- Service: `/generic_ui/update_manifest`
- Operations: `set` (initial), `patch` (state updates)
- Manifest includes: component registry, layout, instances, ops

### 8.3 communication_hub Translation

```
/ui/input (String)
    → communication_hub
    → /intents (Intent msg with modality)
```

---

## 9. Implementation Order

1. **Models** (Pydantic) - Foundation for all other code
2. **Phase handlers** - Core game logic
3. **generalPhases.json** - Phase configuration
4. **option_generation.py** - Difficulty-based options
5. **builder.py** - GAME_INIT payload
6. **manifest_builder.py** - UI state patches
7. **node.py** - Wire everything together
8. **colores.json** - Game content

---

## 10. Testing Strategy

- Unit tests for each phase handler
- Unit tests for option generation
- Integration tests for full game flow
- E2E tests with mock ROS2 topics

---

## Questions for Clarification

1. Should the unified P4_YESNO phase count as 1 or 2 rounds in the round counter?
2. For P6, should there be a limit on how many items the child can highlight?
3. What happens if the child doesn't respond to P6 - timeout to next phase?
4. Should P7 allow speech input ("el primero"/"el segundo") or only touch?
