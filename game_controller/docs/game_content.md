# Game Content Format

## Overview
`game_controller` content is metadata-driven. Game files define:
- game identity and menu metadata
- supported phases and per-phase prompts
- difficulty presets
- answer pool type

Rounds/questions sent to `decision_making` are generated at runtime by `build_game_init_payload()`.

## File Layout

```text
game_controller/game_controller/games/
  colores.json
  answers/
    colours.json
  phases/
    generalPhases.json
```

## Game File Schema (`games/<slug>.json`)

Example (`colores.json`):

```json
{
  "slug": "colores",
  "title": "Colores",
  "image": "assets/colores.png",
  "intro": "Vamos a jugar a los colores.",
  "requires": ["chatbot", "tts"],
  "supportedPhases": ["P1", "P2", "P3", "P4_YESNO", "P6", "P7"],
  "difficulties": {
    "basic": { "level": "basic", "optionsCount": 2 }
  },
  "phaseConfig": {
    "P3": { "phase_introduction": "Señala el color correcto.", "prompt": "Señala el color {colour}" }
  },
  "answerType": "colours"
}
```

Key fields:
- `slug`, `title`: game identity
- `image`, `intro`: menu/intro display
- `supportedPhases`: allowed phase sequence options
- `difficulties`: UI/selection metadata
- `phaseConfig`: per-phase overrides merged with shared defaults
- `answerType`: answer pool file name under `games/answers/`
- Optional: `gameType`, `specialHandler`, `requires`, feedback/effects metadata

## Answer Pool Schema (`games/answers/<answerType>.json`)

Example (`colours.json`):

```json
[
  { "label": "red", "value": "red", "image": "red_circle", "type": "colour" },
  { "label": "blue", "value": "blue", "image": "blue_circle", "type": "colour" }
]
```

Supported fields used by builder:
- `label` (display)
- `value` (semantic answer)
- `image` (asset id/path)
- Optional: `type`, `difficulty`

## Runtime Payload Generation
`build_game_init_payload()` in `game_controller/game_controller/game_controller/content/builder.py`:

1. Resolves phase sequence from request + `supportedPhases`.
2. Merges `phaseConfig` with shared defaults.
3. Loads answer pool via `answerType`.
4. Generates round/question options per phase and difficulty.
5. Produces `GAME_INIT.payload` with:
- `phaseSequence`
- `phaseConfigs`
- `rounds[]`

Generated round shape:

```json
{
  "id": 1,
  "phase": "P3",
  "difficulty": "basic",
  "question": {
    "questionId": 1,
    "prompt": "Señala el color red",
    "imageUrl": "red_circle",
    "answer": "red",
    "questionType": "multiple_choice",
    "options": [
      { "id": "red", "label": "red", "imageUrl": "red_circle", "correct": true },
      { "id": "blue", "label": "blue", "imageUrl": "blue_circle", "correct": false }
    ],
    "meta": { "answerType": "colours" }
  }
}
```

## Phase Notes
Controller defaults include:
- `P1`, `P2`, `P3`, `P4_YESNO`, `P6`, `P7`, `TRACING`

Behavior comes from generated question payloads + `phaseConfigs` passed to `decision_making`.

## Adding A New Game
1. Create `games/<slug>.json`.
2. Add answer pool `games/answers/<type>.json` and set `answerType`.
3. Set `supportedPhases` and optional `phaseConfig` prompt templates.
4. Verify menu metadata appears in initial manifest.
5. Start a session and inspect generated `GAME_INIT` on `/decision/events`.
