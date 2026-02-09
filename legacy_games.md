# Legacy games (legacy/game_tasks + legacy/emorobcare_games)

This document describes the **legacy game behaviors**, **what the robot asks**, and **where the JSON data lives**.

In this monorepo, the legacy packages live under:
- `legacy/emorobcare_games/`
- `legacy/game_tasks/`

## Launcher & UI (emorobcare_games)

### Mission controller (game orchestrator)
- **File**: `legacy/emorobcare_games/emo_games/mission_controller.py`
- **Behavior**: lifecycle node that listens to `/intents`, publishes `/ui/show` and `/emo_games/ongoing_activities`, and starts individual games via action clients on `/game1/control` … `/game7/control`. It also exposes `/emo_games/list_available_activities` and calls `/idle_behavior`.
- **What is asked**: the mission controller itself does not ask questions; it routes to specific game tasks and pages.
- **JSONs**: none in this package (game content lives under `legacy/game_tasks/*/data/*.json`).

### Menu/config pages
- **Files**: `legacy/emorobcare_games/pages/menu.html`, `menu.js`, `config.html`
- **Behavior**: web menu selects a game and level, then navigates to the correct game page; listens to `/ui/show` to switch pages.
- **What is asked**: menu UI is for selection, not gameplay.
- **JSONs**: none.

## Legacy game catalog (game_tasks)

### Game 1 — “encuentra_objetos” (Atención)
- **Package**: `legacy/game_tasks/game1`
- **Behavior**:
  - Uses `AtencionGame` from `game1/game_templates.py` (JSON-based).
  - Two-step interaction per object:
    1) **Yes/No recognition**: asks if the target object is visible (e.g., “¿Ves la pera?”).
    2) **Follow-up naming**: asks to name the object (e.g., “¿Es un…?”).
  - Hints change by step: visual hints for the yes/no step; syllable/letter/full-word hints for naming.
- **What is asked**:
  - Example questions in JSON: “Ves la pera?”, “Ves el tomate?”, “Ves el plátano?”
  - Follow-up: “Es un?” with answer like “pera”, “tomate”, “plátano”.
- **JSONs**:
  - Primary data: `legacy/game_tasks/game1/data/atencion.json`
  - Additional (not referenced by template): `legacy/game_tasks/game1/data/encuentra.json` (intro + objects list)

### Game 2A — “donde_esta”
- **Package**: `legacy/game_tasks/game2`
- **Behavior**:
  - Uses `DondeEstaGame` from `game2/game_templates.py`.
  - UI shows a **big image** with multiple **small option images**; child answers by touch and/or speech (input mode is configurable in `config/00-defaults.yml`).
  - LLM evaluation is used (see README/config) and **open follow-up questions** can be triggered after consecutive correct answers using a per-question `llm_followup_prompt`.
  - Hints progressively reveal the answer (first syllable → full phrase).
- **What is asked**:
  - Example questions: “¿Dónde está el barco?”, “¿Dónde está el coche?”, “¿Dónde está el avión?”, “¿Dónde está el niño?”, “¿Dónde está el gato?”
  - Answers are locations like “agua”, “carretera”, “cielo”, “parque”, “debajo”.
- **JSONs**:
  - `legacy/game_tasks/game2/data/donde_esta.json`

### Game 2B — “quiero”
- **Package**: `legacy/game_tasks/game2`
- **Behavior**:
  - Uses `QuieroGame` from `game2/game_templates.py`.
  - Same UI structure as “donde_esta”, but the prompt is **“¿Qué quiero? Quiero…”**.
  - Hints use the “Quiero …” framing and progressive syllable/full-word cues.
- **What is asked**:
  - Example questions: “¿Qué quiero? Quiero…” with answers like “pan”, “agua”, “pintar”, “pelota”, “columpiarme”, “abrazar”, “descansar”.
- **JSONs**:
  - `legacy/game_tasks/game2/data/quiero.json`

### Game 3 — “emociones”
- **Package**: `legacy/game_tasks/game3`
- **Behavior**:
  - Uses `EmotionGame` from `game3/game_templates.py`.
  - Robot shows an **expression** and asks the child to identify it.
  - Hints progress from **first consonant → first syllable → full word**.
  - Level 3 asks the child to **choose an emotion** (answers are `null` in JSON).
- **What is asked**:
  - Level 1–2: “¡MIRA! Estoy contento/sorprendido/triste/enojado”.
  - Level 3: “¡TE TOCA! ¿Qué emoción quieres?”
- **JSONs**:
  - `legacy/game_tasks/game3/data/emotion.json`

### Game 4 — “formas” (shape tracing)
- **Package**: `legacy/game_tasks/game4`
- **Behavior**:
  - Uses `ShapeTracingGame` from `game4/game_templates.py`.
  - **Level 1**: child traces the displayed shape (e.g., circle, square).
  - **Level 2**: child identifies the shape (“¿Qué forma es?”).
  - **Level 3**: child chooses a shape; JSON includes a `joke` flag to allow the robot to sometimes draw a different shape.
  - JS page publishes `/ui/tracing` events and `/ui/input` for interactions.
- **What is asked**:
  - Level 1 prompts: “Traza el círculo/cuadrado/triángulo/estrella/rombo”.
  - Level 2 prompts: “¿Qué forma es?”
  - Level 3 prompts: “Te toca a ti. ¿Qué forma quieres?”
- **JSONs**:
  - `legacy/game_tasks/game4/data/shape_tracing.json`

### Game 5 — “animales”
- **Package**: `legacy/game_tasks/game5`
- **Behavior**:
  - Uses `AnimalesGame` from `game5/game_templates.py`.
  - **Yes/No** responses about animals shown in the image.
  - Hints: repeat question → add correct yes/no answer → emphasize.
- **What is asked**:
  - Level 1: “¿Es un elefante?”, “¿Es una gallina?”, “¿Es un perro?”
  - Level 2: “¿Ves un elefante?”, “¿Ves una gallina?”, “¿Ves una vaca?”
  - Answers are “sí” or “no”.
- **JSONs**:
  - `legacy/game_tasks/game5/data/animales.json`

### Game 7 — “colores”
- **Package**: `legacy/game_tasks/game7`
- **Behavior**:
  - Uses `ColorGame` from `game7/game_templates.py`.
  - Level 1: child identifies a displayed color.
  - Level 2: child chooses a color (answers are `null` in JSON).
  - Hints: first consonant → first syllable → full color name.
- **What is asked**:
  - Level 1: “Te toca. Es el color …” with answers like “rojo”, “verde”, “amarillo”, “azul”.
  - Level 2: “¡TE TOCA! ¿Qué color quieres?”
- **JSONs**:
  - `legacy/game_tasks/game7/data/colors.json`

## Notes on JSON locations

All **game content** (levels, questions, answers, images, prompts) lives under:
- `legacy/game_tasks/<game>/data/*.json`

The **menu / launcher** in `legacy/emorobcare_games` does not contain gameplay JSON; it only routes to the game pages and passes game names/levels.

## Missing/legacy references

- `legacy/emorobcare_games/pages/menu.js` references a **game6 / “charla”** entry (button `game8Button`), and the mission controller has an action client for `/game6/control`, but there is **no `legacy/game_tasks/game6` package or JSON** in this repo.
