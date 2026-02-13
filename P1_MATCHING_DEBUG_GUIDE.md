# P1 MatchingPhase — Full Debug Guide

## Quick Start: How to Run

```bash
cd /home/alono/EmorobCare/games_src/game_controller
docker compose up --build
```

**Access points:**
| Service | URL |
|---------|-----|
| UI (React shell) | http://localhost:8083 |
| Backend (WebSocket gateway) | ws://localhost:8092 |
| CDN (Module Federation) | http://localhost:8084 |
| LED mock UI | http://localhost:8095 |

**Services started (9 total):**
`decision_making`, `game_controller`, `communication_hub`, `llm_service`, `led_service_ros`, `led_service_mock`, `backend`, `web`, `emorobcare_components_cdn`

---

## Step-by-Step Manual Test

1. Open http://localhost:8083
2. Select a user (any — defaults to user 1)
3. Select game **"colores"** (with phases including P1)
4. Wait for the matching question to appear (e.g., "Señala rojo" with color swatches)
5. Drag and match the correct color item
6. **Expected:** Confetti effect, feedback text, then next question
7. **If frozen:** Check logs using commands below

---

## Log Commands

```bash
# All services (verbose — use for first-time debugging)
cd /home/alono/EmorobCare/games_src/game_controller
docker compose logs -f

# Just the input pipeline (recommended for P1 debugging)
docker compose logs -f communication_hub game_controller

# Only game_controller
docker compose logs -f game_controller

# Only communication_hub (ui_intent_bridge)
docker compose logs -f communication_hub

# Only decision_making FSM
docker compose logs -f decision_making

# Only the UI backend (WebSocket gateway)
docker compose logs -f backend

# Filter for debug tags (grep for our [BRIDGE]/[GC]/[TRANSLATE] tags)
docker compose logs -f communication_hub game_controller 2>&1 | grep -E '\[BRIDGE\]|\[GC\]|\[TRANSLATE\]'

# Filter for state transitions only
docker compose logs -f game_controller 2>&1 | grep '\[GC\] State update'

# Filter for input flow only
docker compose logs -f communication_hub game_controller 2>&1 | grep -E '\[BRIDGE\].*Published|\[GC\].*Input from|\[GC\].*Translated|\[GC\].*Published /decision'
```

---

## Complete Data Flow: From `docker compose up` to Button Press

### Phase A: Startup & Initialization

```
┌─────────────────────────────────────────────────────────────────────┐
│ STEP A1: Docker containers start in dependency order               │
│                                                                     │
│ File: docker-compose.yml                                            │
│                                                                     │
│ 1. decision_making starts (ROS 2 FSM node)                         │
│    - Publishes initial /decision/state: {state: "IDLE", tx: 1}     │
│    - QoS: TRANSIENT_LOCAL (latecomer subscribers get last state)    │
│                                                                     │
│ 2. backend starts (generic_ui WebSocket gateway)                    │
│    - Exposes /generic_ui/update_manifest ROS service                │
│    - Serves WebSocket on ws://localhost:8092/ws                     │
│                                                                     │
│ 3. game_controller starts                                           │
│    - Creates publishers, subscribers, manifest client               │
│    - After 2s timer: sends initial manifest to backend              │
│                                                                     │
│ 4. communication_hub starts                                         │
│    - ui_intent_bridge subscribes to /ui/input                       │
│    - Publishes to /intents                                          │
│                                                                     │
│ 5. web starts (React shell on :8083)                                │
│    - Connects to backend WebSocket                                  │
│    - Receives manifest → renders menu screen                        │
└─────────────────────────────────────────────────────────────────────┘
```

**Log markers to look for:**
```
[game_controller] Game Controller node initialized
[game_controller] Initial manifest sent. Hash: <hash>
[communication_hub] UI→Intent bridge ready (/ui/input -> /intents)
[decision_making] Decision making node ready
[backend] Manifest updated via set (hash: ... -> ...)
```

### Phase B: Game Selection & GAME_INIT

```
┌─────────────────────────────────────────────────────────────────────┐
│ STEP B1: User selects a game in the UI                             │
│                                                                     │
│ UI (React) → WebSocket → backend gateway                            │
│ → publishes to /game/game_selector: '{"slug":"colores"}'           │
│                                                                     │
│ File: node.py:_on_game_selected() (line ~893)                      │
│ - Parses game slug from JSON                                       │
│ - Sets _selected_game = "colores"                                  │
│ - Sets _start_requested = True                                      │
│ - Calls _try_start_game()                                           │
│                                                                     │
│ Log: "[game_controller] Game selected: colores"                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP B2: User selects a user (or default user 1)                   │
│                                                                     │
│ UI → /game/user_selector: '{"userId": 1}'                          │
│                                                                     │
│ File: node.py:_on_user_selected() (line ~937)                      │
│ - Parses user ID                                                    │
│ - If _start_requested=True → calls _try_start_game()               │
│                                                                     │
│ Log: "[game_controller] User selected: 1"                           │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP B3: Game content loaded & GAME_INIT published                 │
│                                                                     │
│ File: node.py:_start_game() (line ~993)                            │
│ 1. load_game_content("colores")                                     │
│    File: content/loaders.py:load_game_content()                     │
│    - Reads games/colores.json (or .yaml)                            │
│    - Returns: {slug, title, supportedPhases, phaseConfig, ...}     │
│                                                                     │
│ 2. build_game_init_payload(game_content, phases, difficulty, ...)   │
│    File: content/builder.py:build_game_init_payload()               │
│    - Generates rounds with questions                                │
│    - Each question has: prompt, answer, options[], meta             │
│    - P1 options include {id, label, imageUrl, correct: true/false}  │
│    - Returns full payload with rounds array                         │
│                                                                     │
│ 3. Publishes to /decision/events:                                   │
│    {                                                                │
│      "type": "GAME_INIT",                                           │
│      "payload": {                                                   │
│        "slug": "colores",                                           │
│        "phaseSequence": ["P1", "P2", "P3"],                        │
│        "difficulty": "basic",                                       │
│        "rounds": [                                                  │
│          {                                                          │
│            "id": 1, "phase": "P1",                                  │
│            "question": {                                            │
│              "questionId": 1,                                       │
│              "prompt": "Señala rojo",                               │
│              "answer": "rojo",                                      │
│              "options": [                                           │
│                {"id":"rojo","label":"Rojo","correct":true},         │
│                {"id":"azul","label":"Azul","correct":false}         │
│              ]                                                      │
│            }                                                        │
│          }, ...                                                     │
│        ]                                                            │
│      }                                                              │
│    }                                                                │
│                                                                     │
│ Log: "[GC] Published /decision/events: type=GAME_INIT, tx=?, ..."  │
│ Log: "Starting game: colores (phases=['P1','P2','P3'], ...)"       │
└─────────────────────────────────────────────────────────────────────┘
```

### Phase C: FSM State Machine Flow (decision_making drives the game)

After `GAME_INIT`, decision_making drives state transitions. Each state publishes to `/decision/state`, which game_controller receives in `_on_decision_state()` (node.py line ~399).

```
┌─────────────────────────────────────────────────────────────────────┐
│ STATE C1: PHASE_INTRO                                               │
│                                                                     │
│ /decision/state:                                                    │
│ {                                                                   │
│   "state": "GAME",                                                  │
│   "gameState": "PHASE_INTRO",                                      │
│   "transactionId": 2,                                               │
│   "sessionId": 1,                                                   │
│   "payload": {"phase": "P1"}                                       │
│ }                                                                   │
│                                                                     │
│ game_controller action:                                             │
│ - auto_advance.phase_intro = 0.0 → immediately sends ON_COMPLETE   │
│ - No UI patch (PHASE_INTRO is skipped in _patch_ui_for_state)       │
│                                                                     │
│ Log: "[GC] State update: system=GAME, game=PHASE_INTRO, tx=2, ..." │
│ Log: "[GC] Published /decision/events: type=ON_COMPLETE, tx=2, ..." │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STATE C2: ROUND_SETUP                                               │
│                                                                     │
│ /decision/state:                                                    │
│ {                                                                   │
│   "state": "GAME",                                                  │
│   "gameState": "ROUND_SETUP",                                      │
│   "transactionId": 3,                                               │
│   "payload": {"roundId": 1, "phase": "P1"}                         │
│ }                                                                   │
│                                                                     │
│ game_controller action:                                             │
│ - auto_advance.round_setup = 0.05s → quick ON_COMPLETE             │
│ - UI patch: mode → "game", state patch with system/game state       │
│                                                                     │
│ Log: "[GC] State update: system=GAME, game=ROUND_SETUP, tx=3, ..." │
│ Log: "[GC] Published /decision/events: type=ON_COMPLETE, tx=3, ..." │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STATE C3: QUESTION_PRESENT ← This is where P1 UI renders          │
│                                                                     │
│ /decision/state:                                                    │
│ {                                                                   │
│   "state": "GAME",                                                  │
│   "gameState": "QUESTION_PRESENT",                                 │
│   "transactionId": 4,                                               │
│   "payload": {                                                      │
│     "question": {                                                   │
│       "questionId": 1,                                              │
│       "prompt": "Señala rojo",                                     │
│       "promptVerbal": "Señala rojo",                               │
│       "answer": "rojo",                                             │
│       "options": [                                                  │
│         {"id":"rojo","label":"Rojo","imageUrl":"...","correct":true},│
│         {"id":"azul","label":"Azul","imageUrl":"...","correct":false}│
│       ]                                                             │
│     },                                                              │
│     "phase": "P1"                                                   │
│   }                                                                 │
│ }                                                                   │
│                                                                     │
│ game_controller action (node.py:_on_decision_state):                │
│ 1. Updates translator: _ui_translator.update_state(tx=4,            │
│    game_state="QUESTION_PRESENT", question={...})                   │
│ 2. Caches question in _latest_question_payload                      │
│ 3. Sets _current_phase = "P1"                                       │
│ 4. Calls _patch_ui_for_state("QUESTION_PRESENT", payload)          │
│                                                                     │
│ UI Manifest Patches (manifest_builder.py):                          │
│ - build_state_based_patches("QUESTION_PRESENT", payload):           │
│   a. _answer_type_for_payload(phase="P1") → returns "match"        │
│      File: manifest_builder.py:_answer_type_for_payload()           │
│      P1 always maps to "match" answer type                          │
│   b. format_options_for_ui(options) → [{id, label, image, ...}]    │
│   c. Patches sent:                                                  │
│      - /instances/0/config/question = {text: "Señala rojo", ...}   │
│      - /instances/0/config/options = [{id:"rojo",...},{id:"azul"}]  │
│      - /instances/0/config/answerType = "match"  ← KEY FOR P1      │
│      - /instances/0/config/inputDisabled = true                     │
│                                                                     │
│ 5. auto_advance.question_present = 0.05s → sends ON_COMPLETE       │
│    (or TTS speaks the prompt first if tts.enabled=True)             │
│                                                                     │
│ Log: "[GC] State update: system=GAME, game=QUESTION_PRESENT, ..."  │
│ Log: "Sending manifest patch (N ops)"                               │
│ Log: "[GC] Published /decision/events: type=ON_COMPLETE, tx=4, ..." │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STATE C4: WAIT_INPUT ← User can now interact                       │
│                                                                     │
│ /decision/state:                                                    │
│ {                                                                   │
│   "state": "GAME",                                                  │
│   "gameState": "WAIT_INPUT",                                       │
│   "transactionId": 5,                                               │
│   "payload": {                                                      │
│     "question": { ... same question ... }                           │
│   }                                                                 │
│ }                                                                   │
│                                                                     │
│ game_controller action:                                             │
│ 1. Updates translator state: game_state="WAIT_INPUT", tx=5         │
│ 2. UI patch: inputDisabled = false                                  │
│    (no other options/question patches — kept from QUESTION_PRESENT) │
│ 3. No auto-advance timer (waits for user input)                    │
│                                                                     │
│ *** INPUT GATE IS NOW OPEN ***                                      │
│ _ui_translator._current_game_state == "WAIT_INPUT"                 │
│ _ui_translator._current_transaction_id == 5                         │
│                                                                     │
│ Log: "[GC] State update: system=GAME, game=WAIT_INPUT, tx=5, ..."  │
└─────────────────────────────────────────────────────────────────────┘
```

### Phase D: User Input — THE MATCHING FLOW (P1 Bug Fix Area)

This is the critical path that was broken and is now fixed.

```
┌─────────────────────────────────────────────────────────────────────┐
│ STEP D1: User drags/clicks matching item in UI                     │
│                                                                     │
│ React MatchingPhase component (in emorobcare_components — vendor)   │
│ emits event via runtime.emitUiEvent():                              │
│                                                                     │
│ Event name: "ui_input"                                              │
│ Payload: {                                                          │
│   "leftId": "rojo",                                                 │
│   "rightId": "rojo",                                                │
│   "correct": true                                                   │
│ }                                                                   │
│                                                                     │
│ Shell resolves binding "ui_input" → operation "ui_input"            │
│ → sends to gateway WebSocket                                        │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP D2: Gateway publishes to /ui/input                            │
│                                                                     │
│ File: generic_ui/backend/gateway/ros/operations.py                  │
│ Function: publish(op_id="ui_input", data={leftId, rightId, correct})│
│                                                                     │
│ Publishes std_msgs/String to /ui/input:                             │
│ '{"leftId":"rojo","rightId":"rojo","correct":true}'                │
│                                                                     │
│ Log (backend): "Publishing to /ui/input"                            │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP D3: communication_hub receives /ui/input                      │
│                                                                     │
│ File: communication_hub/ui_intent_bridge.py                         │
│ Function: _on_ui_input(msg)                                         │
│                                                                     │
│ 1. _parse_payload('{"leftId":"rojo","rightId":"rojo","correct":true}')
│    → Returns: {"leftId":"rojo","rightId":"rojo","correct":true}    │
│                                                                     │
│ 2. _extract_intent_text(payload):                                   │
│    a. _extract_control_command → no control keys → ""              │
│    b. Check label/value/text → none present → skip                 │
│    c. *** FIX: Check leftId/rightId → finds leftId="rojo" ***     │
│    d. Returns: "rojo"                                               │
│                                                                     │
│ 3. Builds Intent message:                                           │
│    intent.intent = RAW_USER_INPUT                                   │
│    intent.modality = MODALITY_TOUCHSCREEN                           │
│    intent.data = '{"label":"rojo"}'                                │
│                                                                     │
│ 4. Publishes to /intents                                            │
│                                                                     │
│ Log: "[BRIDGE] Raw /ui/input received: {leftId:rojo,...}"          │
│ Log: "[BRIDGE] Extracted intent from MatchingPhase 'leftId': rojo" │
│ Log: "[BRIDGE] Published /intents RAW_USER_INPUT: rojo"            │
│                                                                     │
│ *** BEFORE FIX: step 2c didn't exist, returned "" → DROPPED ***    │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP D4: game_controller receives /intents                         │
│                                                                     │
│ File: node.py:_on_intent(msg)                                       │
│ Intent.data = '{"label":"rojo"}'                                   │
│                                                                     │
│ 1. _parse_intent_input_data(msg):                                   │
│    a. json.loads('{"label":"rojo"}') → {"label":"rojo"}            │
│    b. Check keys: "label" in data → YES                            │
│    c. _normalize_input_data({"label":"rojo"})                      │
│       - No nested wrappers to unwrap                                │
│       - No nested answer                                            │
│       - label already present → no matching normalization needed    │
│       - Returns: {"label":"rojo"}                                  │
│                                                                     │
│ 2. _map_intent_modality(MODALITY_TOUCHSCREEN) → "touch"            │
│                                                                     │
│ 3. _handle_ui_like_input(                                           │
│      input_data={"label":"rojo"},                                   │
│      modality="touch",                                              │
│      source="/intents"                                              │
│    )                                                                │
│                                                                     │
│ Log: "[GC] /intents received: intent=..., data={\"label\":\"rojo\"}"│
│ Log: "[GC] Parsed intent input_data={'label':'rojo'}, modality=touch│
│ Log: "[GC] Input from /intents: {'label':'rojo'} (system_state=GAME,│
│       game_state=WAIT_INPUT, tx=5, phase=P1)"                      │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP D4-ALT: game_controller ALSO receives /ui/input (fallback)    │
│                                                                     │
│ File: node.py:_on_ui_input(msg)                                     │
│ (runs in parallel with D3→D4 since both subscribe to /ui/input)    │
│                                                                     │
│ 1. _parse_ui_input_data('{"leftId":"rojo","rightId":"rojo",...}')   │
│    a. parse_input_json → {"leftId":"rojo","rightId":"rojo",...}     │
│    b. _normalize_input_data:                                        │
│       - No nested wrappers                                          │
│       - No nested answer                                            │
│       - *** FIX: No label/value → checks leftId → found! ***      │
│       - Sets label="rojo", value="rojo"                             │
│       - correct=true already present                                │
│       - Returns: {"leftId":"rojo","rightId":"rojo","correct":true,  │
│                   "label":"rojo","value":"rojo"}                    │
│                                                                     │
│ 2. _handle_ui_like_input → de-duplicate check:                      │
│    - Same payload as D4 → marked as duplicate → SKIPPED            │
│    (this is correct — avoids double-processing)                     │
│                                                                     │
│ Log: "[GC] Ignored duplicate input from /ui/input: ..."            │
│                                                                     │
│ *** BEFORE FIX: _normalize_input_data couldn't find label/value    │
│     so it returned a dict with only correct/leftId/rightId, and    │
│     extract_user_answer returned value=None → DROPPED ***           │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STEP D5: Input translation                                          │
│                                                                     │
│ File: node.py:_publish_translated_input_event()                     │
│ Calls: _ui_translator.translate_input_data(                         │
│          {"label":"rojo"}, modality="touch"                         │
│        )                                                            │
│                                                                     │
│ File: input_translation.py:InputTranslator.translate_input_data()   │
│                                                                     │
│ 1. translate_input_to_game_control({"label":"rojo"})               │
│    - extract_control_command → "ROJO" not a control → None         │
│                                                                     │
│ 2. Gate check: _current_game_state == "WAIT_INPUT" → ✅ PASS       │
│ 3. Gate check: _current_transaction_id == 5 → ✅ PASS (not None)   │
│                                                                     │
│ 4. translate_input_to_user_intent(                                  │
│      {"label":"rojo"}, tx=5, question={...}, modality="touch"      │
│    )                                                                │
│                                                                     │
│ 5. extract_user_answer({"label":"rojo"}):                           │
│    - value = input_data.get("label") = "rojo"                      │
│    - correct = input_data.get("correct") = None (stripped by bridge)│
│    - Returns: ("rojo", None)                                        │
│                                                                     │
│ 6. correct is None → compute_correct_for_question(question, "rojo")│
│    File: content/correctness.py                                     │
│    - Looks at question.options for matching id/label                │
│    - Finds {"id":"rojo","correct":true} → returns True              │
│                                                                     │
│ 7. Builds USER_INTENT event:                                        │
│    {                                                                │
│      "type": "USER_INTENT",                                        │
│      "payload": {                                                   │
│        "transactionId": 5,                                          │
│        "value": "rojo",                                             │
│        "modality": "touch",                                         │
│        "correct": true                                              │
│      }                                                              │
│    }                                                                │
│                                                                     │
│ Log: "[TRANSLATE] extract_user_answer → value=rojo, correct=None"  │
│ Log: "[TRANSLATE] translate_input_to_user_intent → {type:USER_INTENT│
│ Log: "[GC] Translated event: {type: USER_INTENT, ...}"             │
│ Log: "[GC] Published /decision/events: type=USER_INTENT, tx=5,     │
│       value=rojo, correct=True"                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Phase E: Response from decision_making

```
┌─────────────────────────────────────────────────────────────────────┐
│ STATE E1: CORRECT (if answer was right)                             │
│                                                                     │
│ /decision/state:                                                    │
│ {                                                                   │
│   "state": "GAME",                                                  │
│   "gameState": "CORRECT",                                          │
│   "transactionId": 6,                                               │
│   "payload": {                                                      │
│     "correctOptionId": "rojo",                                      │
│     "phase": "P1"                                                   │
│   }                                                                 │
│ }                                                                   │
│                                                                     │
│ game_controller action (node.py:_on_decision_state):                │
│ 1. _build_correct_feedback_text(payload):                           │
│    - P1 → uses answer label directly: "rojo" → feedback="Rojo"    │
│ 2. _patch_ui_for_state("CORRECT", payload):                        │
│    - Patches: question text = feedback, options = []                │
│    - answerType = "none"                                            │
│    - effect = "confetti" ← triggers confetti animation              │
│ 3. auto_advance.correct = 0.6s → ON_COMPLETE after 600ms          │
│    (or TTS speaks feedback first if enabled)                        │
│                                                                     │
│ Log: "[GC] State update: system=GAME, game=CORRECT, tx=6, ..."    │
│ Log: "Sending manifest patch (N ops)"                               │
│ Log: "[GC] Published /decision/events: type=ON_COMPLETE, tx=6, ..." │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STATE E1-ALT: FAIL_L1 (if answer was wrong)                        │
│                                                                     │
│ /decision/state:                                                    │
│ {                                                                   │
│   "state": "GAME",                                                  │
│   "gameState": "FAIL_L1",                                          │
│   "transactionId": 6,                                               │
│   "payload": {                                                      │
│     "action": "highlight",                                          │
│     "hint": "Inténtalo de nuevo",                                  │
│     "correctOptionId": "rojo"                                       │
│   }                                                                 │
│ }                                                                   │
│                                                                     │
│ game_controller action:                                             │
│ 1. _enrich_failure_payload if action is "say_correct" or FAIL_L2   │
│ 2. UI patch with hint text and highlight options                    │
│ 3. auto_advance.fail_l1 = 2.0s → retries after 2s                 │
│    (loops back to WAIT_INPUT with same question)                    │
│                                                                     │
│ *** NOTE: User input is BLOCKED during FAIL_L1 ***                  │
│ translate_input_data() returns None because                         │
│ _current_game_state != "WAIT_INPUT"                                 │
│                                                                     │
│ Log: "[GC] State update: system=GAME, game=FAIL_L1, tx=6, ..."    │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ STATE E2: Next round (after CORRECT → ON_COMPLETE)                 │
│                                                                     │
│ Decision_making cycles through:                                     │
│ ROUND_SETUP → QUESTION_PRESENT → WAIT_INPUT → ...                 │
│                                                                     │
│ After all P1 rounds complete:                                       │
│ PHASE_COMPLETE → (next phase P2 starts with PHASE_INTRO)           │
│                                                                     │
│ After all phases complete:                                          │
│ GAME_OVER → game_controller returns to IDLE / menu screen          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Key Files Reference

| File | Location | Purpose |
|------|----------|---------|
| `ui_intent_bridge.py` | `communication_hub/communication_hub/` | `/ui/input` → `/intents` bridge |
| `node.py` | `game_controller/game_controller/game_controller/` | Main orchestration node |
| `input_translation.py` | `game_controller/game_controller/game_controller/` | Input→event translation + state gating |
| `decision_events.py` | `game_controller/game_controller/game_controller/` | Event payload builders |
| `manifest_builder.py` | `game_controller/game_controller/game_controller/ui/` | UI manifest JSON patches |
| `manifest_client.py` | `game_controller/game_controller/game_controller/ui/` | ROS service client for manifest updates |
| `builder.py` | `game_controller/game_controller/game_controller/content/` | GAME_INIT payload construction |
| `loaders.py` | `game_controller/game_controller/game_controller/content/` | Game JSON/YAML file loading |
| `correctness.py` | `game_controller/game_controller/game_controller/content/` | Answer correctness computation |
| `auto_advance.py` | `game_controller/game_controller/game_controller/` | Timer-based ON_COMPLETE scheduling |
| `operations.py` | `generic_ui/backend/gateway/ui_gateway/ros/` | Gateway ROS publisher (publishes /ui/input) |

---

## Debug Log Tags

All debug logging added uses consistent tags for easy filtering:

| Tag | Service | File | What it covers |
|-----|---------|------|----------------|
| `[BRIDGE]` | communication_hub | `ui_intent_bridge.py` | /ui/input parsing, intent extraction, /intents publishing |
| `[GC]` | game_controller | `node.py` | State updates, intent parsing, normalization, event publishing |
| `[TRANSLATE]` | game_controller | `input_translation.py` | Answer extraction, state gating, event building |

### Enable Debug Logging

By default ROS 2 nodes log at INFO level. To see DEBUG logs:

```bash
# Option 1: Set log level via environment variable in docker-compose.yml
# Add to game_controller service:
environment:
  - RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Option 2: Set at runtime inside container
docker compose exec game_controller bash
ros2 param set /game_controller rosout.level debug

# Option 3: Use ros2 CLI to change log level dynamically
docker compose exec game_controller ros2 service call /game_controller/set_logger_level rcl_interfaces/srv/SetLoggerLevel "{logger_name: 'game_controller', level: 10}"
```

---

## Troubleshooting Decision Tree

```
User clicks matching item → nothing happens. Where is it stuck?

1. Check backend logs:
   docker compose logs -f backend 2>&1 | grep -i "ui_input\|publish"

   ❌ No "Publishing" log → WebSocket not receiving event from UI
      → Check browser console for WebSocket errors
      → Check if frontend is connected: browser DevTools → Network → WS

   ✅ "Publishing to /ui/input" appears → continue to step 2

2. Check communication_hub logs:
   docker compose logs -f communication_hub 2>&1 | grep "BRIDGE"

   ❌ No "[BRIDGE] Raw /ui/input received" → hub not subscribed
      → Check: docker compose exec communication_hub ros2 topic list
      → Verify /ui/input exists

   ❌ "[BRIDGE] DROPPED - no answer text" → payload format not recognized
      → This was the original bug (now fixed for leftId/rightId)
      → Check the logged payload — what keys does it have?

   ✅ "[BRIDGE] Published /intents RAW_USER_INPUT: rojo" → continue to step 3

3. Check game_controller /intents reception:
   docker compose logs -f game_controller 2>&1 | grep "\[GC\].*intent"

   ❌ No "[GC] /intents received" → gc not subscribed to /intents
      → Check: docker compose exec game_controller ros2 topic info /intents

   ❌ "[GC] Failed to parse /intents data" → intent data format issue
      → Check what communication_hub actually published

   ✅ "[GC] Input from /intents: ..." → continue to step 4

4. Check translation:
   docker compose logs -f game_controller 2>&1 | grep "\[GC\].*Translate\|TRANSLATE"

   ❌ "[GC] Translator returned None" → state gate blocked it
      → Check game_state: should be "WAIT_INPUT"
      → Check tx: should not be None
      → If game_state is something else, the user clicked too early/late

   ❌ "[TRANSLATE] BLOCKED - game_state=QUESTION_PRESENT" → clicked during presentation
      → Wait for WAIT_INPUT state

   ✅ "[GC] Translated event: {type: USER_INTENT, ...}" → continue to step 5

5. Check event publishing:
   docker compose logs -f game_controller 2>&1 | grep "Published /decision/events"

   ❌ No publish log → event was None (shouldn't happen if step 4 passed)

   ✅ "Published /decision/events: type=USER_INTENT, tx=N, value=rojo" → continue to step 6

6. Check decision_making response:
   docker compose logs -f decision_making 2>&1 | grep -i "user_intent\|state\|correct\|fail"

   ❌ No response → decision_making didn't receive or ignored the event
      → Check transactionId matches! (most common cause)
      → docker compose exec decision_making ros2 topic echo /decision/events --once

   ✅ State transitions to CORRECT or FAIL_L1 → continue to step 7

7. Check UI update:
   docker compose logs -f game_controller 2>&1 | grep "manifest patch"

   ❌ No patch → _patch_ui_for_state returned empty
   ✅ "Sending manifest patch (N ops)" → check backend applied it

   docker compose logs -f backend 2>&1 | grep -i "patch\|manifest"
   ✅ "Manifest patched (N ops)" → UI should update
   ❌ Patch failed → check error message, may need full manifest re-send
```

---

## Useful ROS 2 Debug Commands (run inside containers)

```bash
# List all active topics
docker compose exec game_controller ros2 topic list

# Echo a topic in real-time
docker compose exec game_controller ros2 topic echo /decision/state
docker compose exec game_controller ros2 topic echo /decision/events
docker compose exec game_controller ros2 topic echo /intents
docker compose exec game_controller ros2 topic echo /ui/input

# Check topic info (publishers/subscribers count)
docker compose exec game_controller ros2 topic info /intents
docker compose exec game_controller ros2 topic info /ui/input
docker compose exec game_controller ros2 topic info /decision/state
docker compose exec game_controller ros2 topic info /decision/events

# List all nodes
docker compose exec game_controller ros2 node list

# Check node's subscriptions and publications
docker compose exec game_controller ros2 node info /game_controller
docker compose exec game_controller ros2 node info /communication_hub_ui_intent_bridge

# Manually publish a test input (simulate user click)
docker compose exec game_controller ros2 topic pub --once /ui/input std_msgs/msg/String \
  '{data: "{\"leftId\":\"rojo\",\"rightId\":\"rojo\",\"correct\":true}"}'

# Manually publish a button-style input
docker compose exec game_controller ros2 topic pub --once /ui/input std_msgs/msg/String \
  '{data: "{\"label\":\"rojo\",\"value\":\"rojo\",\"correct\":true}"}'

# Check latest decision state
docker compose exec game_controller ros2 topic echo /decision/state --once

# Check manifest service availability
docker compose exec game_controller ros2 service list | grep manifest
```

---

## Testing Commands

```bash
# Unit tests (fast, no external deps)
docker compose -f docker-compose.unit.yml up --build --abort-on-container-exit

# Full integration (game_controller + decision_making + backend)
docker compose -f docker-compose.tests.yml up --build --abort-on-container-exit

# Check test results
ls -la game_controller/test/results/
```

---

## The Bug That Was Fixed (Summary)

**Problem:** MatchingPhase (P1) emits `{leftId, rightId, correct}` but the pipeline only recognized `{label, value, text, answer}`.

**Fix locations (3 files, 4 changes):**

1. **`ui_intent_bridge.py`** — `_extract_intent_text()`: Added `leftId`/`rightId` fallback after `label`/`value`/`text` check.

2. **`node.py`** — `_parse_intent_input_data()`: Added `"leftId"` to the key-existence gate so matching payloads aren't rejected.

3. **`node.py`** — `_normalize_input_data()`: Added block that maps `leftId` → `label`/`value` when neither exists.

4. **`input_translation.py`** — `extract_user_answer()`: Added `leftId` fallback for extracting the answer value (safety net).
