# Architecture

## Overview
`game_controller` is the orchestration node between UI and FSM logic.

- `decision_making` owns state transitions and game-state truth.
- `game_controller` owns session bootstrap, manifest updates, input translation, and auto-advance triggers.
- `generic_ui` renders whatever the controller publishes in the manifest.

## Components

### `GameControllerNode`
Main node in `game_controller/game_controller/game_controller/node.py`.

Responsibilities:
- Subscribes to:
  - `/decision/state`
  - `/intents`
  - `/ui/input` (when `input_bridge.enabled=true`)
  - `/game/game_selector`
  - `/game/user_selector`
- Publishes to:
  - `/decision/events`
  - `/game/current_user`
- Calls service:
  - `/generic_ui/update_manifest`
  - `/chatbot/rephrase` (optional)
  - `/chatbot/evaluate_answer` (optional)

### Content Pipeline
Files:
- `game_controller/game_controller/games/*.yaml` (preferred)
- `game_controller/game_controller/games/*.json` (legacy fallback)
- `game_controller/game_controller/games/answers/*.json`
- `game_controller/game_controller/games/phases/generalPhases.yaml` (`.json` fallback)

`build_game_init_payload()` generates decision-ready rounds from game metadata + answer pools.

### Manifest Pipeline
Files:
- `game_controller/game_controller/game_controller/ui/manifest_builder.py`
- `game_controller/game_controller/game_controller/ui/manifest_client.py`

Manifest model:
- Stable `layout`
- Stable `instances`:
  - `user_panel` (`UserPanel`)
  - `game_screen` (dynamic `GameSelector`/`GameComponent`)
- State-driven JSON patches on `game_screen.config`

### Input Pipeline
Recommended path:
- UI publishes `/ui/input` (`std_msgs/String`)
- `communication_hub` converts to `/intents` (`hri_actions_msgs/Intent`)
- `game_controller` translates to `USER_INTENT` or `GAME_CONTROL`
- UI volume events publish to `/volume` (`std_msgs/Float32`) via manifest op `volume`

Debug path:
- Publish directly to `/ui/input` in compose stacks that do not include `communication_hub`.
- `USER_INTENT` is accepted only while `decision_making` is in `WAIT_INPUT`.

## End-To-End Data Flow
1. User selected on `/game/user_selector`.
2. Game selected on `/game/game_selector`.
3. `game_controller` publishes `GAME_INIT` on `/decision/events`.
4. `decision_making` publishes `/decision/state` with `transactionId`.
5. `game_controller` updates manifest state/phase/question/items/answerType/pause.
6. User answer/control returns as `/intents` (bridge) or `/ui/input` (direct debug).
7. `game_controller` publishes `USER_INTENT` or `GAME_CONTROL` with current transaction context.

## Auto-Advance Behavior
Configured in `auto_advance.*` params.

Current pattern:
- `PHASE_INTRO`: timer-based `ON_COMPLETE`
- `QUESTION_PRESENT`: expressive TTS completion gate when enabled, timer fallback otherwise
- `CORRECT`, `FAIL_L1`, `FAIL_L2`, `PHASE_COMPLETE`: timer-based `ON_COMPLETE`

## Integration Prerequisites
For browser rendering with current controller manifests, `generic_ui/emorobcare_components` must expose:
- `./UserPanel`
- `./GameSelector` and `./GameComponent`

with federation scope `demo` and remote entry `/emorobcare-components/assets/remoteEntry.js`.

For local docker-compose development, the stack also includes:
- `llm_service` (mock chatbot services)
- `led_service_ros` (LED ROS service with `LED_USE_MOCK=1` by default)
- `led_service_mock` (web UI on `http://localhost:8095` for LED interaction/debug)
