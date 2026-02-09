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
  - `/game/game_selector`
  - `/game/user_selector`
- Publishes to:
  - `/decision/events`
  - `/game/current_user`
- Calls service:
  - `/generic_ui/update_manifest`

### Content Pipeline
Files:
- `game_controller/game_controller/games/*.json`
- `game_controller/game_controller/games/answers/*.json`
- `game_controller/game_controller/games/phases/generalPhases.json`

`build_game_init_payload()` generates decision-ready rounds from game metadata + answer pools.

### Manifest Pipeline
Files:
- `game_controller/game_controller/game_controller/ui/manifest_builder.py`
- `game_controller/game_controller/game_controller/ui/manifest_client.py`

Manifest model:
- Stable `layout`
- Stable `instances`:
  - `user_panel` (`UserPanel`)
  - `game_screen` (`GameScreenComponent`)
- State-driven JSON patches on `game_screen.config`

### Input Pipeline
Recommended path:
- UI publishes `/ui/input` (`std_msgs/String`)
- `communication_hub` converts to `/intents` (`hri_actions_msgs/Intent`)
- `game_controller` translates to `USER_INTENT` or `GAME_CONTROL`

## End-To-End Data Flow
1. User selected on `/game/user_selector`.
2. Game selected on `/game/game_selector`.
3. `game_controller` publishes `GAME_INIT` on `/decision/events`.
4. `decision_making` publishes `/decision/state` with `transactionId`.
5. `game_controller` updates manifest state/question/options/controls.
6. User answer/control returns as `/intents`.
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
- `./GameScreenComponent`

with federation scope `demo` and remote entry `/emorobcare-components/assets/remoteEntry.js`.
