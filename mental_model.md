# Game Controller Mental Model (Current)

This document describes the current runtime behavior of `game_controller` in this repo.

## Ownership Boundaries

### `decision_making`
- Owns FSM transitions and authoritative `/decision/state`.
- Accepts `/decision/events` envelopes.

### `game_controller`
- Starts sessions (`GAME_INIT`).
- Tracks latest transaction and state context.
- Translates `/intents` to `USER_INTENT`/`GAME_CONTROL`.
- Sends manifest `set` + state-driven `patch` updates.
- Schedules `ON_COMPLETE` with timer or TTS completion gate.

### `generic_ui`
- Renders manifest and emits UI events.

## Current UI Model
Manifest layout is stable and always includes:
- `user_panel` (`UserPanel`)
- `game_screen` (swaps between `GameSelector` and `GameComponent`)

No per-phase layout changes are used. State is rendered by patching `game_screen` config and component.

## Runtime Spine
1. Startup sends initial manifest (`set`).
2. User selection arrives on `/game/user_selector`.
3. Game selection arrives on `/game/game_selector`.
4. Controller builds payload from game metadata and publishes `GAME_INIT`.
5. FSM state updates arrive on `/decision/state`.
6. Controller patches state/phase/question/items/answerType/effect/pause.
7. Inputs arrive on `/intents` and are translated to events.
8. Controller publishes `ON_COMPLETE` when state progression requires it.

## Invariants
- `transactionId` from latest `/decision/state` must be used in `ON_COMPLETE` and `USER_INTENT`.
- Answer intents are forwarded only in `WAIT_INPUT` and `FAIL_L1`.
- On return to `IDLE`, controller switches `game_screen` to `GameSelector` and clears stale gameplay fields.

## Control Semantics
Accepted control labels (from UI input translation path):
- `PAUSE`, `RESUME`
- `STOP` (`EXIT`, `BACK` aliases)
- `RESET` (`RESTART` alias)
- `SKIP_PHASE` (`SKIP` alias)

All are translated to `GAME_CONTROL` and published on `/decision/events`.

## Files To Read First
1. `game_controller/game_controller/game_controller/node.py`
2. `game_controller/game_controller/game_controller/content/builder.py`
3. `game_controller/game_controller/game_controller/input_translation.py`
4. `game_controller/game_controller/game_controller/ui/manifest_builder.py`
5. `INTEGRATION_CONTRACT.md`
