# Game Controller Documentation

`game_controller` orchestrates the Colors/Colores gameplay loop between:
- `decision_making` (authoritative FSM)
- `generic_ui` backend (`/generic_ui/update_manifest`)
- UI input channels (`/game/*`, `/ui/input` via `/intents`)

## Read In This Order
1. [Architecture](architecture.md)
2. [State Machine Integration](state_machine.md)
3. [UI Integration](ui_integration.md)
4. [Game Content Format](game_content.md)
5. [Configuration Reference](configuration.md)

## Source Of Truth
For cross-stack payload contracts, use:
- `INTEGRATION_CONTRACT.md`
- `UI_integration.md`

These docs describe package-level behavior. Contract fields and JSON schemas live in `INTEGRATION_CONTRACT.md`.

## Runtime Summary
1. UI selects user on `/game/user_selector`.
2. UI selects game on `/game/game_selector`.
3. `game_controller` builds and publishes `GAME_INIT` to `/decision/events`.
4. `decision_making` publishes `/decision/state`.
5. `game_controller` patches `game_screen.config` in the manifest.
6. UI interactions flow through `/ui/input` -> `communication_hub` -> `/intents`.
7. `game_controller` publishes `USER_INTENT` or `GAME_CONTROL` to `/decision/events`.

## Key Invariants
- `transactionId` from `/decision/state` gates `ON_COMPLETE` and `USER_INTENT`.
- Manifest layout is stable: `UserPanel` + `GameScreenComponent` are always present.
- `QUESTION_PRESENT.payload.question.prompt` must be non-empty (used for expressive TTS completion flow).

## Troubleshooting Shortlist
- Game does not start: verify both user and game selections are published.
- UI does not change: verify `/generic_ui/update_manifest` is available.
- Inputs ignored: verify `/ui/input` is bridged to `/intents` and `gameState` is `WAIT_INPUT`/`FAIL_L1`.
- Session stalls: verify `transactionId` matches latest `/decision/state`.
