# Game Controller Documentation

`game_controller` orchestrates the Colors/Colores gameplay loop between:
- `decision_making` (authoritative FSM)
- `generic_ui` backend (`/generic_ui/update_manifest`)
- UI input channels (`/game/*` for selection, `/ui/input` translated by `communication_hub` into `/intents`)
- local development services (`communication_hub`, `llm_service`, `led_service_ros`, `led_service_mock`)

## Read In This Order
1. [Architecture](architecture.md)
2. [State Machine Integration](state_machine.md)
3. [UI Integration](ui_integration.md)
4. [Game Content Format](game_content.md) (YAML-first, JSON fallback)
5. [Configuration Reference](configuration.md)

## Source Of Truth
For cross-stack payload contracts, use:
- `INTEGRATION_CONTRACT.md`
- `UI_integration.md`
- `REAL_PACKAGE_MIGRATION_DOCUMENTATION.md` and `IMPLEMENTATION_PLAN_REAL_PACKAGES.md` in the workspace root for package-source migration status
- `GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md` for process-level package-path wiring discrepancies

These docs describe package-level behavior. Contract fields and JSON schemas live in `INTEGRATION_CONTRACT.md`.

## Runtime Summary
1. UI selects user on `/game/user_selector`.
2. UI selects game on `/game/game_selector`.
3. `game_controller` builds and publishes `GAME_INIT` to `/decision/events`.
4. `decision_making` publishes `/decision/state`.
5. `game_controller` patches `game_screen.config` in the manifest.
6. UI interactions are translated by `communication_hub` (`/ui/input` -> `/intents`).
7. `game_controller` publishes `USER_INTENT` or `GAME_CONTROL` to `/decision/events`.

## Key Invariants
- `transactionId` from `/decision/state` gates `ON_COMPLETE` and `USER_INTENT`.
- Manifest layout is stable: `UserPanel` is fixed and `game_screen` swaps between `GameSelector` (menu) and `GameComponent` (gameplay).
- `QUESTION_PRESENT.payload.question.prompt` is read and spoken when present; speech can be disabled per question with `question.say_prompt=false`.

## Troubleshooting Shortlist
- Game does not start: verify both user and game selections are published.
- UI does not change: verify `/generic_ui/update_manifest` is available.
- UI fails to load remote components: verify `remoteEntry.js` is reachable at
  `http://localhost:8084/emorobcare-components/assets/remoteEntry.js`.
- Inputs ignored: verify `communication_hub` is up, `/intents` is active, and `gameState` is `WAIT_INPUT`.
- Pause/Resume ignored: verify UI sends `/ui/input` with `action` (`pause`/`resume`) or `label`, and `communication_hub` republishes `PAUSE`/`RESUME` on `/intents`.
- Chatbot-dependent phases fail: verify `/chatbot/rephrase` and `/chatbot/evaluate_answer` in `llm_service`.
- LED feedback missing: verify `/set_leds` and `/play_effect` in `led_service_ros`, and open `http://localhost:8095`.
- Repeated `Inténtalo de nuevo` without progress: verify inputs are not forcing
  `USER_INTENT` during `FAIL_L1`; that state advances by timer.
- Session stalls: verify `transactionId` matches latest `/decision/state`.
