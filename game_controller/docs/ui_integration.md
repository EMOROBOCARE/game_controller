# UI integration (game_controller ↔ generic_ui ↔ emorobcare components)

`game_controller` drives the `generic_ui` frontend by setting an initial manifest and patching it on every `decision_making` state update.

Source of truth:
- `INTEGRATION_CONTRACT.md` (repo root)
- `UI_integration.md` (repo root)

## Manifest service usage

- `/generic_ui/update_manifest`: set initial manifest and apply RFC6902 patches
- `/generic_ui/get_manifest`: tests/debug only

## Manifest structure

Uses `generic_ui` v1 keys:
- `componentRegistry`
- `ops`
- `layout`
- `instances`

### Stable instances

Two instances are always present:
- `UserPanel` (`id: "user_panel"`)
- `game_screen` (`id: "game_screen"`) with dynamic component:
  - `GameSelector` during menu/idle
  - `GameComponent` during gameplay/pause

Layout and instance ids stay stable; controller swaps `instances[game_screen].component` and config.

### Remote modules expected

- `./UserPanel`
- `./GameSelector`
- `./GameComponent`

With:
- `url: "/emorobcare-components/assets/remoteEntry.js"` (default)
  - Override with `GAME_CONTROLLER_REMOTE_ENTRY_URL` (explicit URL), or
  - Set `GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL` to resolve `<base>/assets/remoteEntry.js`.
- `scope: "demo"`

`game_controller` and the component bundle should remain loosely coupled:
- `generic_ui` only needs manifest metadata (`url`, `scope`, `module`)
- component internals/asset locations stay inside the component project

## Ops and emission

Publish ops:
- `user_selector` -> `/game/user_selector` (`std_msgs/msg/String`)
- `game_selector` -> `/game/game_selector` (`std_msgs/msg/String`)
- `ui_input` -> `/ui/input` (`std_msgs/msg/String`)

Runtime must emit ROS message objects:
- `runtime.emitUiEvent("ui_input", { data: "<json-string>" })`

Examples for `data`:
- `{"label":"rojo"}`
- `{"label":"PAUSE"}`

Supported controls:
- `PAUSE`, `RESUME`
- `STOP` (`EXIT`, `BACK`)
- `RESET` (`RESTART`)
- `SKIP_PHASE` (`SKIP`)

## Config shapes rendered by UI

### `GameSelector` config
- `startGameOpId`
- `games`
- optional `username`, `round`
- telemetry: `state`, `phase`

### `GameComponent` config
- `question: {id,text,img}`
- `items: [{label,text?,img?,highlighted?}]`
- `answerType`
  - `match` for phase `P1` (and aliases like `matchingComponents`)
  - `button` for selectable non-matching questions
  - `none` for intro/feedback/non-interactive states
- `answerOpId`
- `effect`
- `pause`
- telemetry: `state`, `phase`

## Start payload shape

Publish to `/game/game_selector` in `std_msgs/String.data`:

```json
{
  "game": { "slug": "colores" },
  "level": { "id": 1, "name": "Nivel 1" },
  "phases": ["P1", "P2", "P3"],
  "difficulty": "basic",
  "roundsPerPhase": 1,
  "questionIdx": 0
}
```
