# UI integration (game_controller ↔ generic_ui ↔ emorobcare components)

This package drives the **generic_ui** frontend by sending a manifest to the UI gateway service and then patching it on every decision_making state update.

Source of truth:
- `INTEGRATION_CONTRACT.md` (repo root)
- `UI_integration.md` (repo root)

## Manifest service

`game_controller` uses:
- `/generic_ui/update_manifest` (service): `set` initial manifest + apply RFC6902 JSON patches
- (tests only) `/generic_ui/get_manifest` (service): read last manifest in a headless runner

## Manifest structure (EmorobCare contract)

The manifest uses `generic_ui` v1 keys:
- `componentRegistry`: module federation remotes
- `ops`: ROS operations (topic_pub/topic_sub)
- `layout`: stable grid layout
- `instances`: remote component instances + configs

### Always-present instances

The manifest always contains exactly two instances:
- `UserPanel` (`id: "user_panel"`)
- `GameScreenComponent` (`id: "game_screen"`)

The layout is stable (no unmount/remount when phases change). Only `instances[0].config` is patched.

### Remote components (module federation)

`game_controller` expects the emorobcare remote to expose:
- `./UserPanel`
- `./GameScreenComponent`

And the manifest uses:
- `url: "/emorobcare-components/assets/remoteEntry.js"`
- `scope: "demo"`

## Ops and event emission

The manifest defines these publish ops:
- `user_selector` → `/game/user_selector` (`std_msgs/msg/String`)
- `game_selector` → `/game/game_selector` (`std_msgs/msg/String`)
- `ui_input` → `/ui/input` (`std_msgs/msg/String`)

Because these ops publish `std_msgs/String`, the UI runtime must emit an object with the ROS message field:
- `runtime.emitUiEvent("ui_input", { data: "<string>" })`

Where the `data` string is JSON, for example:
- answer: `{"label":"rojo"}`
- control: `{"label":"PAUSE"}`

Supported control labels (case-insensitive):
- `PAUSE`, `RESUME`
- `STOP` (aliases: `EXIT`, `BACK`)
- `RESET` (alias: `RESTART`)
- `SKIP_PHASE` (alias: `SKIP`)

## `GameScreenComponent` config schema (what the UI must render)

`game_controller` patches `game_screen.config` with:
- `mode`: `"menu"` or `"game"`
- `games`: list for the menu (slug/title/image/supportedPhases/difficulties)
- `state`: `{system, gameState, sessionId, transactionId}` (debug/telemetry)
- `phase`: `"P1" | "P2" | ...` (best-effort: inferred from GAME_INIT mapping when missing in decision state)
- `question`: `{questionId, questionType, text, imgs}`
- `options`: `[{id,label,img,correct,disabled,hidden,highlighted}]`
- `controls`: `{showPause, showResume, showStop, showReset, showSkipPhase}`
- `inputDisabled`: boolean (UI must disable interaction when true)

Behavior expectations:
- When `mode == "menu"`, render a game selector and publish a start payload to `/game/game_selector`.
- When `mode == "game"`, render the current phase/question/options and the controls.
- Respect `inputDisabled` (and/or `state.system == "PAUSED"`) by disabling all input widgets.

## Start game payload (menu)

Publish to `/game/game_selector` (as JSON string in `std_msgs/String.data`):

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

Only `game.slug` is required; the rest are optional overrides.

## Assets (important)

Game content often uses image paths like `assets/...`.
The UI needs a consistent strategy to resolve these into browser-reachable URLs (CDN, static server, or controller-side rewriting).

Until that is standardized, treat `question.imgs[]` and `options[].img` as opaque IDs/paths.

