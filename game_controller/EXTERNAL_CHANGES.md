# External Changes Required

This document describes the changes required in other nodes/packages to integrate with the `game_controller` package. These changes are listed here (rather than implemented directly) as requested.

## Plan (exact sequence)

1) **Align data flow** — keep `communication_hub` publishing `/intents` only; update `game_controller` to translate intents into FSM `/decision/events`.
2) **Add UI visibility** — create a minimal "intent arriving" UI component in `generic_ui`, then expose it in the manifest for quick verification.
3) **Decide selector shape** — confirm whether `game_selector` and `user_selector` remain topics or move to services (see pros/cons below).
4) **Finalize docs** — update the integration diagram and topic/service summary to match the new flow.

## 1. communication_hub

The `communication_hub` node should publish to `/intents` only. The `game_controller` will subscribe to `/intents` and translate those intents into `/decision/events` for the FSM.

### Current Behavior
- `communication_hub` currently publishes to `/intents` using `hri_actions_msgs/msg/Intent`
- It subscribes to `/ui/input` and processes touch input



### Required Changes

No functional changes are required in `communication_hub` beyond ensuring `/intents` contains enough data for downstream translation (label/value, modality, and any "correct" flag if available).

### Notes
- `/ui/input` → `/intents` remains the responsibility of `communication_hub`.
- `/intents` → `/decision/events` translation moves into `game_controller`.

---

## 2. game_controller Intent Translation (in-repo)

Although this document focuses on external changes, the integration now relies on `game_controller` translating incoming `/intents` into FSM events.

**Expected mapping (summary):**
- Control intents (PAUSE/RESUME/RESTART/EXIT/BACK) → `GAME_CONTROL`
- Answer intents → `USER_INTENT` (value, correct?, modality)

This translation should be centralized in `game_controller`, so that `communication_hub` remains intent-focused and FSM-agnostic.

---

## 3. generic_ui Components

Status (2026-02-03): `game_controller` now publishes a stable manifest with only:
- `UserPanel` (`module: "./UserPanel"`)
- `GameScreenComponent` (`module: "./GameScreenComponent"`)

In `generic_ui/emorobcare_components/`, the required external change is to implement and expose `GameScreenComponent`.

### Required: GameScreenComponent

**Expose in module federation** (`generic_ui/emorobcare_components/vite.config.ts`):
- Add `./GameScreenComponent` to `exposes`.
- Keep the federation `name: "demo"` (must match the manifest `scope` used by `game_controller`).

**Config/props contract:**
- Follow `INTEGRATION_CONTRACT.md` (authoritative) and `UI_integration.md`.
- The component must render both:
  - **menu** (user selects a game; publish to `startGameOpId`)
  - **gameplay** (show phase/question/options/controls; publish to `uiInputOpId`)

**Event payload wrapper (required):**
`/ui/input` is `std_msgs/msg/String`, so emitted UI events must wrap JSON in the ROS `data` field:
```ts
runtime.emitUiEvent(uiInputOpId, { data: JSON.stringify({ label: "PAUSE" }) });
```

---

### New: IntentArrivalComponent (debug visibility)

Create a minimal component to confirm that intents are flowing into the frontend. This is intentionally simple and can be restyled later.

**Props:**
```typescript
interface IntentArrivalProps {
  lastIntentLabel: string;
  lastIntentAt: string; // ISO timestamp
}
```

**Behavior:**
- Render a small card or banner that shows the latest intent label and timestamp.
- This should be trivially visible in the UI so integrators can confirm data flow.

**TODO (generic_ui):**
- Add a Miriam TODO in `generic_ui` TODO tracking (e.g., `TODO.md` or project tracker) to beautify this component (color, typography, spacing).

---

## 4. game_selector and user_selector: topic vs service

**Why topics make sense:**
- These values are "state-like" and can be latched by subscribers.
- Late joiners (e.g., `game_controller`) can read the latest selection without explicit request/response.
- The UI can publish selections without waiting for a service server to be available.

**Why services make sense:**
- Provides explicit acknowledgement of selection receipt.
- Better for transactional flows (e.g., "select user" must return success/failure).
- Easy to validate and reject invalid requests centrally.

**Recommendation:**
- Keep topics if selections are simple, idempotent, and should be discoverable by late subscribers.
- Prefer services if selection must be validated and confirmed, or if it should drive synchronous control flow.

---

## 5. decision_making Node

No changes required. The `decision_making` node already:
- Subscribes to `/decision/events`
- Publishes to `/decision/state`
- Handles `GAME_INIT`, `ON_COMPLETE`, `USER_INTENT`, `GAME_CONTROL` events

### Verification

Ensure `ros2_config.yaml` has correct topic configuration:

```yaml
topics:
  events: /decision/events
  state: /decision/state
  ui_update: /ui/update
```

---

## 6. generic_ui Backend

No changes required. The backend already:
- Provides `/generic_ui/update_manifest` service
- Handles WebSocket connections to UI
- Bridges ROS topics/services to UI

### Verification

Ensure the `generic_ui_interfaces` package is built and available:

```bash
ros2 interface show generic_ui_interfaces/srv/UpdateManifest
```

---

## 7. Topic Summary

### Topics Used by game_controller

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/decision/state` | std_msgs/String | Subscribe | Track game state |
| `/intents` | hri_actions_msgs/Intent | Subscribe | User intents from `communication_hub` |
| `/game/game_selector` | std_msgs/String | Subscribe | Game selection |
| `/game/user_selector` | std_msgs/String | Subscribe | User selection |
| `/decision/events` | std_msgs/String | Publish | Send events to FSM |
| `/game/current_user` | std_msgs/Int16 | Publish | Current user ID |

### Service Used

| Service | Type | Purpose |
|---------|------|---------|
| `/generic_ui/update_manifest` | generic_ui_interfaces/srv/UpdateManifest | Update UI |

---

## 8. Message Formats

### /ui/input (from UI)

User answer:
```json
{
  "label": "rojo",
  "correct": true
}
```

Control command:
```json
{
  "label": "PAUSE"
}
```

### /intents (from communication_hub)

Use `hri_actions_msgs/Intent` with the intent label/value and modality populated so `game_controller` can translate to FSM events. (Follow the canonical message definition for exact field names.)

### /decision/events (to decision_making)

GAME_INIT:
```json
{
  "type": "GAME_INIT",
  "payload": {
    "slug": "colores",
    "title": "Colores",
    "introduction": "Vamos a jugar a los colores.",
    "difficulty": "basic",
    "phaseSequence": ["P1", "P2", "P3"],
    "phaseConfigs": { ... },
    "rounds": [ ... ]
  }
}
```

ON_COMPLETE:
```json
{
  "type": "ON_COMPLETE",
  "payload": {
    "transactionId": 5
  }
}
```

USER_INTENT:
```json
{
  "type": "USER_INTENT",
  "payload": {
    "transactionId": 5,
    "value": "rojo",
    "correct": true,
    "modality": "touch"
  }
}
```

GAME_CONTROL:
```json
{
  "type": "GAME_CONTROL",
  "payload": {
    "command": "PAUSE"
  }
}
```

### /decision/state (from decision_making)

```json
{
  "state": "GAME",
  "gameState": "WAIT_INPUT",
  "sessionId": 1,
  "transactionId": 5,
  "payload": {
    "roundId": 1,
    "questionId": 1
  }
}
```

---

## 9. Integration Diagram

```mermaid
flowchart LR
  UI[generic_ui (React UI)]
  HUB[communication_hub]
  GC[game_controller]
  DM[decision_making]
  BACK[generic_ui backend]

  UI -->|/ui/input| HUB
  HUB -->|/intents| GC
  GC -->|/decision/events| DM
  DM -->|/decision/state| GC
  DM -->|/decision/state| BACK
  GC -->|update_manifest| BACK
  BACK -->|/ui/update| UI
```
