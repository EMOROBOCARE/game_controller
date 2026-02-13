# Configuration Reference

## Overview
`game_controller` is configured through ROS parameters (declared in `node.py`) plus a small set of environment variables used by manifest rendering behavior.

## ROS Parameters

### Topics
- `topics.decision_state` (default `/decision/state`)
- `topics.intents` (default `/intents`)
- `topics.ui_input` (default `/ui/input`)
- `topics.game_selector` (default `/game/game_selector`)
- `topics.user_selector` (default `/game/user_selector`)
- `topics.decision_events` (default `/decision/events`)
- `topics.current_user` (default `/game/current_user`)
- `input_bridge.enabled` (default `true`, enables direct `/ui/input` subscriber)
- `input_bridge.dedupe_window_sec` (default `0.35`)

### Generic UI
- `generic_ui.update_manifest_service` (default `/generic_ui/update_manifest`)
- `generic_ui.manifest_timeout_sec` (default `5.0`)

### Auto-Advance (seconds)
- `auto_advance.phase_intro` (default `0.0`, intro skip)
- `auto_advance.round_setup` (default `0.05`)
- `auto_advance.question_present` (default `0.05`)
- `auto_advance.fail_l1` (default `2.0`)
- `auto_advance.fail_l2` (default `2.0`)
- `auto_advance.correct` (default `0.6`)
- `auto_advance.correct_min_display` (default `3.0`, minimum time to keep CORRECT state visible before auto-advance when prompt feedback is spoken)
- `auto_advance.correct_p1` (default `3.0`, legacy compatibility override for first P1 success delay)
- `auto_advance.phase_complete` (default `0.3`)

### TTS Gate
- `tts.enabled` (default `false`)
- `tts.action_server` (default `/expressive_say`)
- `tts.language` (default `es`)
- `tts.server_wait_timeout_sec` (default `0.2`)
- `tts.rephrase_question_enabled` (default `true`)
- `tts.rephrase_correct_enabled` (default `true`)

### Chatbot
- `chatbot.enabled` (default `true`)
- `chatbot.rephrase_service` (default `/chatbot/rephrase`)
- `chatbot.evaluate_service` (default `/chatbot/evaluate_answer`)
- `chatbot.service_wait_timeout_sec` (default `0.05`)
- `chatbot.evaluate_speech_only` (default `true`)

### Game Defaults
- `game_defaults.difficulty` (default `basic`)
- `game_defaults.rounds_per_phase` (default `2`)
- `game_defaults.phases` (default `['P1','P2','P3']`)

## Configuration File
Default file:
- `game_controller/config/game_controller.yaml`

Use custom params at launch:

```bash
ros2 launch game_controller game_controller.launch.py \
  difficulty:=intermediate \
  rounds_per_phase:=3 \
  phases:="['P1','P2','P3','P4']"
```

Legacy aliases are still accepted in phase lists:
- `P4_YESNO` maps to `P4`
- `P7` maps to `P6` when legacy numbering is detected

## Environment Variables

### Runtime/ROS
- `ROS_DOMAIN_ID`
- `ROS_LOCALHOST_ONLY`
- `LED_USE_MOCK`
  - `1/true/yes/on` forces `emorobcare_led_service` to use in-memory mock backend.
  - Default in compose: `1` (laptop-safe).
- `LED_NUM_LEDS`
  - Number of LEDs exposed by mock/hardware service.
  - Default in compose: `5`.
- `LED_MOCK_UI_HOST`
  - Bind host for `led_service_mock` web UI.
  - Default: `0.0.0.0`.
- `LED_MOCK_UI_PORT`
  - Port for `led_service_mock` web UI.
  - Default: `8095`.

### Manifest/UI helpers
- `PROJECT_ASSET_BASE_URL`
  - Base used to rewrite project-specific `assets/...` paths.
  - Default: `/assets`
- `EMOROBCARE_PROJECT_ASSET_BASE_URL`
  - Compose-friendly alias used when `PROJECT_ASSET_BASE_URL` is empty/unset.
  - Typical value: `http://localhost:8084/emorobcare-components/images`
- `SHARED_ASSET_BASE_URL`
  - Base used for shared bundle assets (`shared/...`, `images/...`, `fonts/...`).
  - Default: `/emorobcare-components`
- `GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL`
  - Base used to resolve remote entry as `<base>/assets/remoteEntry.js`.
  - Useful in Docker Compose to keep remote-entry and shared-assets on the same base.
  - Default: uses `SHARED_ASSET_BASE_URL`
- `PROJECT_ASSET_IMAGE_DIR`
  - Folder used when converting symbolic asset ids (for example `red_circle`).
  - Default: `images`
- `PROJECT_ASSET_ID_EXTENSION`
  - Extension used when converting symbolic asset ids.
  - Default: `png`
- `ASSET_CDN_URL`
  - Backward-compatible alias for `PROJECT_ASSET_BASE_URL`.
- `GAME_CONTROLLER_REMOTE_ENTRY_URL`
  - Explicit component bundle remote entry URL override.
  - If set, it takes precedence over `GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL`.
  - Default: `<SHARED_ASSET_BASE_URL>/assets/remoteEntry.js`
- `GAME_CONTROLLER_REMOTE_ENTRY_VERSION`
  - Cache-busting query parameter appended to remote entry URL.
  - Default: `20260210`
- `GAME_CONTROLLER_INCLUDE_CORRECT_OPTIONS`
  - When truthy (`1/true/yes/on`), include `correct` field in UI options.

## Docker Notes
Typical dev mount points in `game_controller/docker-compose.yml`:
- `./game_controller/games:/ros2_ws/src/game_controller/games:ro`

This allows iterating game metadata without rebuilding the image.

Default local services in `docker-compose.yml`:
- `llm_service` (`/chatbot/rephrase`, `/chatbot/evaluate_answer`, plus `/chatbot/*` mocks)
- `led_service_ros` (`/set_leds`, `/play_effect`, `/get_led_state`, `/control_leds`)
- `led_service_mock` UI (`http://localhost:8095`)

Recommended compose env pattern:
- `EMOROBCARE_COMPONENTS_BASE_URL=http://localhost:8084/emorobcare-components`
- `EMOROBCARE_PROJECT_ASSET_BASE_URL=http://localhost:8084/emorobcare-components/images`
- `PROJECT_ASSET_BASE_URL=${EMOROBCARE_PROJECT_ASSET_BASE_URL}`
- `SHARED_ASSET_BASE_URL=${EMOROBCARE_COMPONENTS_BASE_URL}`
- `GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL=${EMOROBCARE_COMPONENTS_BASE_URL}`

## Quick Validation
```bash
# list params
ros2 param list /game_controller

# inspect one value
ros2 param get /game_controller auto_advance.phase_intro

# debug logs
ros2 launch game_controller game_controller.launch.py \
  --ros-args --log-level game_controller:=DEBUG
```

## Common Misconfigurations
- Manifest updates fail: `generic_ui` backend not providing `/generic_ui/update_manifest`.
- Inputs ignored: neither `/ui/input` (direct) nor `/intents` (bridge) is connected.
- Images missing in UI: `PROJECT_ASSET_BASE_URL` / `EMOROBCARE_PROJECT_ASSET_BASE_URL` / `SHARED_ASSET_BASE_URL` do not match deployed paths.
