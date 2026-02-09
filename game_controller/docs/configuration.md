# Configuration Reference

## Overview
`game_controller` is configured through ROS parameters (declared in `node.py`) plus a small set of environment variables used by manifest rendering behavior.

## ROS Parameters

### Topics
- `topics.decision_state` (default `/decision/state`)
- `topics.intents` (default `/intents`)
- `topics.game_selector` (default `/game/game_selector`)
- `topics.user_selector` (default `/game/user_selector`)
- `topics.decision_events` (default `/decision/events`)
- `topics.current_user` (default `/game/current_user`)

### Generic UI
- `generic_ui.update_manifest_service` (default `/generic_ui/update_manifest`)
- `generic_ui.manifest_timeout_sec` (default `5.0`)

### Auto-Advance (seconds)
- `auto_advance.phase_intro` (default `2.0`)
- `auto_advance.round_setup` (default `0.05`)
- `auto_advance.question_present` (default `0.05`)
- `auto_advance.fail_l1` (default `2.0`)
- `auto_advance.fail_l2` (default `2.0`)
- `auto_advance.correct` (default `0.6`)
- `auto_advance.phase_complete` (default `0.3`)

### TTS Gate
- `tts.enabled` (default `true`)
- `tts.action_server` (default `/expressive_say`)
- `tts.language` (default `es`)
- `tts.server_wait_timeout_sec` (default `0.2`)

### Game Defaults
- `game_defaults.difficulty` (default `basic`)
- `game_defaults.rounds_per_phase` (default `2`)
- `game_defaults.phases` (default `['P1','P2','P3']`)

## Configuration File
Default file:
- `game_controller/game_controller/config/game_controller.yaml`

Use custom params at launch:

```bash
ros2 launch game_controller game_controller.launch.py \
  difficulty:=intermediate \
  rounds_per_phase:=3 \
  phases:="['P1','P2','P3','P4_YESNO']"
```

## Environment Variables

### Runtime/ROS
- `ROS_DOMAIN_ID`
- `ROS_LOCALHOST_ONLY`

### Manifest/UI helpers
- `ASSET_CDN_URL`
  - Base used to rewrite `assets/...` paths for browser access.
  - Default: `http://localhost:8084/emorobcare-components`
- `GAME_CONTROLLER_INCLUDE_CORRECT_OPTIONS`
  - When truthy (`1/true/yes/on`), include `correct` field in UI options.

## Docker Notes
Typical dev mount points in `game_controller/docker-compose.yml`:
- `./game_controller/games:/ros2_ws/src/game_controller/games:ro`

This allows iterating game metadata without rebuilding the image.

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
- Inputs ignored: `/ui/input` is not bridged to `/intents`.
- Images missing in UI: `ASSET_CDN_URL` does not match deployed CDN path.
