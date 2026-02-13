# Real Package Integration Plan for `game_controller`

## Goal
Use real ROS 2 packages from `/home/alono/EmorobCare/src` for the `games_src/game_controller` runtime stack, without modifying package source code.

## Current status (as of 2026-02-13)

- Package copies from `/src` are in place and match the real sources.
- Runtime wiring is complete:
  - `game_controller/Dockerfile.compose` copies local mirrored packages.
  - `docker-compose.yml` and `docker-compose.gc_dm_integration.yml` both use that image for runtime services.
- There are no unresolved runtime migration gaps.
- Isolated/unit harnesses still use stubs by design:
  - `game_controller/Dockerfile`
  - `game_controller/Dockerfile.isolated`
  - `test/Dockerfile.test`
  - `test/Dockerfile.mocks`
- `game_controller/Dockerfile.e2e` currently builds only real mirrored `hri_actions_msgs` (no stub path configured in this workspace).

For the process-level discrepancy matrix, see [`GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`](GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md).

## Packages identified as needed

Based on `src/repository.md`, `games_src/game_controller/package.xml`, and runtime/service usage in compose files:

1. `audio_tts_msgs`
2. `hri_actions_msgs`
3. `chatbot_msgs`
4. `emorobcare_led_service`
5. `communication_hub`

## Copy status (completed)

Copied from `src/` into `games_src/game_controller/` unchanged:

- `src/audio_tts_msgs` -> `games_src/game_controller/audio_tts_msgs`
- `src/hri_actions_msgs` -> `games_src/game_controller/hri_actions_msgs`
- `src/chatbot_msgs` -> `games_src/game_controller/chatbot_msgs`
- `src/emorobcare_led_service` -> `games_src/game_controller/emorobcare_led_service`
- `src/communication_hub` -> `games_src/game_controller/communication_hub` (refreshed to real version)

## Notes

- `generic_ui_interfaces` is also required by `game_controller`, but it is not in `/src`; it comes from `games_src/generic_ui/ros2/generic_ui_interfaces`.
- Existing stubs remain in place (`game_controller/stub_*`) for isolated/unit test flows.

## Implementation checklist (status)

1. Update `game_controller/Dockerfile.compose` copy sources to use local real packages.
   - `audio_tts_msgs` from `game_controller/audio_tts_msgs`
   - `hri_actions_msgs` from `game_controller/hri_actions_msgs`
   - `chatbot_msgs` from `game_controller/chatbot_msgs`
   - `emorobcare_led_service` from `game_controller/emorobcare_led_service`
   - `communication_hub` from `game_controller/communication_hub`
   - **Status**: Completed

2. Keep `stub_*` packages for isolated/unit harnesses by process design.
   - **Status**: Completed

## Verification commands used

- `colcon build --symlink-install --packages-select` run with `Dockerfile.compose` image context.
- Manual smoke checks in compose:
  - `/intents`
  - `/expressive_say`
  - `/chatbot/rephrase`
  - `/set_leds`
  - `/generic_ui/update_manifest`

## Migration documentation map

- `REAL_PACKAGE_MIGRATION_DOCUMENTATION.md` keeps the evidence and copy provenance record.
- `GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md` keeps the up-to-date process-level package-path mapping.

## Redundancy Check

- `leds_ros/` is present but has no references in current game_controller code, Dockerfiles, or compose/test processes.
