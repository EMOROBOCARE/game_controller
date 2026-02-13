# Game Controller Real Package Migration Documentation

## 1) Request and Scope

Requested objective:

- Identify which packages from `/home/alono/EmorobCare/src` are needed by `game_controller`.
- Copy those packages into `games_src/game_controller` to use real versions.
- Do not modify copied package contents.
- Leave an implementation plan inside `game_controller`.

Note:

- The requested path `game_src/game_controller` does not exist in this workspace.
- Work was performed in `games_src/game_controller`.

## 2) Inputs Reviewed

The following sources were used to identify requirements:

- `src/repository.md`
- `games_src/game_controller/package.xml`
- `games_src/game_controller/game_controller/node.py`
- `games_src/game_controller/game_controller/chatbot_client.py`
- `games_src/game_controller/game_controller/tts_client.py`
- `games_src/game_controller/game_controller/ui_intent_bridge.py`
- `games_src/game_controller/Dockerfile.compose`
- `games_src/game_controller/docker-compose.yml`

## 3) Dependency Identification Result

### 3.1 Real packages needed from `/src`

1. `audio_tts_msgs`
2. `hri_actions_msgs`
3. `chatbot_msgs`
4. `emorobcare_led_service`
5. `communication_hub`

### 3.2 Why each is needed

- `audio_tts_msgs`: used for TTS and expressive communication actions.
- `hri_actions_msgs`: required for `/intents` message type (`Intent`).
- `chatbot_msgs`: required by chatbot client/services (`/chatbot/rephrase`, `/chatbot/evaluate_answer`, etc.).
- `emorobcare_led_service`: required for LED service API (`/set_leds`, `/play_effect`, etc.) used in compose stack.
- `communication_hub`: part of the interaction pipeline described in repository map and local stack documentation.

### 3.3 Dependency not in `/src`

- `generic_ui_interfaces` is required by `game_controller` but is not in `/src`.
- Source for this workspace is `games_src/generic_ui/ros2/generic_ui_interfaces`.

## 4) Copy Operations Performed

Copied unchanged from `/src` into `games_src/game_controller`:

- `src/audio_tts_msgs` -> `games_src/game_controller/audio_tts_msgs`
- `src/hri_actions_msgs` -> `games_src/game_controller/hri_actions_msgs`
- `src/chatbot_msgs` -> `games_src/game_controller/chatbot_msgs`
- `src/emorobcare_led_service` -> `games_src/game_controller/emorobcare_led_service`
- `src/communication_hub` -> `games_src/game_controller/communication_hub` (refreshed)

This migration captures copy operations and verification only. Runtime/runtime-test wiring mismatches are tracked in [`GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`](GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md).

Copy method:

- `rsync -a --exclude '.git' ...`
- No edits were made inside copied package files.

## 5) Verification Performed

Post-copy checks:

- `diff -qr --exclude .git src/audio_tts_msgs games_src/game_controller/audio_tts_msgs`
- `diff -qr --exclude .git src/hri_actions_msgs games_src/game_controller/hri_actions_msgs`
- `diff -qr --exclude .git src/chatbot_msgs games_src/game_controller/chatbot_msgs`
- `diff -qr --exclude .git src/emorobcare_led_service games_src/game_controller/emorobcare_led_service`
- `diff -qr --exclude .git src/communication_hub games_src/game_controller/communication_hub`

Result:

- No differences reported for the copied package trees.

## 6) Existing Workspace State Observed

`games_src/game_controller` already had unrelated in-progress changes before this task (including tracked/untracked edits).  
Those were preserved and not reverted.

## 7) Plan File Created

Implementation plan file added:

- `games_src/game_controller/IMPLEMENTATION_PLAN_REAL_PACKAGES.md`

This migration captured copy operations and runtime migration state; compose/build path migration is now complete for the runtime stack.
For process-level status and discrepancy tracking, see [`GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`](GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md).

## 8) Current Runtime Status

Runtime compose stack migration is complete:

- `game_controller/Dockerfile.compose` now copies local mirrored packages.
- `docker-compose.yml` uses this Dockerfile for all runtime services that need those interfaces.
- No additional runtime migration steps are pending.

Test and isolated flow exceptions are intentional and tracked in
[`GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`](GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md):

- `Dockerfile.isolated`, `Dockerfile.test`, and `test/Dockerfile.mocks` keep stubs for deterministic test harness behavior.
- `Dockerfile.e2e` currently uses the local mirrored `hri_actions_msgs` instead of a stub in this workspace.
- `chatbot_msgs` remains optional in `game_controller` and is not required by the headless `gc_dm` integration harness.

## 9) Redundancy Clarification

- `leds_ros/` is present in this repository but not used by any active game_controller runtime process path:
  - not referenced by source imports
  - not built into `docker-compose*.yml` service images
  - not included in active test Dockerfiles
  - no launch or node path includes it

## 10) Documentation Register

- `IMPLEMENTATION_PLAN_REAL_PACKAGES.md` is the concise action list.
- `GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md` is the process-level matrix and discrepancy register.
- `REAL_PACKAGE_MIGRATION_DOCUMENTATION.md` remains the evidence + copy provenance record.
