# Game Controller Package Source Alignment

## Scope

This file tracks how package sources are currently wired per process after mirroring `/home/alono/EmorobCare/src` packages into `games_src/game_controller`.

### Canonical mirrored packages now present in `games_src/game_controller`

- `audio_tts_msgs`
- `hri_actions_msgs`
- `chatbot_msgs`
- `emorobcare_led_service`
- `communication_hub`

### Source contracts used by `game_controller` code

- Required at runtime:
  - `hri_actions_msgs/Intent`
  - `generic_ui_interfaces/UpdateManifest`
- Optional (feature-gated, handled safely when unavailable):
  - `audio_tts_msgs/Communication`
  - `chatbot_msgs` services `Rephrase` and `EvaluateAnswer`

## Process Mapping

| Process file | Package path inputs | Notes |
|---|---|---|
| `game_controller/Dockerfile.compose` | `generic_ui_interfaces` from `../generic_ui/ros2/generic_ui_interfaces`<br>`hri_actions_msgs` from `game_controller/hri_actions_msgs`<br>`chatbot_msgs` from `game_controller/chatbot_msgs`<br>`audio_tts_msgs` from `game_controller/audio_tts_msgs`<br>`emorobcare_led_service` from `game_controller/emorobcare_led_service`<br>`communication_hub` from `game_controller/communication_hub`<br>`llm_service_mock`, `led_service_mock` local mock packages | Runtime build path for compose services. |
| `docker-compose.yml` | Runtime services (`game_controller`, `communication_hub`, `expressive_say_bridge`, `tts_mock`, `llm_service`, `led_service_ros`, `led_service_mock`) built from `game_controller/Dockerfile.compose` | Runtime package migration is aligned. |
| `docker-compose.gc_dm_integration.yml` | `game_controller` built from `game_controller/Dockerfile.compose`; integration runner uses `test/Dockerfile.gc_dm_integration` | Reuses runtime build for game_controller container. |
| `docker-compose.tests.yml` | `game_controller` built from `game_controller/Dockerfile` with `GENERIC_UI_INTERFACES_PATH=game_controller/stub_generic_ui_interfaces` and `HRI_ACTIONS_MSGS_PATH=game_controller/stub_hri_actions_msgs` | Test isolation; stubs expected. |
| `game_controller/Dockerfile` | `generic_ui_interfaces` and `hri_actions_msgs` from ARG paths (defaults to stubs) | Unit/integration-test base image behavior. |
| `game_controller/Dockerfile.isolated` | `stub_generic_ui_interfaces`, `stub_hri_actions_msgs` | Isolated test image, intentionally stubbed. |
| `game_controller/Dockerfile.e2e` | `generic_ui_interfaces` and `hri_actions_msgs` (local mirror) | E2E runner currently uses the local mirrored message package, not a stub. |
| `docker-compose.e2e.yml` | `game_controller` built from `Dockerfile.compose`; test runner built from `game_controller/Dockerfile.e2e` | Runtime and test runner intentionally split package sources (runtime uses full stack mirrors; e2e runner uses minimal interface mirror set). |
| `test/Dockerfile.gc_dm_integration` | `generic_ui_interfaces` from `generic_ui/ros2/generic_ui_interfaces`<br>`hri_actions_msgs` from `game_controller/hri_actions_msgs`<br>`audio_tts_msgs` from `game_controller/audio_tts_msgs` | Headless integration harness uses local mirrored interface packages. |
| `test/Dockerfile.test` | `game_controller/stub_generic_ui_interfaces` + `game_controller/stub_hri_actions_msgs` | Integration tests isolate from full interface stack. |
| `test/Dockerfile.test_isolated` | `game_controller` source, `test/stubs/generic_ui_interfaces`, and cloned `hri_actions_msgs` | Used by isolated suites and colors simulation harnesses. |
| `docker-compose.isolated.yml` / `docker-compose.colores-sim.yml` | `game_controller` built from `game_controller/Dockerfile.isolated`; mocked decision and generic UI nodes | Fully isolated process graph, stubs + mocked peers by design. |
| `test/Dockerfile.mocks` and related isolated suites | Clones remote hri-actions package + local stubs for generic UI | Mocks for decision-making and generic UI peers. |
| `docker-compose.unit.yml` | Unit tests from `test/Dockerfile.test` | Unit container for quick test-only feedback. |

## Discrepancy Register (2026-02-13)

1. `game_controller/Dockerfile.e2e` currently uses local `hri_actions_msgs` (no stub path configured) despite the file name implying a stub-lean variant.
2. `test/Dockerfile.test`, `game_controller/Dockerfile.isolated`, and `docker-compose.isolated.yml` intentionally isolate from full stack.
3. `test/Dockerfile.gc_dm_integration` and `test/Dockerfile.test_isolated` intentionally avoid `chatbot_msgs` because they target decision/UI flow or harness isolation, not chatbot-service behavior.

## Redundant Artifacts (Verified)

| Artifact | Why it is redundant | Evidence |
|---|---|---|
| `leds_ros/` directory | Present in repository but not consumed by any compose file, Dockerfile, test runner, or runtime node imports | No references to `leds_ros` in `docker-compose*.yml`, `game_controller/Dockerfile*`, `test/Dockerfile*`, `game_controller/**/*.py`, or `communication_hub/**/*.py` |

## Non-Redundant Documentation Direction

- `REAL_PACKAGE_MIGRATION_DOCUMENTATION.md` keeps source evidence (copy sources, verification commands).
- `IMPLEMENTATION_PLAN_REAL_PACKAGES.md` tracks pending/implemented migration work.
- `GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md` (this file) keeps the process-level discrepancy matrix.

Use this file to validate future process updates before changing package wiring.
