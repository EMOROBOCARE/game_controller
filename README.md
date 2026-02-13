# Game Controller System (EmorobCare)

This repository contains ROS 2 packages and Docker Compose tooling for running the **Colors (Colores)** game end-to-end:

- [`game_controller/`](game_controller/) — orchestrates the game session, bridges the FSM and UI, and manages auto-advance + manifest updates.
- [`communication_hub/`](communication_hub/) — expressive robot actions (TTS, facial expressions, motions, LEDs) and UI input handling.

The game runtime integrates with two external components:

- **`decision_making`** (phase-based FSM) publishing `/decision/state` and consuming `/decision/events`.
- **`generic_ui`** (React UI + backend) providing the `/generic_ui/update_manifest` service.

## Documentation

Start here:

- [`game_controller/docs/index.md`](game_controller/docs/index.md) — documentation hub and quick links
- [`../integrationDocs/README.md`](../integrationDocs/README.md) — cross-stack integration docs

Deep-dives:

- [`game_controller/docs/architecture.md`](game_controller/docs/architecture.md) — components and data flow
- [`game_controller/docs/state_machine.md`](game_controller/docs/state_machine.md) — FSM states/events integration
- [`game_controller/docs/ui_integration.md`](game_controller/docs/ui_integration.md) — manifest structure, required UI components, patching strategy
- [`game_controller/docs/game_content.md`](game_controller/docs/game_content.md) — game content format (YAML-first in `game_controller/games/*.yaml`, JSON fallback)
- [`game_controller/docs/configuration.md`](game_controller/docs/configuration.md) — ROS parameters, launch args, Docker configuration
- [`IMPLEMENTATION_PLAN_REAL_PACKAGES.md`](IMPLEMENTATION_PLAN_REAL_PACKAGES.md) — plan to wire runtime to local real packages
- [`REAL_PACKAGE_MIGRATION_DOCUMENTATION.md`](REAL_PACKAGE_MIGRATION_DOCUMENTATION.md) — migration audit, package sources, and scope
- [`GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`](GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md) — process-by-process package source mapping and current discrepancies

## Repository layout

 ```
 .
 ├─ communication_hub/              # ROS 2 package
 ├─ game_controller/                # ROS 2 package (+ docs, config, games)
 ├─ audio_tts_msgs/                 # local mirror of real /src package
 ├─ hri_actions_msgs/               # local mirror of real /src package
 ├─ chatbot_msgs/                   # local mirror of real /src package
 ├─ emorobcare_led_service/         # local mirror of real /src package
 ├─ communication_hub/              # local mirror of real /src package
 ├─ test/                           # integration tests + Dockerfile
 ├─ docker-compose.yml                   # run the system
 ├─ docker-compose.tests.yml             # integration test compose
 ├─ docker-compose.e2e.yml               # e2e test compose
 ├─ docker-compose.gc_dm_integration.yml  # headless DM + manifest integration harness
 ├─ docker-compose.isolated.yml          # isolated ROS2 integration stack
 ├─ docker-compose.colores-sim.yml       # complete colores capture simulation
 └─ docker-compose.unit.yml              # unit test compose
 ```

## Prerequisites

- Docker + Docker Compose (`docker compose …` preferred, `docker-compose …` legacy-compatible)
- ROS 2 Humble (only required if running outside Docker)
- Expected sibling repos (used by the compose build contexts):
  - `../generic_ui`
  - `../refactored_game/decision_making_ws`

## Quick start (Docker)

From this directory:

```bash
if docker compose version >/dev/null 2>&1; then
  COMPOSE="docker compose"
else
  COMPOSE="docker-compose"
fi

$COMPOSE up --build
```

Notes:

- The compose file starts `decision_making`, `game_controller`, `llm_service`, `led_service_ros`, `led_service_mock`, `backend`, `web`, and `emorobcare_components_cdn`.
- Exposed ports:
  - UI web shell: `http://localhost:8083`
  - UI backend: `http://localhost:8092`
  - Components CDN: `http://localhost:8084`
  - LED mock UI: `http://localhost:8095`
  - TTS mock UI: `http://localhost:8096`
- Remote modules are expected at `http://localhost:8084/emorobcare-components/assets/remoteEntry.js`.

### Local mock services

- `llm_service` provides `/chatbot/rephrase` and `/chatbot/evaluate_answer` (plus other `/chatbot/*` mock endpoints) for local development.
- `led_service_ros` runs `emorobcare_led_service`. Set `LED_USE_MOCK=1` (default in compose) to use an in-memory LED backend on laptops without hardware.
- `led_service_mock` exposes a small web UI that calls `/set_leds`, `/play_effect`, `/control_leds`, and `/get_led_state`.
- `tts_mock` provides `/say` (`audio_tts_msgs/action/TTS`) and a browser UI. Open `http://localhost:8096`, click `Enable Audio`, and incoming speech from `/expressive_say` will play through laptop speakers.
- `expressive_say_bridge` exposes `/expressive_say` (`audio_tts_msgs/action/Communication`) and forwards to `/say`, matching the communication-hub style chain.
- `game_controller` now reads users from PostgreSQL `children` and writes child answer events to `interaction_logs`.
- Runtime dependency packages are mirrored locally for the stack from real `/src` sources:
  - `game_controller/audio_tts_msgs`
  - `game_controller/hri_actions_msgs`
  - `game_controller/chatbot_msgs`
  - `game_controller/emorobcare_led_service`
  - `game_controller/communication_hub`
- `generic_ui_interfaces` is external to this folder and is sourced from `../generic_ui/ros2/generic_ui_interfaces`.
- `stub_*` packages under `game_controller` remain for isolated/unit test Dockerfiles.

## Current Package Wiring Status (Runtime vs Tests)

- Real source mirrors from `/home/alono/EmorobCare/src` are present in:
  - `audio_tts_msgs/`
  - `hri_actions_msgs/`
  - `chatbot_msgs/`
  - `emorobcare_led_service/`
  - `communication_hub/`
- Runtime compose + integration harness now build from these real mirrored packages.
- `generic_ui_interfaces` remains external and is sourced from `../generic_ui/ros2/generic_ui_interfaces`.
- Isolated/unit-test paths still keep stubs on purpose (`game_controller/stub_*`, `test/Dockerfile.test`).
- For full alignment details by process, see:
  - [`REAL_PACKAGE_MIGRATION_DOCUMENTATION.md`](REAL_PACKAGE_MIGRATION_DOCUMENTATION.md)
  - [`IMPLEMENTATION_PLAN_REAL_PACKAGES.md`](IMPLEMENTATION_PLAN_REAL_PACKAGES.md)
  - [`GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`](GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md)

Service checks:

```bash
docker compose exec llm_service bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 service list | grep "^/chatbot/"'

docker compose exec led_service_ros bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 service list | grep -E "^/(set_leds|play_effect|get_led_state|control_leds)$"'

docker compose exec tts_mock bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 action list | grep -E "^/(say|expressive_say)$"'
```

### Voice I/O quick check (mock TTS + ROS2 intents)

```bash
# 1) In a browser, open http://localhost:8096 and click "Enable Audio"

# 2) Trigger robot speech through the production action contract
docker compose exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 action send_goal /expressive_say audio_tts_msgs/action/Communication \
   "{text: \"Hola, esta es una prueba de voz\", language: \"es\"}"'

# 3) Inject a speech-like user input on /intents (same contract used by game_controller)
docker compose exec game_controller bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /intents hri_actions_msgs/msg/Intent \
   \"{intent: '__raw_user_input__', data: '{\\\"input\\\":\\\"rojo\\\"}', source: '__unknown_agent__', modality: '__modality_speech__', priority: 128, confidence: 1.0}\""
```

If you already run a speech node from `/home/alono/EmorobCare/src` that publishes `hri_actions_msgs/msg/Intent` on `/intents`, it can replace step 3 directly.

## Configuration

- Default parameters: `game_controller/config/game_controller.yaml`
- Full reference and examples: `game_controller/docs/configuration.md`

### UI option payloads

- `PROJECT_ASSET_BASE_URL`: base URL for project/game assets (defaults to `/assets`).
- `EMOROBCARE_PROJECT_ASSET_BASE_URL`: compose-friendly alias for project/game image base (for example `.../emorobcare-components/images`).
- `SHARED_ASSET_BASE_URL`: base URL for shared bundle assets like `images/...` and `fonts/...` (defaults to `/emorobcare-components`).
- `GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL`: optional base URL/path used to resolve remote entry as `<base>/assets/remoteEntry.js`.
- `PROJECT_ASSET_IMAGE_DIR`: directory used when converting symbolic image ids (default `images`).
- `PROJECT_ASSET_ID_EXTENSION`: extension used for symbolic ids (default `png`).
- `ASSET_CDN_URL`: backward-compatible alias for `PROJECT_ASSET_BASE_URL`.
- `GAME_CONTROLLER_REMOTE_ENTRY_URL`: explicit remote entry URL override (full URL/path wins over base URL).
- `GAME_CONTROLLER_INCLUDE_CORRECT_OPTIONS`: when set to `1/true/yes`, includes `correct` in UI option payloads (default is omitted).

### Compose-friendly CDN configuration

`docker-compose.yml` uses one shared base URL to avoid URL drift between shared
assets and module federation entry:

- `EMOROBCARE_COMPONENTS_BASE_URL` (default `http://localhost:8084/emorobcare-components`)
- `EMOROBCARE_PROJECT_ASSET_BASE_URL` (default `http://localhost:8084/emorobcare-components/images`)
- `PROJECT_ASSET_BASE_URL` derives from `EMOROBCARE_PROJECT_ASSET_BASE_URL`
- `SHARED_ASSET_BASE_URL` derives from that base
- `GAME_CONTROLLER_REMOTE_ENTRY_BASE_URL` derives from that base

This prevents `remoteEntry.js` 404 errors caused by mismatched paths.

### PostgreSQL source of truth (users + answers)

`game_controller` reads users from `children` and saves answers in `interaction_logs`.

Compose defaults:

- `POSTGRES_DB=emorobcare_db`
- `POSTGRES_USER=emorobcare`
- `POSTGRES_PASSWORD=D3B3rdad.Emylio`
- `POSTGRES_HOST=10.147.19.11`
- `POSTGRES_PORT=5432`

Disable DB integration with `GAME_CONTROLLER_DB_ENABLED=0` (falls back to static users and disables answer logging).

## Testing

All tests are run via Docker (local pytest is not supported).

```bash
docker compose -f docker-compose.tests.yml up --build --abort-on-container-exit
```

This runs unit tests in `game_controller/test` and integration tests in `test/integration`.

Artifacts (JUnit XML) are written to `test/results/` by the test runner container.

Unit coverage includes correctness-flag handling and multi-image normalization in
`game_controller/test/test_manifest_builder_correct_and_images.py`.

### E2E tests (Docker)

```bash
docker compose -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

Notes:
- `docker-compose.e2e.yml` runs both `test_game_controller_e2e.py` and `test_game_flow_e2e.py`.
- E2E helpers support both `docker compose` and `docker-compose`.
- E2E tests resolve containers by compose service labels (no hardcoded container names).
- In low-context CI runs (missing docker stack), E2E tests skip with explicit prerequisite reasons instead of hard-failing.
- `test_game_flow_e2e.py` does not manage stack lifecycle by default under `docker-compose.e2e.yml`; enable standalone stack lifecycle with `E2E_MANAGE_STACK=1`.

## Manual Colores Run / Debug

```bash
# Compose compatibility helper (v2 preferred, v1 fallback)
if docker compose version >/dev/null 2>&1; then
  COMPOSE="docker compose"
else
  COMPOSE="docker-compose"
fi

# 1) Start stack
$COMPOSE up --build -d

# 2) Validate CDN paths
curl -I http://localhost:8084/emorobcare-components/assets/remoteEntry.js
curl -I http://localhost:8084/emorobcare-components/images/colores.png
curl -I http://localhost:8084/emorobcare-components/images/red_circle.png
curl -I http://localhost:8084/emorobcare-components/images/animales.png

# 3) Start game from ROS topics
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /game/user_selector std_msgs/msg/String "data: \"1\""'
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /game/game_selector std_msgs/msg/String "data: \"colores\""'

# 4) Manual debug input path (compose stack)
# UI publishes /ui/input and communication_hub translates it to /intents.
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"label\":\"SKIP_PHASE\"}'\''}"'
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"pause\"}'\''}"'
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"resume\"}'\''}"'

# 5) Observe state and logs
$COMPOSE exec decision_making bash -lc \
'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   timeout 10 ros2 topic echo /decision/state --once'
$COMPOSE logs -f game_controller backend emorobcare_components_cdn web
```

Latest live snapshots are stored under `game_controller/test/results/`:
- `ACTUAL_RUNTIME_STATE.md`
- `actual_compose_ps.txt`
- `actual_decision_state.txt`
- `actual_manifest_response.txt`
- `actual_runtime_logs_5m.txt`

Expected UI behavior:
- The first phase (`P1`) is matching mode (`answerType: "match"` / Matching components UI).
- If `decision_making` omits `phase` in `QUESTION_PRESENT`, `game_controller` reuses the latest phase so P1 still renders matching mode.

### Complete Colores game (P1–P6)

Use this payload to run the full flow:

```bash
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /game/game_selector std_msgs/msg/String \
   "{data: \"{\\\"game\\\":{\\\"slug\\\":\\\"colores\\\"},\\\"difficulty\\\":\\\"basic\\\",\\\"phases\\\":[\\\"P1\\\",\\\"P2\\\",\\\"P3\\\",\\\"P4\\\",\\\"P5\\\",\\\"P6\\\"],\\\"roundsPerPhase\\\":1}\"}"'
```

Then validate:
- UI on `http://localhost:8083` advances through P1→P6.
- `/chatbot/evaluate_answer` and `/chatbot/rephrase` are available (LLM mock).
- LED mock UI on `http://localhost:8095` reflects `/set_leds`/`/play_effect` activity.

## Package READMEs

- [`game_controller/README.md`](game_controller/README.md)
- [`communication_hub/README.md`](communication_hub/README.md)

## License

This repo does not currently include a top-level `LICENSE` file. Refer to each package’s `package.xml` for its declared license.
