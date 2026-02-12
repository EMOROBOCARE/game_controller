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

## Repository layout

```
.
├─ communication_hub/              # ROS 2 package
├─ game_controller/                # ROS 2 package (+ docs, config, games)
├─ test/                           # integration tests + Dockerfile
├─ docker-compose.yml              # run the system
├─ docker-compose.tests.yml        # integration test compose
└─ docker-compose.e2e.yml          # e2e test compose
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

- The compose file starts `decision_making`, `game_controller`, `backend`, `web`, and `emorobcare_components_cdn`.
- Exposed ports:
  - UI web shell: `http://localhost:8083`
  - UI backend: `http://localhost:8092`
  - Components CDN: `http://localhost:8084`
- Remote modules are expected at `http://localhost:8084/emorobcare-components/assets/remoteEntry.js`.

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
# NOTE: /intents expects hri_actions_msgs/Intent; for manual CLI testing use /ui/input.
$COMPOSE exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"label\":\"SKIP_PHASE\"}'\''}"'

# 5) Observe state and logs
$COMPOSE exec decision_making bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
   timeout 10 ros2 topic echo /decision/state --once'
$COMPOSE logs -f game_controller backend emorobcare_components_cdn web
```

## Package READMEs

- [`game_controller/README.md`](game_controller/README.md)
- [`communication_hub/README.md`](communication_hub/README.md)

## License

This repo does not currently include a top-level `LICENSE` file. Refer to each package’s `package.xml` for its declared license.
