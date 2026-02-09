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
- [`game_controller/docs/game_content.md`](game_controller/docs/game_content.md) — game JSON format (`game_controller/games/*.json`)
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

- Docker + Docker Compose v2 (`docker compose …`)
- ROS 2 Humble (only required if running outside Docker)
- Expected sibling repos (used by the compose build contexts):
  - `../generic_ui`
  - `../refactored_game/decision_making_ws`

## Quick start (Docker)

From this directory:

```bash
docker compose up --build
```

Notes:

- The compose file starts `decision_making`, `game_controller`, and the `generic_ui` backend. (The React web app is managed in the `generic_ui` repo.)
- The backend is published on `http://localhost:8092` (mapped to the backend’s port 8080).

## Configuration

- Default parameters: `game_controller/config/game_controller.yaml`
- Full reference and examples: `game_controller/docs/configuration.md`

### UI option payloads

- `ASSET_CDN_URL`: base URL for rewriting `assets/...` image paths in manifest options (defaults to `http://localhost:8084/emorobcare-components`).
- `GAME_CONTROLLER_INCLUDE_CORRECT_OPTIONS`: when set to `1/true/yes`, includes `correct` in UI option payloads (default is omitted).

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

## Package READMEs

- [`game_controller/README.md`](game_controller/README.md)
- [`communication_hub/README.md`](communication_hub/README.md)

## License

This repo does not currently include a top-level `LICENSE` file. Refer to each package’s `package.xml` for its declared license.
