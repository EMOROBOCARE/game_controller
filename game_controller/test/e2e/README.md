# E2E Tests for Game Controller

This directory contains end-to-end tests for the game_controller package.

## Prerequisites

1. Docker daemon available
2. Docker Compose available (`docker compose` preferred, `docker-compose` fallback)
3. Docker stack running:
   ```bash
   cd /path/to/game_controller
   docker compose -f docker-compose.e2e.yml up -d
   ```

3. Wait for all services to be healthy:
   ```bash
   docker compose -f docker-compose.e2e.yml ps
   ```

## Running Tests

All tests should be run inside Docker with the full stack:

```bash
docker compose -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

For standalone pytest runs (outside compose-managed test runner), e2e tests now:
- auto-detect compose implementation (`docker compose` or `docker-compose`)
- resolve service containers dynamically (no hardcoded container names)
- skip cleanly when the docker stack context is unavailable

Useful environment variables:
- `E2E_COMPOSE_FILE`
- `E2E_COMPOSE_PROJECT`
- `E2E_MANAGE_STACK=1` (let `test_game_flow_e2e.py` bring stack up/down)
- `E2E_DECISION_SERVICE`, `E2E_GAME_CONTROLLER_SERVICE`, `E2E_BACKEND_SERVICE`
- `E2E_DECISION_CONTAINER`, `E2E_GAME_CONTROLLER_CONTAINER`, `E2E_BACKEND_CONTAINER`

## Test Coverage

### TestServiceAvailability
- Services running (decision_making, game_controller)
- Required topics exist

### TestGameInitialization
- Game starts on user/game selection

### TestAnswerFlow
- Correct answer advances to next round
- Incorrect answer allows retry

### TestAutoAdvance
- PHASE_INTRO auto-advances correctly

### TestFullGameFlow
- Failure -> retry -> success flow

## Troubleshooting

### Tests timing out

Check container logs:
```bash
docker compose -f docker-compose.e2e.yml logs game_controller
docker compose -f docker-compose.e2e.yml logs decision_making
```

### State not changing

The game state might already be in a different session. Restart the containers:
```bash
docker compose -f docker-compose.e2e.yml restart game_controller decision_making
```

### Connection errors

Ensure the containers are on the same network:
```bash
docker network ls
docker network inspect game_controller_rosnet
```
