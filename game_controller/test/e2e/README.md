# E2E Tests for Game Controller

This directory contains end-to-end tests for the game_controller package.

## Prerequisites

1. Docker and docker-compose installed
2. Docker stack running:
   ```bash
   cd /path/to/game_controller
   docker compose up -d
   ```

3. Wait for all services to be healthy:
   ```bash
   docker compose ps
   ```

## Running Tests

All tests should be run inside Docker with the full stack:

```bash
docker compose -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

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
docker compose logs game_controller
docker compose logs decision_making
```

### State not changing

The game state might already be in a different session. Restart the containers:
```bash
docker compose restart game_controller decision_making
```

### Connection errors

Ensure the containers are on the same network:
```bash
docker network ls
docker network inspect game_controller_rosnet
```
