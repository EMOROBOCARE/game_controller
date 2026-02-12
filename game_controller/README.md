# Game Controller

ROS 2 package that orchestrates Colors/Colores sessions between `decision_making` and `generic_ui`.

## Role
`game_controller` is the integration layer that:
- receives user/game selections
- builds and publishes `GAME_INIT`
- tracks `/decision/state`
- patches UI manifest state
- translates intents to FSM events
- schedules `ON_COMPLETE`

Game logic transitions remain owned by `decision_making`.

## Runtime Channels

### Subscribed
- `/decision/state` (`std_msgs/String` JSON)
- `/intents` (`hri_actions_msgs/Intent`)
- `/ui/input` (`std_msgs/String`, optional direct debug bridge)
- `/game/game_selector` (`std_msgs/String`)
- `/game/user_selector` (`std_msgs/String`)

### Published
- `/decision/events` (`std_msgs/String` JSON)
- `/game/current_user` (`std_msgs/Int16`)

### Service Client
- `/generic_ui/update_manifest` (`generic_ui_interfaces/srv/UpdateManifest`)

## Manifest Model
The controller publishes a stable manifest with two instances:
- `user_panel` (`UserPanel`)
- `game_screen` (switches between `GameSelector` and `GameComponent`)

State updates are sent as JSON Patch operations against `game_screen.config`.

## Quick Start
```bash
cd game_controller
if docker compose version >/dev/null 2>&1; then
  COMPOSE="docker compose"
else
  COMPOSE="docker-compose"
fi

$COMPOSE up --build
```

Or run node directly:
```bash
ros2 launch game_controller game_controller.launch.py
```

## Testing
```bash
if docker compose version >/dev/null 2>&1; then
  COMPOSE="docker compose"
else
  COMPOSE="docker-compose"
fi

$COMPOSE -f docker-compose.tests.yml up --build --abort-on-container-exit
$COMPOSE -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

Notes:
- E2E helpers support both `docker compose` (v2) and `docker-compose` (v1).
- E2E container targeting is service-based (no hardcoded container names).
- E2E tests skip with explicit prerequisite reasons when docker stack context is unavailable.
- `E2E_MANAGE_STACK=1` makes `test_game_flow_e2e.py` own stack lifecycle for standalone runs.

## Manual Input Debug
For compose-only manual testing, publish UI payloads to `/ui/input`:

```bash
docker compose exec game_controller bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"label\":\"SKIP_PHASE\"}'\''}"'
```

`/intents` uses `hri_actions_msgs/Intent` and is intended for `communication_hub` integration.

## Game Content
- Game files are loaded from `game_controller/games/<slug>.yaml` (preferred) with `.json` fallback.
- The `colores` game is currently sourced from `game_controller/games/colores.yaml`.

## Documentation
- `docs/index.md`
- `docs/architecture.md`
- `docs/state_machine.md`
- `docs/ui_integration.md`
- `docs/game_content.md`
- `docs/configuration.md`
