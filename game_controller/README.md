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
- `game_screen` (`GameScreenComponent`)

State updates are sent as JSON Patch operations against `game_screen.config`.

## Quick Start
```bash
cd game_controller
docker compose up --build
```

Or run node directly:
```bash
ros2 launch game_controller game_controller.launch.py
```

## Testing
```bash
docker compose -f docker-compose.tests.yml up --build --abort-on-container-exit
docker compose -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

## Documentation
- `docs/index.md`
- `docs/architecture.md`
- `docs/state_machine.md`
- `docs/ui_integration.md`
- `docs/game_content.md`
- `docs/configuration.md`
