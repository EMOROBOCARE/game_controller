# Colores Full Game Simulation

This script simulates a complete colores game session with all 6 phases and captures every manifest state for UI development reference.

## Overview

The `simulate_colores_full_game.py` script:

1. Starts a colores game with all 6 phases (P1, P2, P3, P4_YESNO, P6, P7)
2. Uses basic difficulty (2 colors: red, blue)
3. Captures EVERY manifest state transition
4. Saves manifest snapshots with descriptions and metadata
5. Creates a well-organized folder structure for UI developers

## Output Structure

All captured states are saved to:
```
/home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/
```

Example output structure:
```
colores_full_game/
├── README.md                        # Explains structure and usage
├── index.md                         # Complete state listing
├── 00_initial_menu/                 # Initial menu state
│   ├── manifest.json                # Full manifest
│   ├── decision_state.json          # Decision FSM state
│   └── description.md               # Human-readable docs
├── 01_P1_phase_intro/               # Phase 1 intro
│   ├── manifest.json
│   ├── patches.json                 # Patches applied
│   ├── decision_state.json
│   └── description.md
├── 02_P1_question_1_present/        # Question presented
│   ├── manifest.json
│   ├── patches.json
│   ├── decision_state.json
│   └── description.md
├── 03_P1_question_1_wait_input/     # Waiting for input
│   └── ...
├── 04_P1_question_1_fail_l1/        # Wrong answer feedback
│   └── ...
├── 05_P1_question_1_wait_input_retry/  # Retry after wrong answer
│   └── ...
├── 06_P1_question_1_correct/        # Correct answer feedback
│   └── ...
├── 07_P1_phase_complete/            # Phase complete
│   └── ...
├── 08_P2_phase_intro/               # Next phase...
│   └── ...
... (continues through all 6 phases)
```

## Running the Simulation

### Option 1: Docker Compose (Recommended)

```bash
cd /home/alono/EmorobCare/games_src/game_controller

# Run the simulation
docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit

# View results
ls -la ../ui_developer_manifests/colores_full_game/
```

The simulation will:
1. Start all required mock services (decision_making, generic_ui)
2. Start game_controller
3. Run the complete colores game
4. Save all manifest states
5. Exit automatically when complete

### Option 2: Direct Execution (Advanced)

If you have ROS2 and all dependencies installed locally:

```bash
# Ensure ROS2 environment is sourced
source /opt/ros/humble/setup.bash
source /path/to/game_controller_ws/install/setup.bash

# Set output directory (optional)
export MANIFEST_OUTPUT_DIR=/path/to/output

# Run simulation
python3 test/integration/simulate_colores_full_game.py
```

## Configuration

Environment variables:

- `MANIFEST_OUTPUT_DIR`: Output directory for manifest states
  - Default: `/home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game`
  - Set to customize output location

- `ROS_DOMAIN_ID`: ROS2 domain for isolation
  - Docker compose uses: 88 (dedicated)
  - Adjust if you have domain conflicts

## What Gets Captured

For each state, the script captures:

1. **manifest.json** - Complete manifest at that moment
2. **patches.json** - JSON patches applied to reach this state (when available)
3. **decision_state.json** - Full decision_making FSM state
4. **description.md** - Human-readable documentation including:
   - State description
   - Phase and question number
   - Whether input is enabled
   - Expected user actions
   - Next state transitions
   - Manifest highlights (mode, question, options)

## Use Cases for UI Developers

### 1. Understanding Game Flow

Browse states sequentially to see the complete game progression:
- Menu → Phase intro → Questions → Correct/Wrong feedback → Phase complete → Repeat

### 2. Building UI Components

Use manifest.json files to:
- See exact data structure for each component
- Understand config values at different states
- Build mock data for component testing

### 3. Testing State Transitions

Compare sequential states to understand:
- What changes between states
- How patches modify the manifest
- When input is enabled/disabled

### 4. Debugging UI Issues

- Check if your UI matches expected state
- Verify manifest interpretation
- Compare actual vs. expected UI appearance

### 5. Documentation

- Share with team to explain game flow
- Use as reference for new features
- Create test cases based on real states

## Game Phases Simulated

All 6 phases are included:

### P1 - Matching/Association
"Une los colores iguales" - Match identical colors

### P2 - Voice/Repetition
"Repite el nombre del color" - Repeat the color name

### P3 - Discrimination
"Señala el color correcto" - Point to the correct color

### P4_YESNO - Yes/No Questions
"Responde sí o no a las preguntas" - Answer yes or no

### P6 - Child Asks
"Dime qué color quieres señalar" - Tell which color you want

### P7 - Two-Option Choice
"¿Cuál es el color correcto?" - Which is the correct color?

## Colors Used (Basic Difficulty)

- **red** - Always available (difficulty: null)
- **blue** - Basic level (difficulty: 1)

## Expected State Count

Approximate state counts per phase:
- Phase intro: 1 state
- Each question: 4-6 states (present, wait_input, fail_l1, retry, correct)
- Phase complete: 1 state

Total: ~40-60 states for complete game (varies by questions per phase)

## Troubleshooting

### Simulation hangs or times out

Check that all services are healthy:
```bash
docker compose -f docker-compose.colores-sim.yml ps
```

All services should show "healthy" status.

### Missing output directory

The script creates the directory automatically. If you see errors:
```bash
mkdir -p /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game
```

### Permission errors

Ensure the output directory is writable:
```bash
chmod -R 755 /home/alono/EmorobCare/games_src/ui_developer_manifests
```

### ROS domain conflicts

If running multiple ROS2 systems:
```bash
# Edit docker-compose.colores-sim.yml
# Change ROS_DOMAIN_ID: 88 to another value (0-232)
```

## Related Files

- `simulate_colores_full_game.py` - Main simulation script
- `docker-compose.colores-sim.yml` - Docker compose configuration
- `run_gc_dm_manifest_integration.py` - Standard integration test (reference)
- `../mocks/mock_decision_making.py` - Mock FSM for simulation
- `../mocks/mock_generic_ui.py` - Mock UI backend

## Integration with Existing Tests

This simulation is complementary to existing integration tests:

- **run_gc_dm_manifest_integration.py**: Tests multiple games, control flows (pause/resume/stop)
- **simulate_colores_full_game.py**: Captures complete manifest states for UI reference

Use both:
- Run integration tests for validation
- Run simulation for UI development artifacts

## Next Steps

After running the simulation:

1. **Browse the output**
   ```bash
   cd /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game
   cat README.md
   cat index.md
   ```

2. **Explore sequential states**
   ```bash
   # View state descriptions
   cat 00_initial_menu/description.md
   cat 01_P1_phase_intro/description.md

   # View manifests
   cat 00_initial_menu/manifest.json | jq .
   ```

3. **Use in UI development**
   - Import manifest.json files as mock data
   - Build UI components to match each state
   - Test transitions between states

4. **Share with team**
   - The entire directory is self-contained
   - Can be zipped and shared
   - README.md explains everything

## Contributing

To add more simulations:

1. Copy this script as a template
2. Modify game selection (slug, phases, difficulty)
3. Adjust capture points for your game's flow
4. Update output directory name
5. Create new docker-compose file if needed

Example:
```python
# For animales game with intermediate difficulty
self.publish_game_selection("animales", ["P1", "P3", "P7"], difficulty="intermediate")
```

## License

Part of the EmorobCare game_controller project.
