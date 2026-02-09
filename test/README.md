# Game Controller Tests

Comprehensive test suite for the game_controller ROS2 node.

## Test Levels

### 1. Unit Tests
Pure function tests with no ROS dependencies.

**Location:** `game_controller/test/test_*.py`

**Run:**
```bash
cd game_controller
python3 -m pytest test/test_correctness.py test/test_v2_builder.py -v
```

**Tests:**
- Text normalization and correctness computation
- Payload builder functions
- Filter and transform utilities

### 2. Isolated Integration Tests (NEW)
Full node integration tests using **mock ROS2 services**.

**The Trick:** Instead of depending on external `decision_making_ws`, `generic_ui`, and `communication_hub` repositories, we use lightweight mock nodes that simulate their behavior.

**Location:** `test/integration/test_isolated_integration.py`

**Mocks:**
- `test/mocks/mock_decision_making.py` - Simulates FSM state transitions
- `test/mocks/mock_generic_ui.py` - Simulates UpdateManifest service

**Run:**
```bash
# Run with Docker (recommended)
./test/run_isolated_tests.sh

# Or manually
docker compose -f docker-compose.isolated.yml up --build --abort-on-container-exit
```

**Tests:**
- Game initialization flow
- Transaction ID tracking
- User intent translation and forwarding
- Auto-advance scheduling
- State-based input gating
- Configuration and parameter handling

**Benefits:**
- ✅ No external dependencies required
- ✅ Fast execution (no heavy services)
- ✅ Isolated ROS_DOMAIN_ID (99)
- ✅ Deterministic behavior
- ✅ Easy CI/CD integration

### 3. Integration Tests (Full Stack)
Tests with real `decision_making` and `generic_ui` services.

**Location:** `test/integration/test_game_flow.py`

**Run:**
```bash
docker compose -f docker-compose.tests.yml up --build --abort-on-container-exit
```

**Tests:**
- End-to-end game flow
- Real FSM integration
- Real UI service integration
- Multi-node communication

### 4. Manifest Capture Simulations (NEW)
**Complete game simulations that capture ALL manifest states for UI development.**

**Purpose:** Generate reference manifests for UI developers to understand game flow and build/test UI components.

**Location:** `test/integration/simulate_colores_full_game.py`

**Run:**
```bash
# Run colores full game simulation
docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit

# View captured manifests
ls -la ../ui_developer_manifests/colores_full_game/
```

**Output:** Creates organized directory structure with:
- Complete manifest.json for each state
- JSON patches showing incremental changes
- Human-readable description.md for each state
- Decision state context
- README and index for navigation

**Use Cases:**
- Understanding complete game flow
- Building UI components with real data
- Testing state transitions
- Debugging UI issues
- Creating documentation

**Details:** See `test/integration/README_COLORES_SIM.md` for complete documentation.

### 5. E2E Tests
Full system tests using docker exec for ROS interaction.

**Location:** `test/e2e/test_*.py`

**Run:**
```bash
docker compose -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

**Tests:**
- Complete system with all services
- Real-world scenarios
- Docker container interaction

## Test Architecture

### Isolated Tests Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Docker Container: rosnet_isolated (ROS_DOMAIN_ID=99)   │
│                                                          │
│  ┌────────────────┐    ┌──────────────────┐            │
│  │ Mock Decision  │───→│  Game Controller │            │
│  │    Making      │←───│   (under test)   │            │
│  └────────────────┘    └──────────────────┘            │
│          │                       │                      │
│          │              ┌────────┴────────┐             │
│          │              │                 │             │
│          │         ┌────▼─────┐    ┌─────▼─────┐       │
│          │         │   Mock   │    │   Test    │       │
│          │         │Generic UI│    │  Helper   │       │
│          │         └──────────┘    └───────────┘       │
│          │                                │             │
│          │                                │             │
│          └────────────────────────────────┘             │
│                     Test Runner                         │
└─────────────────────────────────────────────────────────┘
```

### Key Features

**Mock Decision Making:**
- Responds to GAME_INIT, USER_INTENT, ON_COMPLETE, GAME_CONTROL
- Publishes state transitions to /decision/state
- Simulates transaction ID progression
- Validates event structure and timing

**Mock Generic UI:**
- Provides UpdateManifest service
- Validates manifest JSON structure
- Computes manifest hashes
- Tracks set and patch operations

**Test Helper:**
- Subscribes to all relevant topics
- Provides convenience methods for publishing
- Waits for specific events/states
- Stores message history for assertions

## Mock Node Behavior

### Mock Decision Making States

The mock FSM simulates this basic flow:

```
IDLE → GAME_START → PHASE_INTRO → WAIT_INPUT → CORRECT/FAIL_L1 → PHASE_COMPLETE
```

**Transaction ID:** Increments on each state change

**Event Handling:**
- `GAME_INIT` → Transition to GAME_START, then PHASE_INTRO
- `ON_COMPLETE` → Progress to next state (if transaction ID matches)
- `USER_INTENT` → Evaluate answer, transition to CORRECT or FAIL_L1
- `GAME_CONTROL` → Handle PAUSE/RESUME/EXIT

### Mock Generic UI Service

**Service:** `/generic_ui/update_manifest`

**Operations:**
- `set` - Replace entire manifest, validate structure, return hash
- `patch` - Apply JSON patches, validate format, return hash

**Validation:**
- JSON structure validation
- Operation type validation
- Patch format validation (op, path required)

## Running Tests

### Quick Start

```bash
# All isolated tests (recommended for development)
./test/run_isolated_tests.sh

# Full stack tests (requires sibling repos)
docker compose -f docker-compose.tests.yml up --build --abort-on-container-exit

# E2E tests (requires sibling repos)
docker compose -f docker-compose.e2e.yml up --build --abort-on-container-exit
```

### Test Results

Results are saved to `test/results/`:
- `isolated_results.xml` - Isolated integration test results (JUnit XML)
- `results.xml` - Full integration test results
- `e2e_results.xml` - E2E test results

### Debugging Failed Tests

```bash
# View test logs
docker compose -f docker-compose.isolated.yml logs test_runner

# View mock node logs
docker compose -f docker-compose.isolated.yml logs mock_decision_making
docker compose -f docker-compose.isolated.yml logs mock_generic_ui

# Run tests interactively
docker compose -f docker-compose.isolated.yml up -d
docker compose -f docker-compose.isolated.yml exec test_runner bash
# Inside container:
pytest /tests/integration/test_isolated_integration.py -v -s
```

## Writing New Tests

### Adding Unit Tests

Add pure function tests to `game_controller/test/`:

```python
def test_my_function():
    result = my_function(input)
    assert result == expected
```

### Adding Isolated Integration Tests

Add node interaction tests to `test/integration/test_isolated_integration.py`:

```python
def test_my_behavior(test_helper):
    # Setup
    test_helper.clear_messages()

    # Action
    test_helper.publish_game_selection("colores")

    # Assert
    event = test_helper.wait_for_event_type("GAME_INIT", timeout=3.0)
    assert event is not None
```

### Extending Mock Nodes

Enhance mock behavior in `test/mocks/`:

```python
def _handle_new_event_type(self, payload: Dict[str, Any]) -> None:
    """Handle new event type."""
    # Simulate response
    self._publish_state("NEW_STATE", "ACTIVE", payload={...})
```

## CI/CD Integration

The isolated tests are ideal for CI/CD:

```yaml
# Example GitHub Actions
- name: Run Isolated Tests
  run: |
    cd game_controller
    docker compose -f docker-compose.isolated.yml up --build --abort-on-container-exit

- name: Upload Test Results
  uses: actions/upload-artifact@v3
  with:
    name: test-results
    path: test/results/isolated_results.xml
```

**Benefits:**
- No external repository dependencies
- Fast execution (~30-60 seconds)
- Deterministic results
- No authentication/credentials needed
- Can run in parallel

## Troubleshooting

### Tests hang or timeout

- Check that mock nodes are healthy: `docker compose -f docker-compose.isolated.yml ps`
- Verify ROS_DOMAIN_ID is consistent (99 for isolated tests)
- Check logs for initialization errors

### Mock nodes not responding

- Ensure healthchecks are passing
- Check that topics are on correct domain: `ros2 topic list`
- Verify mock node logs for errors

### Import errors in tests

- Ensure game_controller is built: Check test_runner Dockerfile
- Verify test workspace is sourced
- Check Python path includes test directories

## Manifest Capture for UI Development

### Overview

The manifest capture simulations provide a comprehensive reference for UI developers by:

1. **Capturing Every State:** Every manifest update during a complete game session
2. **Organizing Output:** Well-structured directories with descriptive names
3. **Documenting Context:** Each state includes human-readable descriptions
4. **Showing Transitions:** Patches reveal how manifests change incrementally

### Running Colores Full Game Simulation

```bash
cd /home/alono/EmorobCare/games_src/game_controller

# Run simulation
docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit

# Results will be saved to:
# /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/
```

### Output Structure Example

```
colores_full_game/
├── README.md                        # Complete documentation
├── index.md                         # State listing and stats
├── 00_initial_menu/                 # Each state in its own folder
│   ├── manifest.json                # Complete manifest
│   ├── decision_state.json          # FSM state context
│   └── description.md               # Human-readable docs
├── 01_P1_phase_intro/
│   ├── manifest.json
│   ├── patches.json                 # Patches from previous state
│   ├── decision_state.json
│   └── description.md
... (40-60 states total)
```

### What's Captured

Each state includes:

- **Full Manifest:** Complete UI manifest at that moment
- **Patches:** JSON patches that created this state (when available)
- **Decision State:** Complete FSM state including transactionId
- **Description:** Explains what user sees, available actions, next transitions
- **Metadata:** Phase, question number, interaction state

### Use Cases

**For UI Developers:**
- Build components with real manifest data
- Test UI rendering for all game states
- Understand state transition patterns
- Debug UI issues by comparing expected vs actual

**For Documentation:**
- Visual guide to game flow
- Reference for new team members
- Specification for UI requirements

**For Testing:**
- Mock data for unit tests
- Integration test verification
- UI snapshot testing

### Creating New Simulations

To capture manifests for other games:

1. Copy `simulate_colores_full_game.py` as template
2. Modify game configuration:
   ```python
   self.publish_game_selection(
       "animales",  # Different game
       ["P1", "P3", "P7"],  # Different phases
       difficulty="intermediate"  # Different difficulty
   )
   ```
3. Update output directory name
4. Optionally create new docker-compose file

See `test/integration/README_COLORES_SIM.md` for detailed instructions.

## Future Enhancements

- [ ] Add manifest capture for all games (animales, frutas, etc.)
- [ ] Add visualization of manifest transitions (flow diagrams)
- [ ] Add manifest diff tool to compare states
- [ ] Add mock for communication_hub (Intent generation)
- [ ] Add performance benchmarks
- [ ] Add load testing with multiple simultaneous games
- [ ] Add chaos testing (random failures, delays)
- [ ] Add property-based testing with Hypothesis
- [ ] Add mutation testing
- [ ] Add code coverage reporting
