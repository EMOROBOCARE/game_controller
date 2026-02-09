# Quick Start: Colores Game Manifest Capture

Generate a complete reference of all manifest states from a full colores game session.

## What This Does

Simulates a complete colores game with all 6 phases (P1, P2, P3, P4_YESNO, P6, P7) using basic difficulty (red and blue colors) and captures **every single manifest state** to help UI developers:

- Understand the complete game flow
- Build UI components with real manifest data
- Debug UI issues
- Create test data
- Document the system

## Quick Run

```bash
cd /home/alono/EmorobCare/games_src/game_controller

# Run the simulation (takes ~2-3 minutes)
docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit

# View the results
cd ../ui_developer_manifests/colores_full_game
ls -la
```

## What You Get

```
ui_developer_manifests/colores_full_game/
├── README.md                          # Full documentation
├── index.md                           # Complete state list
├── 00_initial_menu/                   # Initial state
│   ├── manifest.json                  # Full manifest
│   ├── decision_state.json            # FSM context
│   └── description.md                 # Human docs
├── 01_P1_phase_intro/                 # Phase 1 intro
├── 02_P1_question_1_present/          # Question shown
├── 03_P1_question_1_wait_input/       # Waiting for input
├── 04_P1_question_1_fail_l1/          # Wrong answer
├── 05_P1_question_1_wait_input_retry/ # Retry
├── 06_P1_question_1_correct/          # Correct!
├── 07_P1_phase_complete/              # Phase done
├── 08_P2_phase_intro/                 # Next phase...
... (~40-60 total states)
```

## How to Use the Output

### 1. Browse the Game Flow

```bash
cd colores_full_game

# Read overview
cat README.md

# See all states
cat index.md

# View specific states
cat 00_initial_menu/description.md
cat 02_P1_question_1_present/description.md
```

### 2. Use Manifests in UI Development

```bash
# View manifest as JSON
cat 03_P1_question_1_wait_input/manifest.json | jq .

# Extract specific fields
cat 03_P1_question_1_wait_input/manifest.json | jq '.instances[] | select(.id == "game_screen")'

# See what changed from previous state
cat 03_P1_question_1_wait_input/patches.json | jq .
```

### 3. Copy for Testing

```bash
# Copy manifests to your UI test fixtures
cp 03_P1_question_1_wait_input/manifest.json /path/to/ui/tests/fixtures/question_state.json
```

### 4. Compare States

```bash
# See difference between two states
diff -u 02_P1_question_1_present/manifest.json 03_P1_question_1_wait_input/manifest.json
```

## Expected Output

You should see approximately:

- **~40-60 total states** depending on questions per phase
- **1 folder per state** with sequential numbering
- **Each folder contains:** manifest.json, description.md, decision_state.json, (patches.json if applicable)
- **All 6 phases covered:** P1, P2, P3, P4_YESNO, P6, P7

## Example State: Wait for Input

**File:** `03_P1_question_1_wait_input/description.md`

```markdown
# P1_question_1_wait_input

**Phase:** P1
**Question:** 1

## Description
Waiting for user input on question 1.
The UI is interactive and ready to receive user's answer.

## User Interactions Available
- Select option: red
- Select option: blue

## Next State Transitions
- User selects correct answer -> CORRECT
- User selects wrong answer -> FAIL_L1

## Manifest Highlights
- Mode: game
- Phase: P1
- Input Disabled: false
```

**File:** `03_P1_question_1_wait_input/manifest.json`

```json
{
  "version": 1,
  "instances": [
    {
      "id": "game_screen",
      "component": "GameScreen",
      "config": {
        "mode": "game",
        "phase": "P1",
        "inputDisabled": false,
        "question": {
          "text": "Señala el color red",
          "questionType": "select"
        },
        "options": [
          {"id": "red", "label": "red", "correct": true},
          {"id": "blue", "label": "blue", "correct": false}
        ]
      }
    }
  ]
}
```

## Troubleshooting

### Simulation doesn't start

```bash
# Check service health
docker compose -f docker-compose.colores-sim.yml ps

# View logs
docker compose -f docker-compose.colores-sim.yml logs
```

### Missing output

```bash
# Ensure output directory exists
mkdir -p /home/alono/EmorobCare/games_src/ui_developer_manifests

# Check permissions
ls -la /home/alono/EmorobCare/games_src/ui_developer_manifests/
```

### Timeout errors

```bash
# Increase timeout (edit simulate_colores_full_game.py)
timeout = 300.0  # Increase from 180.0
```

## Configuration

Edit `docker-compose.colores-sim.yml` to customize:

```yaml
environment:
  MANIFEST_OUTPUT_DIR: /output/colores_full_game  # Change output location
  ROS_DOMAIN_ID: 88                                # Change if conflicts
```

## Next Steps

1. **Explore the output**
   - Read README.md and index.md
   - Browse state descriptions
   - Examine manifest structures

2. **Use in development**
   - Import manifests as test fixtures
   - Build UI components to match states
   - Verify UI behavior

3. **Create more simulations**
   - Different games (animales, frutas, etc.)
   - Different difficulties (intermediate, advanced)
   - Different phase combinations

## More Information

- **Full Documentation:** `test/integration/README_COLORES_SIM.md`
- **Script Source:** `test/integration/simulate_colores_full_game.py`
- **Test Documentation:** `test/README.md`

## Related Commands

```bash
# Run standard integration tests
docker compose -f docker-compose.isolated.yml up --build --abort-on-container-exit

# Run full stack tests
docker compose -f docker-compose.tests.yml up --build --abort-on-container-exit

# View existing manifests
ls -la /home/alono/EmorobCare/games_src/ui_developer_manifests/
```

---

**Time Required:** ~2-3 minutes
**Output Size:** ~2-5 MB (depends on manifest complexity)
**Prerequisites:** Docker, docker-compose
