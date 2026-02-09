# How to Run Colores Full Game Simulation

## Quick Start (3 Commands)

```bash
# 1. Navigate to game_controller
cd /home/alono/EmorobCare/games_src/game_controller

# 2. Run simulation
docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit

# 3. View results
ls -la ../ui_developer_manifests/colores_full_game/
```

That's it! The simulation will capture all manifest states from a complete colores game.

## What You Get

After running (takes ~2-3 minutes):

```
ui_developer_manifests/colores_full_game/
├── README.md                 # Complete documentation
├── index.md                  # State listing
├── 00_initial_menu/          # Initial state
├── 01_P1_phase_intro/        # Phase 1 starts
├── 02_P1_question_1_present/
├── 03_P1_question_1_wait_input/
├── 04_P1_question_1_fail_l1/
├── 05_P1_question_1_correct/
... (~40-60 total states)
```

Each folder contains:
- `manifest.json` - Complete manifest
- `decision_state.json` - FSM state
- `description.md` - Human-readable docs
- `patches.json` - JSON patches (when available)

## Validate Setup First (Optional)

```bash
cd test/integration
./validate_colores_sim.sh
```

Should show all green checkmarks ✓

## View Results

```bash
cd ../ui_developer_manifests/colores_full_game

# Read overview
cat README.md

# See all states
cat index.md

# View a state
cat 03_P1_question_1_wait_input/description.md
cat 03_P1_question_1_wait_input/manifest.json | jq .
```

## Use in UI Development

```bash
# Copy manifest for testing
cp 03_P1_question_1_wait_input/manifest.json \
   /path/to/ui/tests/fixtures/

# View what changed between states
diff 02_P1_question_1_present/manifest.json \
     03_P1_question_1_wait_input/manifest.json
```

## Game Configuration

Simulates:
- **Game:** colores
- **Phases:** P1, P2, P3, P4_YESNO, P6, P7 (all 6)
- **Difficulty:** basic (2 colors: red, blue)
- **Rounds:** 1 per phase
- **Paths:** Both correct and incorrect answers

## Documentation

- **Quick Start:** `QUICKSTART_COLORES_SIM.md`
- **Full Docs:** `test/integration/README_COLORES_SIM.md`
- **Setup Guide:** `ui_developer_manifests/colores_full_game/SETUP_INSTRUCTIONS.md`
- **Summary:** `COLORES_SIMULATION_COMPLETE.md`

## Troubleshooting

### Simulation hangs
```bash
docker compose -f docker-compose.colores-sim.yml logs
```

### No output
```bash
mkdir -p /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game
```

### Clean restart
```bash
docker compose -f docker-compose.colores-sim.yml down -v
docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit
```

## Next Steps

1. Run the simulation
2. Browse the output directory
3. Read the generated README.md
4. Use manifests in your UI development
5. Share with your team

---

**Ready to run!** Just execute the 3 commands at the top.
