# Manifest Organization for UI Developers

This document explains the post-processing script that organizes integration test results into a UI-developer-friendly structure.

## Overview

Instead of creating a complex simulator for the Colores game, we use the **PROVEN WORKING** integration test (`docker-compose.gc_dm_integration.yml`) which already successfully generates all the manifests we need. A post-processing script then organizes these results into an easy-to-browse structure for UI developers.

## Architecture

```
Integration Test          Post-Processor           UI Developer Output
─────────────────        ───────────────          ────────────────────
┌─────────────────┐      ┌─────────────┐          ┌──────────────────┐
│ Docker Compose  │      │ Python      │          │ 57 State Dirs    │
│ Integration     │──┬──▶│ Organizer   │─────────▶│ - manifest.json  │
│ Test            │  │   │ Script      │          │ - decision_*.json│
└─────────────────┘  │   └─────────────┘          │ - description.md │
                     │                             └──────────────────┘
                     │                             ┌──────────────────┐
                     └────────────────────────────▶│ README.md        │
                       (reads JSONL logs)          │ index.md         │
                                                   └──────────────────┘
```

## Key Files

### Scripts

- **`organize_manifests_for_ui.py`**: Post-processor that reads the integration test logs and creates the organized output
- **`run_and_organize.sh`**: Convenience script that runs both the integration test and the post-processor

### Input Files (from integration test)

- **`test/results/manifest_log.jsonl`**: All manifest updates captured during the test (57 entries)
- **`test/results/decision_state_log.jsonl`**: All FSM state transitions (65 entries)
- **`test/results/decision_event_log.jsonl`**: All FSM events (64 entries)

### Output Directory

- **`/home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/`**: Organized manifests for UI developers

## Usage

### Option 1: Run Everything (Integration Test + Post-Processor)

```bash
cd /home/alono/EmorobCare/games_src/game_controller
./test/integration/run_and_organize.sh
```

This will:
1. Run the integration test (generates fresh `manifest_log.jsonl`)
2. Run the post-processor (organizes manifests)
3. Report completion with file locations

### Option 2: Just Re-Organize Existing Results

If you already have integration test results and just want to re-organize them:

```bash
cd /home/alono/EmorobCare/games_src/game_controller
python3 test/integration/organize_manifests_for_ui.py
```

## Output Structure

The post-processor creates this structure:

```
/home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/
├── README.md              # Overview and usage guide
├── index.md               # List of all states with links
├── 00_startup_menu/       # Initial menu state
│   ├── manifest.json      # Full UI manifest
│   ├── decision_state.json # FSM state and payload
│   └── description.md     # Human-readable summary
├── 01_run_a_phase_intro/  # First phase introduction
│   ├── manifest.json
│   ├── decision_state.json
│   └── description.md
├── 02_run_a_question_present_q1/  # First question
│   └── ...
... (57 total states)
```

## State Types Captured

The organized output includes all major game states:

1. **Menu States**: Initial idle/menu screens
2. **Phase Intros**: Introduction screens for each phase (P1, P2, P3, etc.)
3. **Question Present**: Questions being shown to the user
4. **Wait Input**: System waiting for user input (input enabled)
5. **Correct**: Positive feedback after correct answer
6. **Fail L1**: First-level failure feedback
7. **Paused**: Game paused state
8. **Phase Complete**: Phase completion screen
9. **Control Operations**: STOP, RESET, SKIP_PHASE demonstrations

## Integration Test Coverage

The integration test runs 3 complete sessions:

- **Run A (Pepe)**: P1, P2, P3 with PAUSE/RESUME and STOP on P3
- **Run B (María)**: P3, P4_YESNO, P6, P7 - full completion
- **Run C (Juan)**: P1, P2, P3 with SKIP_PHASE and RESET demonstrations

This provides comprehensive coverage of all phase types and control operations.

## Benefits of This Approach

1. **No QoS Mismatch Issues**: Uses the proven Docker integration test stack
2. **Real System Behavior**: Captures actual manifests from the working system
3. **Comprehensive Coverage**: Includes all phases, controls, and edge cases
4. **Easy to Browse**: Organized by state with human-readable descriptions
5. **Easy to Maintain**: Post-processor is simple Python code that just reorganizes existing data

## For UI Developers

UI developers can:

1. Browse states sequentially to understand game flow
2. Jump to specific states using the index
3. Read `description.md` for quick context
4. Examine `manifest.json` for exact UI configuration
5. Check `decision_state.json` to understand FSM context

Example workflow:
```bash
# View the index
cat /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/index.md

# Pick a state, e.g., question presentation
cd /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/02_run_a_question_present_q1/

# Read the description
cat description.md

# Examine the manifest
python3 -m json.tool manifest.json | less

# Check the FSM state
python3 -m json.tool decision_state.json
```

## Customization

The post-processor can be customized via environment variables:

```bash
# Use different input directory
TEST_RESULTS_DIR=/path/to/results python3 test/integration/organize_manifests_for_ui.py

# Use different output directory
OUTPUT_DIR=/path/to/output python3 test/integration/organize_manifests_for_ui.py
```

## Troubleshooting

### "ERROR: Manifest log not found"

The integration test hasn't been run yet. Run:
```bash
docker compose -f docker-compose.gc_dm_integration.yml up
```

### "No manifest entries found"

The `manifest_log.jsonl` file is empty or corrupted. Re-run the integration test.

### "Failed writing report"

Check directory permissions. The script needs write access to the output directory.

## See Also

- Integration test source: `test/integration/run_gc_dm_manifest_integration.py`
- Integration test docs: `GC_integration.md`
- UI integration docs: `UI_integration.md`
