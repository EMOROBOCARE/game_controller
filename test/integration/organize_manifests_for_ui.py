#!/usr/bin/env python3
"""Post-processor for integration test results.

Reads manifest_log.jsonl and decision_state_log.jsonl from the integration test
and organizes them into a UI-developer-friendly directory structure.

Output structure:
    /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/
    ├── README.md
    ├── index.md
    ├── 00_initial_menu/
    │   ├── manifest.json (full manifest state)
    │   ├── decision_state.json (FSM context)
    │   └── description.md (human-readable)
    ├── 01_P1_phase_intro/
    ├── 02_P1_question_1_present/
    ... etc
"""

from __future__ import annotations

import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

@dataclass
class LogEntry:
    """A single entry from manifest_log.jsonl."""
    ts: float
    step: str
    decision: Optional[Dict[str, Any]]
    manifest_hash: str
    manifest: Dict[str, Any]


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    """Load a JSONL file into a list of dictionaries."""
    entries = []
    if not path.exists():
        return entries

    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return entries


def parse_manifest_log(entries: List[Dict[str, Any]]) -> List[LogEntry]:
    """Parse manifest log entries into LogEntry objects."""
    log_entries = []
    for entry in entries:
        log_entries.append(LogEntry(
            ts=float(entry.get('ts', 0)),
            step=str(entry.get('step', '')),
            decision=entry.get('decision'),
            manifest_hash=str(entry.get('manifest_hash', '')),
            manifest=entry.get('manifest', {})
        ))
    return log_entries


def _as_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_list(value: Any) -> List[Any]:
    return value if isinstance(value, list) else []


def generate_description(entry: LogEntry, index: int) -> str:
    """Generate a human-readable description for a manifest state."""
    decision = entry.decision or {}
    state = decision.get('state', 'UNKNOWN')
    game_state = decision.get('gameState')
    transaction_id = decision.get('transactionId', 0)
    payload = decision.get('payload', {})

    lines = [
        f"# Step {index:02d}: {entry.step}",
        "",
        "## Overview",
        f"- **Timestamp**: {entry.ts:.2f}s",
        f"- **FSM State**: {state}",
        f"- **Game State**: {game_state or 'N/A'}",
        f"- **Transaction ID**: {transaction_id}",
        f"- **Manifest Hash**: {entry.manifest_hash}",
        "",
    ]

    # Add specific details based on state
    if game_state == "PHASE_INTRO":
        phase = payload.get('phase', '')
        intro = payload.get('introduction', '')
        lines.extend([
            "## Phase Introduction",
            f"- **Phase**: {phase}",
            f"- **Introduction**: {intro}",
            "",
        ])

    elif game_state == "QUESTION_PRESENT":
        question = payload.get('question', {})
        q_id = question.get('questionId', 0)
        prompt = question.get('prompt', '')
        q_type = question.get('questionType', '')
        answer = question.get('answer', '')
        options = question.get('options', [])

        lines.extend([
            "## Question",
            f"- **Question ID**: {q_id}",
            f"- **Type**: {q_type}",
            f"- **Prompt**: {prompt}",
            f"- **Answer**: {answer}",
            "",
        ])

        if options:
            lines.append("### Options")
            for opt in options:
                opt_id = opt.get('id', '')
                label = opt.get('label', '')
                correct = opt.get('correct', False)
                marker = "✓" if correct else "✗"
                lines.append(f"- [{marker}] {opt_id}: {label}")
            lines.append("")

    elif game_state == "WAIT_INPUT":
        round_id = payload.get('roundId', 0)
        question_id = payload.get('questionId', 0)
        lines.extend([
            "## Waiting for Input",
            f"- **Round ID**: {round_id}",
            f"- **Question ID**: {question_id}",
            "",
        ])

    elif game_state == "FAIL_L1":
        hint = payload.get('hint', '')
        lines.extend([
            "## Failure - Level 1",
            f"- **Hint**: {hint}",
            "",
        ])

    elif game_state == "CORRECT":
        lines.extend([
            "## Correct Answer",
            "User answered correctly.",
            "",
        ])

    elif game_state == "PHASE_COMPLETE":
        phase = payload.get('phase', '')
        lines.extend([
            "## Phase Complete",
            f"- **Phase**: {phase}",
            "",
        ])

    elif state == "PAUSED":
        lines.extend([
            "## Game Paused",
            "The game is currently paused.",
            "",
        ])

    elif state == "IDLE":
        lines.extend([
            "## Idle State",
            "System is idle, showing the menu.",
            "",
        ])

    # Add manifest instance info
    instances = entry.manifest.get('instances', [])
    if instances:
        lines.append("## UI Components")
        for inst in instances:
            inst_id = inst.get('id', '')
            component = inst.get('component', '')
            config = inst.get('config', {})

            lines.append(f"### {inst_id} ({component})")
            if not isinstance(config, dict):
                lines.append("")
                continue

            if component == "GameSelector":
                games = config.get("games") if isinstance(config.get("games"), list) else []
                lines.append(f"- **Games**: {len(games)}")
                lines.append(f"- **Start Op**: {config.get('startGameOpId', '')}")
                username = config.get("username")
                if username:
                    lines.append(f"- **Username**: {username}")

            elif component == "GameComponent":
                lines.append(f"- **Answer Type**: {config.get('answerType', '')}")
                lines.append(f"- **Pause Panel**: {config.get('pause', False)}")
                question = config.get('question', {})
                if isinstance(question, dict):
                    q_text = question.get('text', '')
                    q_id = question.get('id', 0)
                    if q_id:
                        lines.append(f"- **Question ID**: {q_id}")
                    if q_text:
                        lines.append(f"- **Question Text**: {q_text}")
                items = config.get("items") if isinstance(config.get("items"), list) else []
                lines.append(f"- **Items Count**: {len(items)}")
            else:
                mode = config.get('mode', '')
                if mode:
                    lines.append(f"- **Mode**: {mode}")

            lines.append("")

    return "\n".join(lines)


def generate_index(entries: List[LogEntry], output_dir: Path) -> None:
    """Generate an index.md file listing all states."""
    lines = [
        "# Colores Full Game - Manifest States Index",
        "",
        "This directory contains all manifest states captured during the integration test.",
        "",
        "## States",
        "",
    ]

    for i, entry in enumerate(entries):
        step_dir = f"{i:02d}_{entry.step}"
        decision = entry.decision or {}
        state = decision.get('state', 'UNKNOWN')
        game_state = decision.get('gameState', '')

        state_desc = f"{state}"
        if game_state:
            state_desc += f" / {game_state}"

        lines.append(f"{i:02d}. [{entry.step}](./{step_dir}/) - {state_desc}")

    lines.append("")

    index_path = output_dir / "index.md"
    with open(index_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))

    print(f"Generated index: {index_path}")


def generate_readme(output_dir: Path, total_states: int) -> None:
    """Generate a README.md file for the output directory."""
    lines = [
        "# Colores Full Game - UI Developer Manifests",
        "",
        "This directory contains all UI manifest states from a complete run of the Colores game,",
        "captured during integration testing with the decision_making FSM.",
        "",
        "## Contents",
        "",
        f"- **Total States**: {total_states}",
        "- **Source**: Integration test (`docker-compose.gc_dm_integration.yml`)",
        "- **Manifest Format**: direct snapshots as published by `game_controller` + `generic_ui` (no post-adaptation)",
        "- **Game**: Colores",
        "- **Test Runs**: Multiple sessions with different phases (P1-P7)",
        "",
        "## Directory Structure",
        "",
        "Each numbered directory represents a single state in the game progression:",
        "",
        "```",
        "NN_state_name/",
        "├── manifest.json        # Full UI manifest for this state",
        "├── decision_state.json  # FSM state and payload",
        "└── description.md       # Human-readable description",
        "```",
        "",
        "## How to Use",
        "",
        "1. Browse the states in order by number (00, 01, 02, ...)",
        "2. Read `description.md` for a quick overview of each state",
        "3. Examine `manifest.json` to see the exact UI configuration",
        "4. Check `decision_state.json` to understand the FSM context",
        "",
        "## Key States to Review",
        "",
        "- **00_initial_menu**: Initial menu screen",
        "- **Phase intros**: Look for states with `PHASE_INTRO` in the name",
        "- **Questions**: Look for `question_present` in the name",
        "- **User input**: Look for `wait_input` in the name",
        "- **Feedback**: Look for `correct` or `fail_l1` in the name",
        "",
        "## See Also",
        "",
        "- [index.md](./index.md) - Complete list of all states",
        "- Integration test source: `game_controller/test/integration/run_gc_dm_manifest_integration.py`",
        "- Test runner: `game_controller/test/integration/organize_manifests_for_ui.py` (this script)",
        "",
    ]

    readme_path = output_dir / "README.md"
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))

    print(f"Generated README: {readme_path}")


def organize_manifests(
    manifest_log_path: Path,
    output_dir: Path,
) -> None:
    """Main function to organize manifests into UI-friendly structure."""

    # Load manifest log
    print(f"Loading manifest log from {manifest_log_path}")
    raw_entries = load_jsonl(manifest_log_path)
    print(f"Loaded {len(raw_entries)} manifest entries")

    if not raw_entries:
        print("ERROR: No manifest entries found!")
        return

    # Parse entries
    entries = parse_manifest_log(raw_entries)

    # Create output directory
    print(f"Creating output directory: {output_dir}")
    if output_dir.exists():
        # Clean up existing directory
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Process each entry
    for i, entry in enumerate(entries):
        step_dir = output_dir / f"{i:02d}_{entry.step}"
        step_dir.mkdir(exist_ok=True)

        # Write manifest.json
        manifest_path = step_dir / "manifest.json"
        with open(manifest_path, 'w', encoding='utf-8') as f:
            json.dump(entry.manifest, f, indent=2, ensure_ascii=False)

        # Write decision_state.json
        if entry.decision:
            decision_path = step_dir / "decision_state.json"
            with open(decision_path, 'w', encoding='utf-8') as f:
                json.dump(entry.decision, f, indent=2, ensure_ascii=False)

        # Write description.md
        description = generate_description(entry, i)
        description_path = step_dir / "description.md"
        with open(description_path, 'w', encoding='utf-8') as f:
            f.write(description)

        print(f"  [{i:02d}/{len(entries):02d}] {entry.step}")

    # Generate index and README
    generate_index(entries, output_dir)
    generate_readme(output_dir, len(entries))

    print(f"\nSuccess! Organized {len(entries)} manifest states into {output_dir}")
    print(f"\nNext steps:")
    print(f"  1. Review the README: {output_dir}/README.md")
    print(f"  2. Browse the index: {output_dir}/index.md")
    print(f"  3. Explore individual states in the numbered directories")


def main() -> None:
    """Main entry point."""
    # Default paths
    test_results_dir = Path("/home/alono/EmorobCare/games_src/game_controller/test/results")
    manifest_log_path = test_results_dir / "manifest_log.jsonl"
    output_dir = Path("/home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game")

    # Allow override via environment variables
    if "TEST_RESULTS_DIR" in os.environ:
        test_results_dir = Path(os.environ["TEST_RESULTS_DIR"])
        manifest_log_path = test_results_dir / "manifest_log.jsonl"

    if "OUTPUT_DIR" in os.environ:
        output_dir = Path(os.environ["OUTPUT_DIR"])

    # Check if manifest log exists
    if not manifest_log_path.exists():
        print(f"ERROR: Manifest log not found at {manifest_log_path}")
        print("Please run the integration test first:")
        print("  docker compose -f docker-compose.gc_dm_integration.yml up")
        return

    # Run the organizer
    organize_manifests(manifest_log_path, output_dir)


if __name__ == "__main__":
    main()
