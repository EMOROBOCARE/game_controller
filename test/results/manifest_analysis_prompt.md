# Codex prompt: manifest + event log analysis (headless integration)

Use this prompt with Codex CLI (GPT‑5.2, `model_reasoning_effort="xhigh"`) to analyze the already-generated logs **without re-running Docker**.

## Command

From repo root (`/home/alono/EmorobCare/games_src`):

```bash
cat <<'PROMPT' | codex exec --full-auto --skip-git-repo-check -C /home/alono/EmorobCare/games_src -m gpt-5.2 -c model_reasoning_effort="xhigh" -
<paste the PROMPT body below>
PROMPT
```

## PROMPT body

You are a Codex CLI agent. Goal: analyze the headless **game_controller ↔ decision_making ↔ generic_ui** integration logs and produce a readable report (without dumping raw JSON).

Inputs (already generated; do NOT rerun docker):
- `game_controller/test/results/manifest_log.jsonl`
- `game_controller/test/results/decision_state_log.jsonl`
- `game_controller/test/results/decision_event_log.jsonl`
- `game_controller/test/results/report.json`
Context docs:
- `INTEGRATION_CONTRACT.md`
- `UI_integration.md`
- `GC_integration.md`

Constraints:
- Do NOT rerun Docker or regenerate logs.
- Do NOT modify production code.
- You MAY create/overwrite analysis files under `game_controller/test/results/`.
- Avoid printing full manifests/events; summarize and reference `step` names + key fields.

Contract invariants to validate (high priority):
1) Manifest always includes `instances` for `user_panel` (UserPanel) + `game_screen` (currently `GameSelector`/`GameComponent` in this runtime) and layout references only existing instances.
2) `game_screen.config.controls` always exists with boolean keys:
   - `showPause`, `showResume`, `showStop`, `showReset`, `showSkipPhase`
3) Menu state: `game_screen.config.mode == "menu"`, `inputDisabled == false`, and all controls are hidden.
4) Game state: `game_screen.config.mode == "game"`, `controls.showStop == true`, `controls.showReset == true`, `controls.showSkipPhase == true`.
5) No empty prompts in gameplay: when `game_screen.config.mode=="game"` and `question.questionType` indicates a question/intro (`phase_intro`, `multiple_choice`, `speech`, `yes_no`, etc), `question.text` must be non-empty.
6) Event correctness:
   - Every `USER_INTENT` event includes `transactionId`, `value`, `modality`, and boolean `correct`.
   - `ON_COMPLETE` includes `transactionId` and matches a previous decision state transaction.
   - `GAME_CONTROL` commands are among: `PAUSE`, `RESUME`, `STOP`, `RESET`, `SKIP_PHASE` (aliases may appear as `EXIT`, `RESTART`, `SKIP`).

Tasks (careful + systematic):
1) Parse `manifest_log.jsonl` (JSON per line). Count records. Verify ordering by `ts`.
2) Build a step timeline table:
   - `step`, `decision.state`, `decision.gameState`, `manifest_hash`,
   - `game_screen.config` summary: `mode`, `state.system/gameState/tx`, `phase`, `questionType`, `question.text` (truncated), `options.len`, `controls`, `inputDisabled`.
3) Group by `manifest_hash` and explain what changed between successive hashes (layout vs instance config vs ops/registry). Keep concise.
4) Validate schema/types and registry coverage:
   - manifest keys `{version, componentRegistry, ops, layout, instances, ui}`
   - every `instance.component` exists in `componentRegistry`
   - ops include `game_selector`, `user_selector`, `ui_input`
5) Correlate decision state ↔ manifest:
   - for each decision state transition in `decision_state_log.jsonl`, verify the manifest reflects it within a small delay (use the nearest later manifest snapshot).
   - verify `game_screen.config.phase` stays consistent with the phase implied by rounds/questions.
6) Correlate decision events ↔ behavior:
   - Find `GAME_CONTROL` events and confirm the corresponding state transitions exist:
     - `PAUSE` → `state=PAUSED` and `controls.showResume=true` and `inputDisabled=true`
     - `RESUME` → back to `state=GAME` and `controls.showPause=true`
     - `STOP`/`EXIT` → `state=IDLE` and menu manifest
     - `SKIP_PHASE`/`SKIP` → a new `PHASE_INTRO` with the *next* phase and manifest phase updates
     - `RESET`/`RESTART` → `PHASE_INTRO` back to the first phase
   - For several `USER_INTENT` events (at least 3), verify:
     - `transactionId` matches the current decision state tx at that time
     - `correct==false` leads to `FAIL_L1` (at least once per phase where applicable)
     - `correct==true` leads to `CORRECT`
7) Produce actionable findings:
   - Any anomalies/risks for a real UI (missing fields, unstable values, timing concerns).
   - Any mismatches with `INTEGRATION_CONTRACT.md`.

Outputs:
- Overwrite `game_controller/test/results/manifest_analysis.md` (<= 200 lines).
- Overwrite `game_controller/test/results/manifest_analysis.json` with:
  - `total_records_manifest`, `total_records_state`, `total_records_events`
  - `unique_manifest_hashes`
  - `steps[]`: `{ts, step, systemState, gameState, hash, mode, phase, questionType, optionsCount, inputDisabled, controls}`
  - `controls_events[]`: `{ts, command, resulting_state, resulting_gameState, resulting_phase}`
  - `anomalies[]`: `{where, reason}`
- In your final message: print only the output file paths + an executive summary (<= 10 bullets). No raw dumps.
