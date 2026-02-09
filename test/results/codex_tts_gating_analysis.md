# Codex TTS gating analysis (headless GC↔DM↔UI)

## Scope
- Requirement: `QUESTION_PRESENT` must not advance to `WAIT_INPUT` until `game_controller` emits `ON_COMPLETE` (transactionId == QUESTION_PRESENT.tx) after the expressive TTS action finishes.
- Sources: existing headless integration artifacts (no docker reruns).

## Inputs
- `game_controller/test/results/decision_state_log.jsonl`
- `game_controller/test/results/decision_event_log.jsonl`
- `game_controller/test/results/manifest_log.jsonl`
- `game_controller/test/results/report.json`

## Record counts
- manifest records: `57`
- decision state records: `65`
- decision event records: `64`
- QUESTION_PRESENT states: `13`

## 1) Contract invariants (manifest)
- ✅ No violations detected:
  - `instances` always exactly `{game_screen, user_panel}`
  - `controls` schema always present
  - `mode` consistent with system state (`IDLE`→`menu`, non-`IDLE`→`game`)
  - `question.text` non-empty when `gameState ∈ {QUESTION_PRESENT, WAIT_INPUT}`

## 2) TTS gating evidence (decision logs)
- ON_COMPLETE deltas (ON_COMPLETE.ts - QUESTION_PRESENT.ts): min/median/max = `0.155` / `0.156` / `0.160` sec
- Missing ON_COMPLETE for QUESTION_PRESENT: `0`
- Next state is WAIT_INPUT (tx+1): `13` / `13`

### Per-QUESTION_PRESENT checks
| tx | phase | qType | state_ts | on_complete_ts | Δ sec | next_tx | next_state |
|---:|---|---|---:|---:|---:|---:|---|
| 3 | P1 | multiple_choice | 37113.718 | 37113.878 | 0.160 | 4 | WAIT_INPUT |
| 10 | P2 | speech | 37116.811 | 37116.967 | 0.156 | 11 | WAIT_INPUT |
| 15 | P3 | multiple_choice | 37119.590 | 37119.747 | 0.157 | 16 | WAIT_INPUT |
| 19 | P3 | multiple_choice | 37121.963 | 37122.119 | 0.155 | 20 | WAIT_INPUT |
| 24 | P4_YESNO | yes_no | 37124.835 | 37124.991 | 0.156 | 25 | WAIT_INPUT |
| 29 | P4_YESNO | yes_no | 37127.712 | 37127.869 | 0.157 | 30 | WAIT_INPUT |
| 32 | P6 | pointing | 37128.579 | 37128.735 | 0.155 | 33 | WAIT_INPUT |
| 35 | P7 | multiple_choice | 37129.346 | 37129.502 | 0.155 | 36 | WAIT_INPUT |
| 43 | P1 | multiple_choice | 37134.543 | 37134.699 | 0.157 | 44 | WAIT_INPUT |
| 46 | P2 | speech | 37136.712 | 37136.869 | 0.156 | 47 | WAIT_INPUT |
| 49 | P1 | multiple_choice | 37138.881 | 37139.037 | 0.156 | 50 | WAIT_INPUT |
| 54 | P2 | speech | 37141.661 | 37141.817 | 0.156 | 55 | WAIT_INPUT |
| 59 | P3 | multiple_choice | 37144.534 | 37144.690 | 0.156 | 60 | WAIT_INPUT |

## 3) Manifest correlation (QUESTION_PRESENT ↔ WAIT_INPUT)
- ✅ For every QUESTION_PRESENT tx=T:
  - manifest `inputDisabled==true` in QUESTION_PRESENT
  - manifest `inputDisabled==false` in WAIT_INPUT (system==GAME)
  - manifest `question.text` unchanged between QUESTION_PRESENT and WAIT_INPUT for the same question

## Outliers
- None
