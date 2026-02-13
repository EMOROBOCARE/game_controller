# State Machine Integration

## Overview
`decision_making` is the FSM authority. `game_controller` does not decide transitions; it publishes events and reacts to `/decision/state`.

## Active Game States
Current game-level states emitted by `decision_making`:
- `PHASE_INTRO`
- `QUESTION_PRESENT`
- `WAIT_INPUT`
- `FAIL_L1`
- `FAIL_L2`
- `CORRECT`
- `PHASE_COMPLETE`

System state wrapper includes `IDLE`, `GAME`, `PAUSED`, `BATTERY_LOW`, `SHUTDOWN`.

## Core Transition Flow

```text
IDLE
  -- GAME_INIT --> GAME/PHASE_INTRO

PHASE_INTRO
  -- ON_COMPLETE --> QUESTION_PRESENT

QUESTION_PRESENT
  -- ON_COMPLETE --> WAIT_INPUT

WAIT_INPUT
  -- USER_INTENT(correct) --> CORRECT
  -- USER_INTENT(incorrect #1) --> FAIL_L1
  -- USER_INTENT(incorrect #2+) --> FAIL_L2

FAIL_L1
  -- ON_COMPLETE --> WAIT_INPUT

CORRECT or FAIL_L2
  -- ON_COMPLETE --> QUESTION_PRESENT (next round) OR PHASE_COMPLETE

PHASE_COMPLETE
  -- ON_COMPLETE --> IDLE (session end in current controller flow)
```

## Event Contracts
All events are JSON in `std_msgs/String.data` on `/decision/events`.

### `GAME_INIT`
Starts session with generated payload:
- `slug`, `title`, `introduction`
- `difficulty`
- `phaseSequence`, `phaseConfigs`
- `rounds[]`

### `ON_COMPLETE`
```json
{ "type": "ON_COMPLETE", "payload": { "transactionId": 12 } }
```

### `USER_INTENT`
```json
{
  "type": "USER_INTENT",
  "payload": {
    "transactionId": 14,
    "value": "blue",
    "modality": "touch",
    "correct": true
  }
}
```

### `GAME_CONTROL`
```json
{ "type": "GAME_CONTROL", "payload": { "command": "PAUSE" } }
```

Accepted command set:
- `PAUSE`, `RESUME`
- `STOP` (`EXIT`, `BACK` aliases upstream)
- `RESET` (`RESTART` alias)
- `SKIP_PHASE` (`SKIP` alias)

## Transaction Guarding
- Every `/decision/state` carries a `transactionId`.
- `ON_COMPLETE` and `USER_INTENT` must use the active `transactionId`.
- Stale events are ignored by `decision_making`.

## Auto-Advance in `game_controller`
Timer configuration is read from `auto_advance.*` parameters.

Special case:
- `PHASE_INTRO` is auto-skipped by default (`auto_advance.phase_intro = 0.0`).
- In `QUESTION_PRESENT`, controller may gate `ON_COMPLETE` on `/expressive_say` completion when TTS is enabled and the prompt is present for speech.
- Prompt playback is controlled per question via `question.say_prompt`; when explicitly `false`, prompt speech is skipped and timer-based advance is used.
- In `CORRECT`, controller gates `ON_COMPLETE` to wait for positive feedback speech (generated from `positive_feedback`) when available.
- `CORRECT` transitions are additionally delayed until at least `auto_advance.correct_min_display` seconds have elapsed from speech start.
- If the spoken-gate path is not active, `CORRECT` falls back to timer-based `ON_COMPLETE`.

- In `WAIT_INPUT` for `P5`, if the child does not answer within `hint_timeout` (from game content, default 5s), `game_controller` speaks a hint prompt derived from `expected_question`; repeated hints for the same child in a phase become shorter ("¿Donde está el?").

## Input Gating
`game_controller` only forwards answers when current game state is:
- `WAIT_INPUT`

Speech inputs without an explicit `correct` field are semantically evaluated via `/chatbot/evaluate_answer` before publishing `USER_INTENT`.

## Service Use During Flow
- `QUESTION_PRESENT` / `CORRECT`: prompt/feedback may be rephrased via `/chatbot/rephrase` (when enabled).
- `WAIT_INPUT` (speech modality): semantic correctness may be resolved via `/chatbot/evaluate_answer`.
- Correct answers trigger UI `effect="confetti"` and continue on `ON_COMPLETE`.

Control commands are accepted independently of answer gating.
