# Codex manifest deep analysis (headless GC↔DM↔generic_ui)

- Generated: 2026-02-03 18:38 UTC
- Inputs: manifest_log.jsonl, decision_state_log.jsonl, decision_event_log.jsonl, report.json

## Executive summary
- Contract invariants (A1–A6): 6/6 PASS
- Records: manifest=57, decision_state=65, decision_event=64, unique_manifest_hashes=46
- GAME_CONTROL coverage: 5/5 commands observed (PAUSE/RESUME/STOP/RESET/SKIP_PHASE)
- USER_INTENT: 18 events (correct=10, incorrect=8) with 0 tx/state mismatches
- ON_COMPLETE: 38 events; observed transitions match expected mapping for PHASE_INTRO/QUESTION_PRESENT/FAIL_L1/PHASE_COMPLETE
- Timing note: QUESTION_PRESENT is transient in these logs (median 0.102s); some manifest snapshots labeled QUESTION_PRESENT already show WAIT_INPUT

## Test runs (from report.json)
- run_a: phases_requested=['P1', 'P2', 'P3'] phases_seen=['P1', 'P2', 'P3'] exited_early=True
- run_b: phases_requested=['P3', 'P4_YESNO', 'P6', 'P7'] phases_seen=['P3', 'P4_YESNO', 'P6', 'P7'] exited_early=False
- run_c_controls: phases_requested=['P1', 'P2', 'P3'] phases_seen=['P1', 'P2', 'P3'] exited_early=False

## A) Contract invariant validation
| id | result | notes |
|---|---|---|
| A1 | PASS | bad_instances=0, bad_layout_refs=0 |
| A2 | PASS | violations=0 |
| A3 | PASS | menu_snapshots=4, violations=0 |
| A4 | PASS | game_snapshots=53, violations=0 |
| A5 | PASS | checked_snapshots=33, violations=0 |
| A6 | PASS | violations=0 |

### Coherence observations (non-fatal)
- Manifest snapshots where step decision tx != manifest config.state tx: 12/57
- In all snapshots, mode/system/session/phase are internally coherent (menu⇔IDLE, game⇔GAME/PAUSED).

## B) Behavior correlation & control semantics
### B1) Timeline correlation (high-level)
- decision_state transactionId is monotonic from 1→65 across all runs; events reference these ids consistently.
- In the headless harness, WAIT_INPUT often lasts only a few milliseconds (automated answers), so real UI dwell times will be longer.

### B2) GAME_CONTROL commands
| command | expected next state | observed next state | manifest after (mode/system, inputDisabled, controls) | result |
|---|---|---|---|---|
| PAUSE | PAUSED | PAUSED | game/PAUSED inputDisabled=True pause=False,resume=True,stop=True,reset=True,skip=True | PASS |
| RESUME | GAME/WAIT_INPUT | GAME/WAIT_INPUT | game/GAME inputDisabled=False pause=True,resume=False,stop=True,reset=True,skip=True | PASS |
| STOP | IDLE | IDLE | menu/IDLE inputDisabled=False pause=False,resume=False,stop=False,reset=False,skip=False | PASS |
| SKIP_PHASE | GAME/PHASE_INTRO | GAME/PHASE_INTRO | game/GAME inputDisabled=True pause=True,resume=False,stop=True,reset=True,skip=True | PASS |
| RESET | GAME/PHASE_INTRO | GAME/PHASE_INTRO | game/GAME inputDisabled=True pause=True,resume=False,stop=True,reset=True,skip=True | PASS |

### B3) USER_INTENT events
- tx correlation: 0 mismatches (each USER_INTENT tx matched a WAIT_INPUT state; tx+1 transitioned to CORRECT/FAIL_L1 as expected).
- coverage: correct=true (10), correct=false (8)

### B4) ON_COMPLETE events
- All ON_COMPLETE events referenced an existing prior transactionId, and advanced the FSM on tx+1.
- Observed transition patterns (count):
  - GAME/QUESTION_PRESENT → GAME/WAIT_INPUT: 13
  - GAME/CORRECT → GAME/QUESTION_PRESENT: 8
  - GAME/FAIL_L1 → GAME/WAIT_INPUT: 8
  - GAME/PHASE_INTRO → GAME/QUESTION_PRESENT: 5
  - GAME/CORRECT → GAME/PHASE_COMPLETE: 2
  - GAME/PHASE_COMPLETE → IDLE/None: 2

## C) Manifest coherence & UI readiness
### Fields the UI must treat carefully
- input gating: inputDisabled should be interpreted as "answer input disabled" (controls may still need to work, especially in PAUSED).
- transient states: QUESTION_PRESENT is very brief in these logs; UI should primarily render off question/options + inputDisabled, not rely on seeing every gameState.
- assets: question/option images are IDs/relative paths; UI needs a defined resolver/base URL.

### UI readiness checklist (minimum)
- Load remote modules from componentRegistry (scope/module/url) for UserPanel and the active game component (`GameSelector` or `GameComponent`).
- Render by mode: menu uses games[] + startGameOpId; game uses question/options/phase/state + uiInputOpId.
- Emit ROS messages via ops as {data: JSON.stringify(payload)} (std_msgs/String) for selections and ui_input.
- Respect controls.show* to show/hide Pause/Resume/Stop/Reset/SkipPhase buttons.
- Respect inputDisabled for answer widgets; still allow session controls as specified by contract.
- Support phase codes seen in logs: P1, P2, P3, P4_YESNO, P6, P7 (plus fallbacks).

## Anomalies & recommendations
- [INFO] Startup manifest uses config.state.transactionId=0 while first decision/state log starts at 1 (placeholder vs published tx). ({'step': 'startup_menu', 'ts': 34233.405764976})
  - Suggested fix: Clarify in contract that manifest initial state may use tx=0, or set it to first observed decision transactionId for consistency.
- [INFO] In PAUSED, manifest config.state.gameState retains the last GAME gameState (e.g., WAIT_INPUT) while /decision/state.gameState is null. ({'step': 'run_a_paused', 'ts': 34235.585174523, 'transactionId': 5})
  - Suggested fix: Either keep and document this as "lastGameState", or set config.state.gameState=null when system!=GAME to match decision_state schema.
- [WARN] Manifest snapshots taken at "*_question_present_*" steps often already reflect the subsequent WAIT_INPUT state (tx+1). This suggests QUESTION_PRESENT is transient (~0.10s median in these logs), so a polling UI could miss it. ({'count': 11})
  - Suggested fix: Ensure UI does not depend on seeing QUESTION_PRESENT separately (use question text + inputDisabled). If needed, add a stable UI-facing substate (e.g., uiPhase="speaking"|"awaiting_input") or enforce a minimum dwell time.
- [WARN] During PAUSED, inputDisabled=true while Resume/Stop/Reset/Skip controls are visible. A UI that globally disables inputs based on inputDisabled might also disable Resume. ({'step': 'run_a_paused', 'ts': 34235.585174523, 'transactionId': 5})
  - Suggested fix: Clarify contract: inputDisabled should gate answer inputs only, or split into answersDisabled vs controlsDisabled.
- [WARN] Game/menu images use relative paths/IDs (e.g., assets/colores.png, blue_circle). Without a defined base URL/CDN strategy, a real browser UI cannot resolve these reliably. ({'scope': 'manifest.games[].image and question/option img fields'})
  - Suggested fix: Standardize an asset base URL (env/config) and have game_controller expand/normalize image URLs before publishing manifests.
- [INFO] UI integration relies on JSON encoded in std_msgs/String, which is brittle (validation, tooling, backward-compat parsing). ({'scope': 'ops: /game/user_selector /game/game_selector /ui/input'})
  - Suggested fix: Add typed services/messages (start_session, control_session, typed selections) as recommended in GC_integration.md to reduce ambiguity and add synchronous acknowledgments.

## GC/DM integration improvement suggestions
- Add typed services for start_session and control_session (ack + validation), instead of JSON-on-String topics for critical flows.
- If the UI must react differently to "speaking" vs "awaiting input", add a stable UI-facing substate field in manifest (don’t infer from transient DM gameState).
- Clarify/rename inputDisabled semantics (answers vs controls) to prevent UI dead-ends in PAUSED.
- Standardize asset URL strategy (controller-side URL expansion) so browser UI can render images reliably.
