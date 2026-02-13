# game_controller ↔ decision_making integration report

- Success: `True`
- Duration: `60.2s`
- ROS_DOMAIN_ID: `43`

## Artifacts
- `/test_results/manifest_log.jsonl`
- `/test_results/decision_state_log.jsonl`
- `/test_results/decision_event_log.jsonl`
- `/test_results/topic_audit_log.jsonl`
- `/test_results/topic_action_stats.json`
- `/test_results/expressive_say_log.jsonl`

## Runs
- `run_a` requested=['P1', 'P2', 'P3'] seen=['P1', 'P2', 'P3'] correct=['P1', 'P2'] fail_once=['P1', 'P2'] exited_early=True
- `run_b` requested=['P3', 'P4', 'P5', 'P6'] seen=['P3', 'P4', 'P5', 'P6'] correct=['P3', 'P4', 'P5', 'P6'] fail_once=['P3', 'P4', 'P5'] exited_early=False
- `run_c_controls` requested=['P1', 'P2', 'P3'] seen=['P1', 'P2', 'P3'] correct=['P1', 'P2', 'P3'] fail_once=['P1', 'P2', 'P3'] exited_early=False

## Notes
- This test publishes UI events to `/ui/input` and bridges them to `/intents` inside the runner container (no browser).
- Topic audit includes pub/sub activity for `/game/*`, `/ui/input`, `/intents`, `/decision/*`, `/game/current_user`.
- Action audit validates `/expressive_say` goals through the mock action-server JSONL log.
- For UI work, see `UI_integration.md` and `GC_integration.md`.

## Step checklist
- `PASS` `startup_menu_manifest_ok`
- `PASS` `run_a_game_init_seen`
- `PASS` `run_a_phase_intro_manifest_ok`
- `PASS` `run_a_stopped_on_phase_P3`
- `PASS` `run_a_finished`
- `PASS` `run_b_game_init_seen`
- `PASS` `run_b_phase_intro_manifest_ok`
- `PASS` `run_b_phase_complete_seen`
- `PASS` `run_b_finished`
- `PASS` `run_c_controls_game_init_seen`
- `PASS` `run_c_controls_phase_intro_manifest_ok`
- `PASS` `run_c_controls_skipped_to_P2`
- `PASS` `run_c_controls_reset_to_P1`
- `PASS` `run_c_controls_phase_complete_seen`
- `PASS` `run_c_controls_finished`
- `PASS` `topic_action_observability_ok`

