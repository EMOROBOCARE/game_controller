# Manifest + event analysis (headless GC↔DM integration)

## Counts
- manifest records: `57`
- decision state records: `65`
- decision event records: `64`
- unique manifest hashes: `57`

## Timeline (by step)

| step | system | gameState | mode | phase | qType | options | inputDisabled | controls | qText |
|---|---|---|---|---|---|---:|---|---|---|
| startup_menu | IDLE |  | menu |  |  | 0 | False | {"showPause": false, "showResume": false, "show… |  |
| run_a_phase_intro | GAME | PHASE_INTRO | game | P1 | phase_intro | 0 | True | {"showPause": true, "showResume": false, "showS… | Fase P1: Vamos a jugar a los colores. |
| run_a_question_present_q1 | GAME | QUESTION_PRESENT | game | P1 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_a_wait_input_q1 | GAME | WAIT_INPUT | game | P1 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_a_paused | PAUSED |  | game | P1 | multiple_choice | 2 | True | {"showPause": false, "showResume": true, "showS… | Elige la opción correcta. |
| run_a_resumed | GAME | WAIT_INPUT | game | P1 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_a_fail_l1_phase_P1 | GAME | FAIL_L1 | game | P1 | fail_l1 | 2 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_a_correct_phase_P1 | GAME | CORRECT | game | P1 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_a_question_present_q2 | GAME | QUESTION_PRESENT | game | P2 | speech | 0 | True | {"showPause": true, "showResume": false, "showS… | ¿Qué color es? Es el color green. |
| run_a_wait_input_q2 | GAME | WAIT_INPUT | game | P2 | speech | 0 | False | {"showPause": true, "showResume": false, "showS… | ¿Qué color es? Es el color green. |
| run_a_fail_l1_phase_P2 | GAME | FAIL_L1 | game | P2 | fail_l1 | 0 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_a_correct_phase_P2 | GAME | CORRECT | game | P2 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_a_question_present_q3 | GAME | QUESTION_PRESENT | game | P3 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | Señala el color green |
| run_a_wait_input_q3 | GAME | WAIT_INPUT | game | P3 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Señala el color green |
| run_a_menu_after_stop | IDLE |  | menu |  |  | 0 | False | {"showPause": false, "showResume": false, "show… |  |
| run_b_phase_intro | GAME | PHASE_INTRO | game | P3 | phase_intro | 0 | True | {"showPause": true, "showResume": false, "showS… | Fase P3: Vamos a jugar a los colores. |
| run_b_question_present_q1 | GAME | QUESTION_PRESENT | game | P3 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | Señala el color green |
| run_b_wait_input_q1 | GAME | WAIT_INPUT | game | P3 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Señala el color green |
| run_b_fail_l1_phase_P3 | GAME | FAIL_L1 | game | P3 | fail_l1 | 2 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_b_correct_phase_P3 | GAME | CORRECT | game | P3 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_b_question_present_q2 | GAME | QUESTION_PRESENT | game | P4_YESNO | yes_no | 2 | True | {"showPause": true, "showResume": false, "showS… | ¿Es esto el color red? |
| run_b_wait_input_q2 | GAME | WAIT_INPUT | game | P4_YESNO | yes_no | 2 | False | {"showPause": true, "showResume": false, "showS… | ¿Es esto el color red? |
| run_b_fail_l1_phase_P4_YESNO | GAME | FAIL_L1 | game | P4_YESNO | fail_l1 | 2 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_b_correct_phase_P4_YESNO | GAME | CORRECT | game | P4_YESNO | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_b_question_present_q3 | GAME | QUESTION_PRESENT | game | P4_YESNO | yes_no | 2 | True | {"showPause": true, "showResume": false, "showS… | ¿Es esto el color yellow? |
| run_b_wait_input_q3 | GAME | WAIT_INPUT | game | P4_YESNO | yes_no | 2 | False | {"showPause": true, "showResume": false, "showS… | ¿Es esto el color yellow? |
| run_b_correct_phase_P4_YESNO | GAME | CORRECT | game | P4_YESNO | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_b_question_present_q4 | GAME | QUESTION_PRESENT | game | P6 | pointing | 2 | True | {"showPause": true, "showResume": false, "showS… | ¿Dónde está el color? |
| run_b_wait_input_q4 | GAME | WAIT_INPUT | game | P6 | pointing | 2 | False | {"showPause": true, "showResume": false, "showS… | ¿Dónde está el color? |
| run_b_correct_phase_P6 | GAME | CORRECT | game | P6 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_b_question_present_q5 | GAME | QUESTION_PRESENT | game | P7 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | ¿Es un green o blue? |
| run_b_wait_input_q5 | GAME | WAIT_INPUT | game | P7 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | ¿Es un green o blue? |
| run_b_fail_l1_phase_P7 | GAME | FAIL_L1 | game | P7 | fail_l1 | 2 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_b_correct_phase_P7 | GAME | CORRECT | game | P7 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_b_phase_complete | GAME | PHASE_COMPLETE | game | P7 | phase_complete | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Fase P7 completada! |
| run_b_menu_after_complete | IDLE |  | menu |  |  | 0 | False | {"showPause": false, "showResume": false, "show… |  |
| run_c_controls_phase_intro | GAME | PHASE_INTRO | game | P1 | phase_intro | 0 | True | {"showPause": true, "showResume": false, "showS… | Fase P1: Vamos a jugar a los colores. |
| run_c_controls_question_present_q1 | GAME | QUESTION_PRESENT | game | P1 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_c_controls_wait_input_q1 | GAME | WAIT_INPUT | game | P1 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_c_controls_skip_intro_P2 | GAME | PHASE_INTRO | game | P2 | phase_intro | 0 | True | {"showPause": true, "showResume": false, "showS… | Fase P2: Vamos a jugar a los colores. |
| run_c_controls_question_present_q2 | GAME | QUESTION_PRESENT | game | P2 | speech | 0 | True | {"showPause": true, "showResume": false, "showS… | ¿Qué color es? Es el color yellow. |
| run_c_controls_wait_input_q2 | GAME | WAIT_INPUT | game | P2 | speech | 0 | False | {"showPause": true, "showResume": false, "showS… | ¿Qué color es? Es el color yellow. |
| run_c_controls_reset_intro_P1 | GAME | PHASE_INTRO | game | P1 | phase_intro | 0 | True | {"showPause": true, "showResume": false, "showS… | Fase P1: Vamos a jugar a los colores. |
| run_c_controls_question_present_q1 | GAME | QUESTION_PRESENT | game | P1 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_c_controls_wait_input_q1 | GAME | WAIT_INPUT | game | P1 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Elige la opción correcta. |
| run_c_controls_fail_l1_phase_P1 | GAME | FAIL_L1 | game | P1 | fail_l1 | 2 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P1 | GAME | CORRECT | game | P1 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_c_controls_question_present_q2 | GAME | QUESTION_PRESENT | game | P2 | speech | 0 | True | {"showPause": true, "showResume": false, "showS… | ¿Qué color es? Es el color yellow. |
| run_c_controls_wait_input_q2 | GAME | WAIT_INPUT | game | P2 | speech | 0 | False | {"showPause": true, "showResume": false, "showS… | ¿Qué color es? Es el color yellow. |
| run_c_controls_fail_l1_phase_P2 | GAME | FAIL_L1 | game | P2 | fail_l1 | 0 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P2 | GAME | CORRECT | game | P2 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_c_controls_question_present_q3 | GAME | QUESTION_PRESENT | game | P3 | multiple_choice | 2 | True | {"showPause": true, "showResume": false, "showS… | Señala el color red |
| run_c_controls_wait_input_q3 | GAME | WAIT_INPUT | game | P3 | multiple_choice | 2 | False | {"showPause": true, "showResume": false, "showS… | Señala el color red |
| run_c_controls_fail_l1_phase_P3 | GAME | FAIL_L1 | game | P3 | fail_l1 | 2 | False | {"showPause": true, "showResume": false, "showS… | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P3 | GAME | CORRECT | game | P3 | correct | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Muy bien! |
| run_c_controls_phase_complete | GAME | PHASE_COMPLETE | game | P3 | phase_complete | 0 | True | {"showPause": true, "showResume": false, "showS… | ¡Fase P3 completada! |
| run_c_controls_menu_after_complete | IDLE |  | menu |  |  | 0 | False | {"showPause": false, "showResume": false, "show… |  |

## Control commands observed

| ts | command | resulting_state | resulting_gameState | resulting_phase |
|---:|---|---|---|---|
| 3262.365 | PAUSE | PAUSED | None | None |
| 3262.478 | RESUME | GAME | WAIT_INPUT | None |
| 3268.417 | STOP | IDLE | None | None |
| 3283.496 | SKIP_PHASE | GAME | PHASE_INTRO | P2 |
| 3285.782 | RESET | GAME | PHASE_INTRO | P1 |

## Anomalies

- None detected by this script

