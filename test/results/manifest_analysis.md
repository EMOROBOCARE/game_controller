# Manifest + event analysis (headless GC↔DM integration)

## Counts
- manifest records: `57`
- decision state records: `66`
- decision event records: `65`
- unique manifest hashes: `57`

## Timeline (by step)

| step | system | gameState | component | mode | phase | answerType | items | pause | qText |
|---|---|---|---|---|---|---|---:|---|---|
| startup_menu | IDLE |  | GameSelector | menu |  | none | 0 | False |  |
| run_a_phase_intro | GAME | PHASE_INTRO | GameSelector | menu | P1 | none | 0 | False | Fase P1: Vamos a jugar a los colores. |
| run_a_question_present_q1 | GAME | QUESTION_PRESENT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_a_wait_input_q1 | GAME | WAIT_INPUT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_a_paused | PAUSED |  | GameSelector | menu | P1 | button | 2 | True | Une los colores iguales. |
| run_a_resumed | GAME | WAIT_INPUT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_a_fail_l1_phase_P1 | GAME | FAIL_L1 | GameSelector | menu | P1 | button | 2 | False | Inténtalo de nuevo. |
| run_a_correct_phase_P1 | GAME | CORRECT | GameSelector | menu | P1 | none | 0 | False | ¡Muy bien! |
| run_a_question_present_q2 | GAME | QUESTION_PRESENT | GameSelector | menu | P2 | button | 2 | False | ¿Qué color es? Es el color green. |
| run_a_wait_input_q2 | GAME | WAIT_INPUT | GameSelector | menu | P2 | button | 2 | False | ¿Qué color es? Es el color green. |
| run_a_fail_l1_phase_P2 | GAME | FAIL_L1 | GameSelector | menu | P2 | button | 2 | False | Inténtalo de nuevo. |
| run_a_correct_phase_P2 | GAME | CORRECT | GameSelector | menu | P2 | none | 0 | False | ¡Muy bien! |
| run_a_question_present_q3 | GAME | QUESTION_PRESENT | GameSelector | menu | P3 | button | 2 | False | Señala el color green |
| run_a_wait_input_q3 | GAME | WAIT_INPUT | GameSelector | menu | P3 | button | 2 | False | Señala el color green |
| run_a_menu_after_stop | IDLE |  | GameSelector | menu |  | none | 0 | False |  |
| run_b_phase_intro | GAME | PHASE_INTRO | GameSelector | menu | P3 | none | 0 | False | Fase P3: Vamos a jugar a los colores. |
| run_b_question_present_q1 | GAME | QUESTION_PRESENT | GameSelector | menu | P3 | button | 2 | False | Señala el color green |
| run_b_wait_input_q1 | GAME | WAIT_INPUT | GameSelector | menu | P3 | button | 2 | False | Señala el color green |
| run_b_fail_l1_phase_P3 | GAME | FAIL_L1 | GameSelector | menu | P3 | button | 2 | False | Inténtalo de nuevo. |
| run_b_correct_phase_P3 | GAME | CORRECT | GameSelector | menu | P3 | none | 0 | False | ¡Muy bien! |
| run_b_question_present_q2 | GAME | QUESTION_PRESENT | GameSelector | menu | P4 | button | 2 | False | ¿Es esto el color yellow? |
| run_b_wait_input_q2 | GAME | WAIT_INPUT | GameSelector | menu | P4 | button | 2 | False | ¿Es esto el color yellow? |
| run_b_fail_l1_phase_P4 | GAME | FAIL_L1 | GameSelector | menu | P4 | button | 2 | False | Inténtalo de nuevo. |
| run_b_correct_phase_P4 | GAME | CORRECT | GameSelector | menu | P4 | none | 0 | False | ¡Muy bien! |
| run_b_question_present_q3 | GAME | QUESTION_PRESENT | GameSelector | menu | P4 | button | 2 | False | ¿Es esto el color green? |
| run_b_wait_input_q3 | GAME | WAIT_INPUT | GameSelector | menu | P4 | button | 2 | False | ¿Es esto el color green? |
| run_b_correct_phase_P4 | GAME | CORRECT | GameSelector | menu | P4 | none | 0 | False | ¡Muy bien! |
| run_b_question_present_q4 | GAME | QUESTION_PRESENT | GameSelector | menu | P5 | button | 2 | False | ¿Dónde está el color? |
| run_b_wait_input_q4 | GAME | WAIT_INPUT | GameSelector | menu | P5 | button | 2 | False | ¿Dónde está el color? |
| run_b_fail_l1_phase_P5 | GAME | FAIL_L1 | GameSelector | menu | P5 | button | 2 | False | Inténtalo de nuevo. |
| run_b_correct_phase_P5 | GAME | CORRECT | GameSelector | menu | P5 | none | 0 | False | ¡Muy bien! |
| run_b_question_present_q5 | GAME | QUESTION_PRESENT | GameSelector | menu | P6 | button | 2 | False | ¿Es un red o blue? |
| run_b_wait_input_q5 | GAME | WAIT_INPUT | GameSelector | menu | P6 | button | 2 | False | ¿Es un red o blue? |
| run_b_correct_phase_P6 | GAME | CORRECT | GameSelector | menu | P6 | none | 0 | False | ¡Muy bien! |
| run_b_phase_complete | GAME | PHASE_COMPLETE | GameSelector | menu | P6 | none | 0 | False | ¡Fase P6 completada! |
| run_b_menu_after_complete | IDLE |  | GameSelector | menu |  | none | 0 | False |  |
| run_c_controls_phase_intro | GAME | PHASE_INTRO | GameSelector | menu | P1 | none | 0 | False | Fase P1: Vamos a jugar a los colores. |
| run_c_controls_question_present_q1 | GAME | QUESTION_PRESENT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_c_controls_wait_input_q1 | GAME | WAIT_INPUT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_c_controls_skip_intro_P2 | GAME | PHASE_INTRO | GameSelector | menu | P2 | none | 0 | False | Fase P2: Vamos a jugar a los colores. |
| run_c_controls_question_present_q2 | GAME | QUESTION_PRESENT | GameSelector | menu | P2 | button | 2 | False | ¿Qué color es? Es el color red. |
| run_c_controls_wait_input_q2 | GAME | WAIT_INPUT | GameSelector | menu | P2 | button | 2 | False | ¿Qué color es? Es el color red. |
| run_c_controls_reset_intro_P1 | GAME | PHASE_INTRO | GameSelector | menu | P1 | none | 0 | False | Fase P1: Vamos a jugar a los colores. |
| run_c_controls_question_present_q1 | GAME | QUESTION_PRESENT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_c_controls_wait_input_q1 | GAME | WAIT_INPUT | GameSelector | menu | P1 | button | 2 | False | Une los colores iguales. |
| run_c_controls_fail_l1_phase_P1 | GAME | FAIL_L1 | GameSelector | menu | P1 | button | 2 | False | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P1 | GAME | CORRECT | GameSelector | menu | P1 | none | 0 | False | ¡Muy bien! |
| run_c_controls_question_present_q2 | GAME | QUESTION_PRESENT | GameSelector | menu | P2 | button | 2 | False | ¿Qué color es? Es el color red. |
| run_c_controls_wait_input_q2 | GAME | WAIT_INPUT | GameSelector | menu | P2 | button | 2 | False | ¿Qué color es? Es el color red. |
| run_c_controls_fail_l1_phase_P2 | GAME | FAIL_L1 | GameSelector | menu | P2 | button | 2 | False | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P2 | GAME | CORRECT | GameSelector | menu | P2 | none | 0 | False | ¡Muy bien! |
| run_c_controls_question_present_q3 | GAME | QUESTION_PRESENT | GameSelector | menu | P3 | button | 2 | False | Señala el color blue |
| run_c_controls_wait_input_q3 | GAME | WAIT_INPUT | GameSelector | menu | P3 | button | 2 | False | Señala el color blue |
| run_c_controls_fail_l1_phase_P3 | GAME | FAIL_L1 | GameSelector | menu | P3 | button | 2 | False | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P3 | GAME | CORRECT | GameSelector | menu | P3 | none | 0 | False | ¡Muy bien! |
| run_c_controls_phase_complete | GAME | PHASE_COMPLETE | GameSelector | menu | P3 | none | 0 | False | ¡Fase P3 completada! |
| run_c_controls_menu_after_complete | IDLE |  | GameSelector | menu |  | none | 0 | False |  |

## Control commands observed

| ts | command | resulting_state | resulting_gameState | resulting_phase |
|---:|---|---|---|---|
| 15865.175 | PAUSE | PAUSED | None | None |
| 15865.280 | RESUME | GAME | WAIT_INPUT | None |
| 15871.132 | STOP | IDLE | None | None |
| 15871.234 | STOP | GAME | PHASE_INTRO | P3 |
| 15886.272 | SKIP_PHASE | GAME | PHASE_INTRO | P2 |
| 15888.537 | RESET | GAME | PHASE_INTRO | P1 |

## Anomalies

- `run_a_phase_intro`: GAME should render GameComponent
- `run_a_question_present_q1`: GAME should render GameComponent
- `run_a_wait_input_q1`: GAME should render GameComponent
- `run_a_paused`: PAUSED should render GameComponent
- `run_a_resumed`: GAME should render GameComponent
- `run_a_fail_l1_phase_P1`: GAME should render GameComponent
- `run_a_correct_phase_P1`: GAME should render GameComponent
- `run_a_question_present_q2`: GAME should render GameComponent
- `run_a_wait_input_q2`: GAME should render GameComponent
- `run_a_fail_l1_phase_P2`: GAME should render GameComponent
- `run_a_correct_phase_P2`: GAME should render GameComponent
- `run_a_question_present_q3`: GAME should render GameComponent
- `run_a_wait_input_q3`: GAME should render GameComponent
- `run_b_phase_intro`: GAME should render GameComponent
- `run_b_question_present_q1`: GAME should render GameComponent
- `run_b_wait_input_q1`: GAME should render GameComponent
- `run_b_fail_l1_phase_P3`: GAME should render GameComponent
- `run_b_correct_phase_P3`: GAME should render GameComponent
- `run_b_question_present_q2`: GAME should render GameComponent
- `run_b_wait_input_q2`: GAME should render GameComponent
- `run_b_fail_l1_phase_P4`: GAME should render GameComponent
- `run_b_correct_phase_P4`: GAME should render GameComponent
- `run_b_question_present_q3`: GAME should render GameComponent
- `run_b_wait_input_q3`: GAME should render GameComponent
- `run_b_correct_phase_P4`: GAME should render GameComponent
- `run_b_question_present_q4`: GAME should render GameComponent
- `run_b_wait_input_q4`: GAME should render GameComponent
- `run_b_fail_l1_phase_P5`: GAME should render GameComponent
- `run_b_correct_phase_P5`: GAME should render GameComponent
- `run_b_question_present_q5`: GAME should render GameComponent
- `run_b_wait_input_q5`: GAME should render GameComponent
- `run_b_correct_phase_P6`: GAME should render GameComponent
- `run_b_phase_complete`: GAME should render GameComponent
- `run_c_controls_phase_intro`: GAME should render GameComponent
- `run_c_controls_question_present_q1`: GAME should render GameComponent
- `run_c_controls_wait_input_q1`: GAME should render GameComponent
- `run_c_controls_skip_intro_P2`: GAME should render GameComponent
- `run_c_controls_question_present_q2`: GAME should render GameComponent
- `run_c_controls_wait_input_q2`: GAME should render GameComponent
- `run_c_controls_reset_intro_P1`: GAME should render GameComponent
- `run_c_controls_question_present_q1`: GAME should render GameComponent
- `run_c_controls_wait_input_q1`: GAME should render GameComponent
- `run_c_controls_fail_l1_phase_P1`: GAME should render GameComponent
- `run_c_controls_correct_phase_P1`: GAME should render GameComponent
- `run_c_controls_question_present_q2`: GAME should render GameComponent
- `run_c_controls_wait_input_q2`: GAME should render GameComponent
- `run_c_controls_fail_l1_phase_P2`: GAME should render GameComponent
- `run_c_controls_correct_phase_P2`: GAME should render GameComponent
- `run_c_controls_question_present_q3`: GAME should render GameComponent
- `run_c_controls_wait_input_q3`: GAME should render GameComponent
- (truncated: 53 anomalies)

