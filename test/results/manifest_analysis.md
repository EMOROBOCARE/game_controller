# Manifest + event analysis (headless GC↔DM integration)

## Counts
- manifest records: `57`
- decision state records: `72`
- decision event records: `71`
- unique manifest hashes: `38`

## Timeline (by step)

| step | system | gameState | component | mode | phase | answerType | items | pause | qText |
|---|---|---|---|---|---|---|---:|---|---|
| startup_menu | IDLE |  | GameSelector | menu |  | none | 0 | False |  |
| run_a_phase_intro | GAME | PHASE_INTRO | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_a_wait_input_recovered_q1 | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_a_wait_input_q1 | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_a_paused | PAUSED |  | GameComponent | game | P1 | match | 2 | True | Une los colores iguales. |
| run_a_resumed | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_a_fail_l1_phase_P1 | GAME | FAIL_L1 | GameComponent | game | P1 | match | 2 | False | Inténtalo de nuevo. |
| run_a_correct_phase_P1 | GAME | CORRECT | GameComponent | game | P1 | none | 0 | False | ¡Correcto! |
| run_a_question_present_q2 | GAME | QUESTION_PRESENT | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_a_wait_input_q2 | GAME | WAIT_INPUT | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_a_fail_l1_phase_P2 | GAME | FAIL_L1 | GameComponent | game | P2 | none | 0 | False | Inténtalo de nuevo. |
| run_a_correct_phase_P2 | GAME | CORRECT | GameComponent | game | P2 | none | 0 | False | Inténtalo de nuevo. |
| run_a_question_present_q3 | GAME | QUESTION_PRESENT | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color amari… |
| run_a_wait_input_q3 | GAME | WAIT_INPUT | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color amari… |
| run_a_menu_after_stop | IDLE |  | GameSelector | menu |  | none | 0 | False |  |
| run_b_phase_intro | GAME | PHASE_INTRO | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color azul |
| run_b_wait_input_recovered_q1 | GAME | WAIT_INPUT | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color azul |
| run_b_wait_input_q1 | GAME | WAIT_INPUT | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color azul |
| run_b_fail_l1_phase_P3 | GAME | FAIL_L1 | GameComponent | game | P3 | button | 2 | False | Inténtalo de nuevo. |
| run_b_correct_phase_P3 | GAME | CORRECT | GameComponent | game | P3 | none | 0 | False | ¡Perfecto! Es el color azul |
| run_b_question_present_q2 | GAME | QUESTION_PRESENT | GameComponent | game | P4 | button | 2 | False | Responde sí o no a las preguntas. ¿Es esto el c… |
| run_b_wait_input_q2 | GAME | WAIT_INPUT | GameComponent | game | P4 | button | 2 | False | Responde sí o no a las preguntas. ¿Es esto el c… |
| run_b_fail_l1_phase_P4 | GAME | FAIL_L1 | GameComponent | game | P4 | button | 2 | False | Inténtalo de nuevo. |
| run_b_correct_phase_P4 | GAME | CORRECT | GameComponent | game | P4 | none | 0 | False | ¡Genial! Es el color no |
| run_b_question_present_q3 | GAME | QUESTION_PRESENT | GameComponent | game | P4 | button | 2 | False | Responde sí o no a las preguntas. ¿Es esto el c… |
| run_b_wait_input_q3 | GAME | WAIT_INPUT | GameComponent | game | P4 | button | 2 | False | Responde sí o no a las preguntas. ¿Es esto el c… |
| run_b_correct_phase_P4 | GAME | CORRECT | GameComponent | game | P4 | none | 0 | False | ¡Muy bien! Es el color si |
| run_b_question_present_q4 | GAME | QUESTION_PRESENT | GameComponent | game | P5 | button | 2 | False | Dime qué color quieres señalar. ¿Dónde está el… |
| run_b_wait_input_q4 | GAME | WAIT_INPUT | GameComponent | game | P5 | button | 2 | False | Dime qué color quieres señalar. ¿Dónde está el… |
| run_b_fail_l1_phase_P5 | GAME | FAIL_L1 | GameComponent | game | P5 | button | 2 | False | Inténtalo de nuevo. |
| run_b_correct_phase_P5 | GAME | CORRECT | GameComponent | game | P6 | none | 0 | False | ¿Cuál es el color correcto? ¿Es un azul o verde? |
| run_b_question_present_q5 | GAME | QUESTION_PRESENT | GameComponent | game | P6 | none | 0 | False | ¿Cuál es el color correcto? ¿Es un azul o verde? |
| run_b_wait_input_q5 | GAME | WAIT_INPUT | GameComponent | game | P6 | none | 0 | False | ¿Cuál es el color correcto? ¿Es un azul o verde? |
| run_b_correct_phase_P6 | GAME | CORRECT | GameComponent | game | P6 | none | 0 | False | ¿Cuál es el color correcto? ¿Es un azul o verde? |
| run_b_phase_complete | GAME | PHASE_COMPLETE | GameComponent | game | P6 | none | 0 | False | ¡Fase P6 completada! |
| run_b_menu_after_complete | IDLE |  | GameSelector | menu |  | none | 0 | False |  |
| run_c_controls_phase_intro | GAME | PHASE_INTRO | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_c_controls_wait_input_recovered_q1 | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_c_controls_wait_input_q1 | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_c_controls_skip_intro_P2 | GAME | PHASE_INTRO | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_c_controls_wait_input_recovered_q2 | GAME | WAIT_INPUT | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_c_controls_wait_input_q2 | GAME | WAIT_INPUT | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_c_controls_reset_intro_P1 | GAME | PHASE_INTRO | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_c_controls_wait_input_recovered_q1 | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_c_controls_wait_input_q1 | GAME | WAIT_INPUT | GameComponent | game | P1 | match | 2 | False | Une los colores iguales. |
| run_c_controls_fail_l1_phase_P1 | GAME | FAIL_L1 | GameComponent | game | P1 | match | 2 | False | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P1 | GAME | CORRECT | GameComponent | game | P1 | none | 0 | False | ¡Correcto! |
| run_c_controls_question_present_q2 | GAME | QUESTION_PRESENT | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_c_controls_wait_input_q2 | GAME | WAIT_INPUT | GameComponent | game | P2 | none | 0 | False | Repite el nombre del color. ¿Qué color es? Es e… |
| run_c_controls_fail_l1_phase_P2 | GAME | FAIL_L1 | GameComponent | game | P2 | none | 0 | False | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P2 | GAME | CORRECT | GameComponent | game | P2 | none | 0 | False | Inténtalo de nuevo. |
| run_c_controls_question_present_q3 | GAME | QUESTION_PRESENT | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color rojo |
| run_c_controls_wait_input_q3 | GAME | WAIT_INPUT | GameComponent | game | P3 | button | 2 | False | Señala el color correcto. Señala el color rojo |
| run_c_controls_fail_l1_phase_P3 | GAME | FAIL_L1 | GameComponent | game | P3 | button | 2 | False | Inténtalo de nuevo. |
| run_c_controls_correct_phase_P3 | GAME | CORRECT | GameComponent | game | P3 | none | 0 | False | ¡Genial! Es el color rojo |
| run_c_controls_phase_complete | GAME | PHASE_COMPLETE | GameComponent | game | P3 | none | 0 | False | ¡Fase P3 completada! |
| run_c_controls_menu_after_complete | IDLE |  | GameSelector | menu |  | none | 0 | False |  |

## Control commands observed

| ts | command | resulting_state | resulting_gameState | resulting_phase |
|---:|---|---|---|---|
| 110624.665 | PAUSE | PAUSED | None | None |
| 110624.774 | RESUME | GAME | WAIT_INPUT | None |
| 110633.185 | STOP | IDLE | None | None |
| 110655.563 | SKIP_PHASE | GAME | PHASE_INTRO | P2 |
| 110661.617 | RESET | GAME | PHASE_INTRO | P1 |

## Anomalies

- None detected by this script

