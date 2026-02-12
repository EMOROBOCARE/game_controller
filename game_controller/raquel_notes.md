1) Standarise interaction modalities across phases: 
   1) matching -> dragging.    
   2) touch -> button
We will call this field "modality"
1) Switch to yaml config files for games so we support native comments and we explain each field with a brief comment.

2) hints is now going to be a list that can contain many of the following:
- highlight the correct answer
- say the correct answer (whispering or not)
- blinking.

1) MaximumFailures is now a configurable field that can be set to a value less than or equal to the number of hints available. This allows for more flexibility in game design and can help tailor the difficulty level to the player's needs. The default value will be the number of hints available, but it can be adjusted as needed and it will be in the .

Let the LLM fill the placeholders in the templates for prompts and everything instead of hardcodin articles inside the answers.

Remove P5 from code and rename P4_YESNO as P4. Also change P6 to P5 and so on...

P4 also depends on the difficulty level (Make this configurable):
1) No, Yes
2) Yes, No
3) Random order

----
prompt and text/phase instructions are now divided in:
1) text_instructions (shown in the UI)
2) verbal_instructions (used for TTS)

---
Remove failL1Action and failL2Action from phases
---
Remove "interactionType". Each phase will have a predefined interaction type that cannot be changed. This simplifies the configuration and reduces the chances of misconfiguration. For example, if a phase is designed for yes/no questions, it will always use the yes/no interaction type, and this will be determined by the phase itself rather than being configurable. This phase will inherit from te more general buttonAnswers ones imposing this restriction.