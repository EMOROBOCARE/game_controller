^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package chatbot_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2025-04-30)
------------------
* update doc for semantic state aggregator
* Removed messages from context manager
* update context manager interfaces
* rework DialoguePurpose into DialogueRole
* rename AddToDialogue to DialogueInteraction
* added service to set the context, as well as modified the one to get the context to provide a list
* remove GetResponse and ResetModel
* rework Dialogue purpose and documentation
* Rename service to get context in the cmakelists
* renamings in the service to get context
* add assistant user to AddToDialogue
* rename GetInitialPrompt to GetSystemPrompt; add the dialogue ID to the latter request
* Added the new service getInitialPrompt to CMakeLists
* Added service to get initial prompt
* use UUID as dialogue identifier
* fix dialogue result
* Contributors: Luka Juricic, ferrangebelli

2.2.0 (2025-02-17)
------------------
* add Dialogue.action and AdvanceDialogue.srv
* Contributors: Luka Juricic

2.1.0 (2024-07-01)
------------------
* rename fields in GetResponse
* remove legacy i18n_msgs dependency
* refine documentation
* reset only one model at a time
* remove Dialogue.action and Status.msg
* Contributors: Luka Juricic

2.0.0 (2024-05-07)
------------------
* update interfaces for new chatbot architecture
* port to humble
* Contributors: Luka Juricic

0.1.0 (2023-01-17)
------------------
* Merge branch 'master' into 'main'
  change names and add srvs
  See merge request interaction/chatbot_msgs!2
* change names and add srvs
* add action to start training RASA
* add langugage changer
* Contributors: Sara Cooper, saracooper

0.0.21 (2022-02-21)
-------------------
* CHANGELOG
* Merge branch 'master' into 'main'
  chatbot msgs
  See merge request interaction/chatbot_msgs!1
* chatbot msgs
* Initial commit
* Contributors: davidfernandez, saracooper
