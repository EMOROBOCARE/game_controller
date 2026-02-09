# Legacy ROS 2 integration (legacy/game_tasks + legacy/emorobcare_games)

Scope: interactions (topics, actions, services) referenced in the **legacy** code/assets under:
- `legacy/game_tasks/`
- `legacy/emorobcare_games/`

Note: The paths in this document are workspace-relative for this monorepo. If you are comparing against an external workspace where these packages live at the root, drop the `legacy/` prefix.

## emorobcare_games

### legacy/emorobcare_games/emo_games/mission_controller.py

**Topics**
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/emo_games/ongoing_activities` — `hri_actions_msgs/msg/ActivityList`
- Pub: `/ui/show` — `std_msgs/msg/String`

**Services**
- Server: `/emo_games/list_available_activities` — `hri_actions_msgs/srv/ListActivities`
- Client: `/idle_behavior` — `std_srvs/srv/SetBool`

**Actions**
- Client: `/say` — `audio_tts_msgs/action/TTS`
- Client: `/game1/control` — `my_game_interface/action/StartGame`
- Client: `/game2/control` — `my_game_interface/action/StartGame`
- Client: `/game3/control` — `my_game_interface/action/StartGame`
- Client: `/game4/control` — `my_game_interface/action/StartGame`
- Client: `/game5/control` — `my_game_interface/action/StartGame`
- Client: `/game6/control` — `my_game_interface/action/StartGame`
- Client: `/game7/control` — `my_game_interface/action/StartGame`

### legacy/emorobcare_games/pages/menu.js

**Topics**
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/volume` — `std_msgs/msg/Float32`
- Sub: `/ui/show` — `std_msgs/msg/String`

## game_tasks

### legacy/game_tasks/game1/game1/task_impl.py

**Topics**
- Pub: `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/ui/update` — `std_msgs/msg/String`

**Services**
- Client: `/chatbot/get_response` — `chatbot_msgs/srv/GetResponse`
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`
- Client: `/sim_scene/highlight_object` — `my_game_interface/srv/EditObject`

**Actions**
- Server: `/game1/control` — `my_game_interface/action/StartGame`
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

### legacy/game_tasks/game1/pages/script.js

**Topics**
- Pub: `/ui/input` — `std_msgs/msg/String`
- Sub: `/ui/update` — `std_msgs/msg/String`
- Sub: `/ui/show` — `std_msgs/msg/String`
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`

### legacy/game_tasks/game2/game2/task_impl.py

**Topics**
- Pub: `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/ui/update` — `std_msgs/msg/String`

**Services**
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`

**Actions**
- Server: `/game2/control` — `my_game_interface/action/StartGame`
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

### legacy/game_tasks/game2/pages/script.js

**Topics**
- Pub: `/ui/input` — `std_msgs/msg/String`
- Sub: `/ui/update` — `std_msgs/msg/String`
- Sub: `/ui/show` — `std_msgs/msg/String`
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`

### legacy/game_tasks/game3/game3/task_impl.py

**Topics**
- Pub: `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/robot_face/expression` — `hri_msgs/msg/Expression`
- Pub: `/ui/update` — `std_msgs/msg/String`

**Services**
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`

**Actions**
- Server: `/game3/control` — `my_game_interface/action/StartGame`
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

### legacy/game_tasks/game3/pages/script.js

**Topics**
- Pub: `/ui/input` — `std_msgs/msg/String`
- Sub: `/ui/update` — `std_msgs/msg/String`
- Sub: `/ui/show` — `std_msgs/msg/String`
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`

### legacy/game_tasks/game4/game4/task_impl.py

**Topics**
- Pub: `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Sub: `/ui/tracing` — `hri_actions_msgs/msg/Intent`
- Pub: `/ui/update` — `std_msgs/msg/String`

**Services**
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`

**Actions**
- Server: `/game4/control` — `my_game_interface/action/StartGame`
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

### legacy/game_tasks/game4/pages/script.js

**Topics**
- Pub: `/ui/input` — `std_msgs/msg/String`
- Pub: `/ui/tracing` — `std_msgs/msg/String`
- Sub: `/ui/update` — `std_msgs/msg/String`
- Sub: `/ui/show` — `std_msgs/msg/String`
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`

### legacy/game_tasks/game5/game5/task_impl.py

**Topics**
- Pub: `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/ui/update` — `std_msgs/msg/String`

**Services**
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`

**Actions**
- Server: `/game5/control` — `my_game_interface/action/StartGame`
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

### legacy/game_tasks/game5/pages/script.js

**Topics**
- Pub: `/ui/input` — `std_msgs/msg/String`
- Sub: `/ui/update` — `std_msgs/msg/String`
- Sub: `/ui/show` — `std_msgs/msg/String`
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`

### legacy/game_tasks/game7/game7/task_impl.py

**Topics**
- Pub: `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/robot_face/color` — `std_msgs/msg/ColorRGBA`
- Pub: `/ui/update` — `std_msgs/msg/String`

**Services**
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`
- Client: `/set_leds` — `emorobcare_led_service/srv/SetLEDs`

**Actions**
- Server: `/game7/control` — `my_game_interface/action/StartGame`
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

### legacy/game_tasks/game7/pages/script.js

**Topics**
- Pub: `/ui/input` — `std_msgs/msg/String`
- Sub: `/ui/update` — `std_msgs/msg/String`
- Sub: `/ui/show` — `std_msgs/msg/String`
- Pub: `/intents` — `hri_actions_msgs/msg/Intent`
