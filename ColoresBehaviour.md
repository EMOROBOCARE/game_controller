# New refactor:
## ros2:

### Topics:
- Sub: `/intents` — `hri_actions_msgs/msg/Intent`
- Pub: `/volume` — `std_msgs/msg/Float32`

## Services:
- Client: `/chatbot/rephrase` — `chatbot_msgs/srv/Rephrase`
- Client: `/chatbot/evaluate_answer` — `chatbot_msgs/srv/EvaluateAnswer`


### Actions:
- Client: `/expressive_say` — `audio_tts_msgs/action/Communication`

## General:
1) No introduction, when a phase loads the game controller uses `/expressive_say` to read verbal_instructions and the UI shows text_instructions. This is placed as the question of the phase. The option specific question can be appended to it.
2) We will subscribe to `/intents` in all phases, and the game controller will decide what to do with the intents based on the current phase. This is to avoid having to change the topics every time we change the phase. More info in `communication_hub` folder
3) Volume slider publish to `/volume` topic.
4) You should call /chatbot/rephrase and /chatbot/evaluate_answer in the appropriate phases, but the game controller will decide when to call them based on the current phase. Check out how answers where checked in legacy code.

## Phase 1:
Example of a manifest, highlight differences in what you are sending and how could we adapt this.

```json
{
        "version": 1,
        "componentRegistry": {
            "Button": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./Button"
            },
            "GameSelector": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./GameSelector"
            },
            "Question": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./Question"
            },
            "UserPanel": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./UserPanel"
            },
            "MatchingPhase": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./MatchingPhase"
            },
            "ButtonAnswer": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./ButtonAnswer"
            },
            "GameComponent": {
                "url": f"{cdn_base_url}/emorobcare-components/assets/remoteEntry.js",
                "scope": "demo",
                "module": "./GameComponent"
            }
        },
        "ops": {
            # Topics published by UI components
            "game_selector": {
                "kind": "topic_pub",
                "rosType": "std_msgs/msg/String",
                "topic": "/emorobcare/game_selected"
            },
            "user_selector": {
                "kind": "topic_pub",
                "rosType": "std_msgs/msg/String",
                "topic": "/emorobcare/user_selected"
            },
            "ui_input": {
                "kind": "topic_pub",
                "rosType": "std_msgs/msg/String",
                "topic": "/emorobcare/ui_input"
            },
            # Topics subscribed by UI components
            "current_user": {
                "kind": "topic_sub",
                "rosType": "std_msgs/msg/Int16",
                "topic": "/emorobcare/current_user"
            },
            "current_user_name": {
                "kind": "topic_sub",
                "rosType": "std_msgs/msg/String",
                "topic": "/emorobcare/current_user_name"
            }
        },
        "layout": {
            "mode": "list",
            "items": [
                {"instanceId": "user_panel"},
                {"instanceId": "game_component_2"}
      

            ]
        },
        "ui": {
            "chrome": "kiosk",
            "background": "#a4c0e0"
        },
        "instances": [
            {
                "id": "game_component_2",
                "component": "GameComponent",
                "config": {
                    "effect": "none",
                    "question": {
                    "id": 1,
                    "text": "Une los que sean iguales"
                   
                    },
                    "answerOpId": "game_answer",
                    "answerType": "match",
                    "items": [
                    { "label": "Tomate", "img": f"{image_base}/child_pan.png","highlighted": True },
                    { "label": "Elefante", "img": f"{image_base}/gusanitos.png" },
                    { "label": "Colores", "img": f"{image_base}/pelota.png" , "disabled": True},
                    { "label": "Colores", "img": f"{image_base}/pelota.png" , "disabled": True}
                    ],
                    "pause": False,
                    "gameFlowOpId": "game_flow",
                    "volumeOpId": "game_volume"
                 }
            },
            {
                "id": "question",
                "component": "Question",
                "config": {
                    "text": "¿Qué ves en la imagen?",
                    "image": f"{image_base}/colores.png" 
                }
            },
             {
                "id": "matching_phase",
                "component": "MatchingPhase",
                "config": {
                    "items": [
                    { "id": "1", "image": f"{image_base}/tomato.png" },
                    { "id": "2", "image": f"{image_base}/elefante.png" },
                    { "id": "3", "image": f"{image_base}/colores.png" }
                    ],
                    "matchingOpId": "matching_ui_1",
                    "matchingResultOpId": "matching_result_1"
                },
                "bindings": {
                    "matching_ui_1": "matching_ui_1"
                },
                "capabilities": ["matching_ui_1"]
            },
            {
                "id": "user_panel",
                "component": "UserPanel",
                "config": {
                    "users": DEFAULT_USERS,
                    "activeUser": DEFAULT_USERS[0],  # Pepe as default
                    "userPanelOpId": "user_selector",
                    "disabled": True
                },
                "bindings": {
                    "user_selector": "user_selector"
                },
                "capabilities": ["user_selector"]
            }
        }

        ]
    }
```
## Phase 2:
There is no answer component here, only a question with an image:

{
                "id": "game_component_2",
                "component": "GameComponent",
                "config": {
                    "effect": "none",
                    "question": {
                    "id": 1,
                    "text": "¿Qué color es?",
                   "img": f"{image_base}/colores.png"
                    },
                    "answerOpId": "game_answer",
                    "answerType": "none", 
                    options:[]
                    "pause": False,
                    "gameFlowOpId": "game_flow",
                    "volumeOpId": "game_volume"
                 }
            },


## Phase 3:
Now the answers are buttons

{
                "id": "game_component_2",
                "component": "GameComponent",
                "config": {
                    "effect": "none",
                    "question": {
                    "id": 1,
                    "text": "¿Qué color es?",
                   "img": f"{image_base}/colores.png"
                    },
                    "answerOpId": "game_answer",
                    "answerType": "button", 
                    options:[
                    { "label": "Tomate", "img": f"{image_base}/child_pan.png","highlighted": True },
                    { "label": "Elefante", "img": f"{image_base}/gusanitos.png" },
                    { "label": "Colores", "img": f"{image_base}/pelota.png" , "disabled": True}
                    ]
                    "pause": False,
                    "gameFlowOpId": "game_flow",
                    "volumeOpId": "game_volume"
                 }
            },

## Phase 4:
Yes/No.
You have to send the game component wit buttons but the only two buttons are always "Si" and "No". The button component has a parameter to specify the labels of the buttons and their background.

## Phase 5: 
All buttons are disabled: the child ask "Dónde está el rojo?" and the screen highlight the correct answer and says: "Aquiiii". We will ask the LLM if what the child said is equivalent to any of our label (a new service might be required so define it precisely)

## Phase 6:
Now the question is of the kind: "Este color es rojo o azul?" depending on the difficulty the answer will be positioned in one place 8read the configs yaml.
The child must use speech.
{
                "id": "game_component_2",
                "component": "GameComponent",
                "config": {
                    "effect": "none",
                    "question": {
                    "id": 1,
                    "text": "¿Es el color rojo o el color azul?",
                   "img": f"{image_base}/red.png"
                    },
                    "answerOpId": "game_answer",
                    "answerType": "none", 
                    options:[
                    ]
                    "pause": False,
                    "gameFlowOpId": "game_flow",
                    "volumeOpId": "game_volume"
                 }
            },

