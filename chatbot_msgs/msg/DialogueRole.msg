# The 'role', or purpose, of the dialogue.
# Chatbot backends (eg chatbot_rasa, chatbot_ollama) may expose one or several,
# depending on the models/capabilities they have.
# For instance, RASA might have a model for an hotel receptionist,
# named 'receptionist'
string name "__default__"

# special roles, wich may be available on PAL robots
string DEFAULT_ROLE = "__default__"
string ASK_ROLE = "__ask__"

# A opaque JSON string whose semantics are specific to each chatbot backend.
#
# For example, for chatbot_rasa:
# - DEFAULT_ROLE:
# {} # -> empty JSON, since no configuration is supported
#
# For chatbot_ollama:
# - DEFAULT_ROLE:
# {
#   "prompt":  "You are... Bla bla",  # -> main prompt to be sent to the model
#   "semantic_state_aggregator": {  # -> used to configure aggregator of semantic states updates to be injected in the dialogue (see semantic_state_msgs/ConfigureSemanticStateAggregator)
#     "filter_classes_env": ["Human"],
#     "task_state_enabled": True,
#     ...
#   },
# }
# - ASK_ROLE:
# {
#   "question":  "What is the age of ...?",  # -> question being uttered to the user, and added to the prompt
#   "result_schema_properties": {  # -> 
#     "age": {
#       "type": "integer",
#       "minimum": 0,
#   },
# }
string configuration "{}"
