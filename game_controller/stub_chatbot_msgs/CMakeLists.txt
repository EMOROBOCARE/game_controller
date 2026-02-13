cmake_minimum_required(VERSION 3.8)
project(chatbot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hri_actions_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EvaluateAnswer.srv"
  "srv/GetResponse.srv"
  "srv/KeywordAction.srv"
  "srv/Rephrase.srv"
  "srv/ResetModel.srv"
  DEPENDENCIES hri_actions_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
