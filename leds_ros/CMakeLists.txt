cmake_minimum_required(VERSION 3.8)
project(emorobcare_led_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLEDs.srv"
  "srv/PlayEffect.srv"
  "srv/ControlLEDs.srv"
  "srv/GetLEDState.srv"
)

install(PROGRAMS
  scripts/led_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
