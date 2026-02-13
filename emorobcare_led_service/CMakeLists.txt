cmake_minimum_required(VERSION 3.8)
project(emorobcare_led_service)

# Set CMake policy to avoid target naming conflicts
cmake_policy(SET CMP0002 NEW)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLEDs.srv"
  "srv/SetLEDsString.srv"
  "srv/PlayEffect.srv"
  "srv/GetLEDState.srv"
  "srv/ControlLEDs.srv"
  "msg/AvailableColors.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install Python modules manually
install(DIRECTORY emorobcare_led_service
  DESTINATION lib/python3.10/site-packages
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  emorobcare_led_service/led_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME led_service_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install examples
install(DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
