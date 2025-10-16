cmake_minimum_required(VERSION 3.8)
project(adam_message)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create action directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/action)

# Create ExecuteTask.action if it doesn't exist
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/action/ExecuteTask.action)
  file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/action/ExecuteTask.action
"# Goal
string step
string value
string station
---
# Result  
bool success
string message
float64 duration
---
# Feedback
int32 progress_percent
string current_status
float64 elapsed_time
")
endif()

# Generate action messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTask.action"
  DEPENDENCIES action_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
