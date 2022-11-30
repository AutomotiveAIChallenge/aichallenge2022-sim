cmake_minimum_required(VERSION 3.8)
project(aichallenge_score_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Score.msg"
  DEPENDENCIES std_msgs action_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
