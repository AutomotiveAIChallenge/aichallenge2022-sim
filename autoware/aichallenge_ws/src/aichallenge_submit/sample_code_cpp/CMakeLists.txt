cmake_minimum_required(VERSION 3.8)
project(sample_code_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_system_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(sample src/sample.cpp)
ament_target_dependencies(sample rclcpp std_msgs geometry_msgs autoware_auto_system_msgs autoware_auto_vehicle_msgs autoware_auto_perception_msgs)


install(TARGETS
  sample
  DESTINATION lib/${PROJECT_NAME})

ament_package()
