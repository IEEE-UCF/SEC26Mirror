cmake_minimum_required(VERSION 3.8)
project(secbot_health)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(secbot_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Health node executable
add_executable(health_node
  src/health_node.cpp
)

ament_target_dependencies(health_node
  rclcpp
  std_msgs
  mcu_msgs
  nav_msgs
  secbot_msgs
  diagnostic_msgs
)

# Install targets
install(TARGETS
  health_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
