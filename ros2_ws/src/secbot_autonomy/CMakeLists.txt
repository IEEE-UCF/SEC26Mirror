cmake_minimum_required(VERSION 3.8)
project(secbot_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(secbot_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Task library (all task state machines)
add_library(secbot_tasks SHARED
  src/antenna_align.cpp
  src/button_press.cpp
  src/crank_turn.cpp
  src/crater_entry.cpp
  src/flag_plant.cpp
  src/keypad_enter.cpp
  src/pressure_clear.cpp
)

ament_target_dependencies(secbot_tasks
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  secbot_msgs
  mcu_msgs
)

# Autonomy node executable
add_executable(autonomy_node
  src/autonomy_node.cpp
)

target_link_libraries(autonomy_node
  secbot_tasks
)

ament_target_dependencies(autonomy_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  secbot_msgs
  mcu_msgs
)

# Mission node executable
add_executable(mission_node
  src/mission_node.cpp
)

ament_target_dependencies(mission_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  secbot_msgs
  mcu_msgs
)

# Install targets
install(TARGETS
  secbot_tasks
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  autonomy_node
  mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(secbot_tasks)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  secbot_msgs
  mcu_msgs
)

ament_package()
