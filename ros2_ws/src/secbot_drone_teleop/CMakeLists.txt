cmake_minimum_required(VERSION 3.8)
project(secbot_drone_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)

add_executable(drone_teleop_node
  src/drone_teleop_node.cpp
)

ament_target_dependencies(drone_teleop_node
  rclcpp
  geometry_msgs
  mcu_msgs
)

install(TARGETS
  drone_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/drone_pid_tuner.py
  scripts/gui_for_pid.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
