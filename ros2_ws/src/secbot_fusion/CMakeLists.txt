cmake_minimum_required(VERSION 3.8)
project(secbot_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mcu_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(fusion_node 
	src/fusion_node.cpp
	src/range_fuser.cpp
	src/sim_sensor_mux.cpp
)

ament_target_dependencies(fusion_node
  rclcpp
  mcu_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS
	fusion_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	config
	DESTINATION share/${PROJECT_NAME}
)

target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_package()
