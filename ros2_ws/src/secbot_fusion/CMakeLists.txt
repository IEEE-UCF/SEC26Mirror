cmake_minimum_required(VERSION 3.8)
project(secbot_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)
# find_package(robot_localization REQUIRED)
# Add these only if you include them in code:
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(fusion_node 
	src/fusion_node.cpp 
	src/odom_fuser.cpp 
	src/range_fuser.cpp 
	src/sim_sensor_mux.cpp 	
	src/tf_broadcaster.cpp
)

ament_target_dependencies(fusion_node
  rclcpp
  mcu_msgs
  nav_msgs
)

target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_package()
