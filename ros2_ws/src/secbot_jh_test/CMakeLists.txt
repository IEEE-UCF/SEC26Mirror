cmake_minimum_required(VERSION 3.8)
project(secbot_jh_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)
#[==[
If you need files from another workspace like RAFEEDS PID CONTROLLER you can do the whole
set MCU_WS_DIR schmone but for now we are just testing the rclcpp and msg stuff, plus adding the executable for compiling!
]==]
set(MCU_SOURCES "")


add_executable(keystroke_publish_node #name of the excuetable for the launch pyton file
	src/keystroke_node.cpp
	${MCU_SOURCES}
)
# add_executable(keystroke_subscribe_node
# 	src/keystroke_listener.cpp
# 	${MCU_SOURCES}
# )
add_executable(keystroke_translator_node
	src/keystroke-translator-gazebo.cpp
	${MCU_SOURCES}
)
target_include_directories(keystroke_publish_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
# target_include_directories(keystroke_subscribe_node PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<INSTALL_INTERFACE:include>
# )
target_include_directories(keystroke_translator_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
# target_link_libraries()#???
ament_target_dependencies(keystroke_publish_node
	rclcpp
	geometry_msgs
	std_msgs
	sensor_msgs
	mcu_msgs
)
ament_target_dependencies(keystroke_translator_node
	rclcpp
	geometry_msgs
	std_msgs
	sensor_msgs
	mcu_msgs
)
#install worlds, launch , and the jh_testing(models) directory
install(DIRECTORY
	worlds
	launch
	Coke
	include
	src
	config
	# worlds/Proper-field
	# secbot_jh_test
	DESTINATION share/${PROJECT_NAME}
)
install (TARGETS
	keystroke_publish_node
	keystroke_translator_node
	# keystroke_subscribe_node
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()
#[==[
When you run colcon build, ROS2 creates an install/folder
	-ROS2 does NOT run code directly from your src folder. It runs it from the  install/(I don't know where this is located, all i know is that it needs to be run there)
	-install(DIRECTORY..): this command tells the build tool to copy your folders(worlds,launch,jh_testing) into the same area as the install/ folder(through something called a share directory)

]==]