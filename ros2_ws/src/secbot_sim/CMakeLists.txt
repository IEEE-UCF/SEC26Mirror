cmake_minimum_required(VERSION 3.8)
project(secbot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ensure M_PI is available
add_compile_definitions(_USE_MATH_DEFINES)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)

# MCU workspace pure-C++ sources (PID, motion profiles, trajectory, localization, math)
set(MCU_WS_DIR "/home/ubuntu/mcu_workspaces/sec26mcu" CACHE PATH
	"MCU workspace root inside Docker")
if(NOT EXISTS "${MCU_WS_DIR}/lib/control")
	# Repo-relative path (host builds without Docker)
	set(MCU_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu_ws")
endif()
if(NOT EXISTS "${MCU_WS_DIR}/lib/control")
	# CI seeded path (ros2_src volume: src/sec26ros/_mcu_lib/)
	set(MCU_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../_mcu_lib")
	set(MCU_LIB_DIR "${MCU_WS_DIR}")
endif()

# Use lib/ subdirectory unless CI seeded flat structure
if(NOT DEFINED MCU_LIB_DIR)
	set(MCU_LIB_DIR "${MCU_WS_DIR}/lib")
endif()

message(STATUS "MCU sources: ${MCU_LIB_DIR}")

# Include directories
include_directories(
	include
	${MCU_LIB_DIR}/control
	${MCU_LIB_DIR}/math
	${MCU_LIB_DIR}/drive
)

# MCU pure-C++ sources to compile (no Arduino deps)
set(MCU_SOURCES
	${MCU_LIB_DIR}/control/motion_profile.cpp
	${MCU_LIB_DIR}/control/traj_controller.cpp
	${MCU_LIB_DIR}/math/Pose2D.cpp
	${MCU_LIB_DIR}/math/Vector2D.cpp
	${MCU_LIB_DIR}/drive/TankDriveLocalization.cpp
)

# Add executable
add_executable(mcu_subsystem_sim
	src/mcu_subsystem_sim.cpp
	${MCU_SOURCES}
)
ament_target_dependencies(mcu_subsystem_sim
	rclcpp
	std_msgs
	geometry_msgs
	sensor_msgs
	mcu_msgs
)

# Install executable
install(TARGETS
	mcu_subsystem_sim
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
