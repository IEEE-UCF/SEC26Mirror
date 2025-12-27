cmake_minimum_required(VERSION 3.8)
project(secbot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(mcu_subsystem_sim src/mcu_subsystem_sim.cpp)
ament_target_dependencies(mcu_subsystem_sim
	rclcpp
	std_msgs
	geometry_msgs
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
