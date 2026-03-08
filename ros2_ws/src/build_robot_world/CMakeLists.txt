cmake_minimum_required(VERSION 3.8)
project(build_robot_world)

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
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# Install launch files
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  controllers
  DESTINATION share/${PROJECT_NAME}/
)

# Install Gazebo resources (worlds, models, sdf)
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)


install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
