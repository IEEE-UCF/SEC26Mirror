cmake_minimum_required(VERSION 3.8)
project(secbot_uwb)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Beacon config library
add_library(beacon_config
  src/beacon_config.cpp
)
ament_target_dependencies(beacon_config
  rclcpp
)

# Positioning node library
add_library(positioning_node_lib
  src/positioning_node.cpp
)
ament_target_dependencies(positioning_node_lib
  rclcpp
  geometry_msgs
  std_msgs
  mcu_msgs
)
target_link_libraries(positioning_node_lib
  beacon_config
  Eigen3::Eigen
)

# Positioning node executable
add_executable(positioning_node
  src/positioning_node_main.cpp
)
ament_target_dependencies(positioning_node
  rclcpp
)
target_link_libraries(positioning_node
  positioning_node_lib
)

# Install targets
install(TARGETS
  beacon_config
  positioning_node_lib
  positioning_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install documentation
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Beacon config tests
  ament_add_gtest(test_beacon_config test/test_beacon_config.cpp)
  target_link_libraries(test_beacon_config beacon_config)
  ament_target_dependencies(test_beacon_config rclcpp)

  # Trilateration tests
  ament_add_gtest(test_trilateration test/test_trilateration.cpp)
  target_link_libraries(test_trilateration Eigen3::Eigen)

  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(beacon_config positioning_node_lib)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  mcu_msgs
  Eigen3
)

ament_package()
