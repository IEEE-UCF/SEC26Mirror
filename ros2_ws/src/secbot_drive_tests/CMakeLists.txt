cmake_minimum_required(VERSION 3.8)
project(secbot_drive_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mcu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Shared test harness library
add_library(drive_test_harness SHARED
  src/test_harness.cpp
)

ament_target_dependencies(drive_test_harness
  rclcpp
  mcu_msgs
  nav_msgs
  geometry_msgs
)

# Test executables
set(TEST_NODES
  test_velocity
  test_goal
  test_trajectory
  test_stress
)

foreach(node ${TEST_NODES})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} drive_test_harness)
  ament_target_dependencies(${node}
    rclcpp
    mcu_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
  )
endforeach()

# Install
install(TARGETS drive_test_harness
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${TEST_NODES}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
