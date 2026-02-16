cmake_minimum_required(VERSION 3.8)
project(uwb_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mcu_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(uwb_simulation_node
  src/uwb_simulation_main.cpp
  src/uwb_simulation.cpp
  src/tinyxml2.cpp)

target_include_directories(uwb_simulation_node PUBLIC include)

target_link_libraries(uwb_simulation_node
  gz-transport${gz-transport13_VERSION_MAJOR}::core)

ament_target_dependencies(uwb_simulation_node
  rclcpp
  geometry_msgs
  ament_index_cpp
  mcu_msgs)

install(TARGETS
  uwb_simulation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
