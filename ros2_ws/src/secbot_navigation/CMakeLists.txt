cmake_minimum_required(VERSION 3.8)
project(secbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# Add these only if you include them in code:
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)


# find_package(std_msgs REQUIRED)

add_executable(pathing_node
  src/pathing_node.cpp
  src/grid_map.cpp
  src/dstar_lite.cpp
  src/path_smoother.cpp
  src/trajectory.cpp
)


# This fixes "secbot_navigation/xxx.hpp not found"
target_include_directories(pathing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pathing_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  ament_index_cpp
  # std_msgs
)

target_link_libraries(pathing_node yaml-cpp)


install(TARGETS pathing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)

ament_package()
