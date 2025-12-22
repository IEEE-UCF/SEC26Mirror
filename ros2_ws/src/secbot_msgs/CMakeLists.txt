# Messages & Interfaces: Small messages so nodes can talk cleanly. Configuration for CMake build system.

cmake_minimum_required(VERSION 3.8)
project(secbot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/AntennaMarker.msg"
  "msg/ArmCommand.msg"
  "msg/DriveCommand.msg"
  "msg/DuckDetection.msg"
  "msg/DuckDetections.msg"
  "msg/IRCommand.msg"
  "msg/ImuLite.msg"
  "msg/LedColor.msg"
  "msg/Odom2D.msg"
  "msg/Pose2D.msg"
  "msg/RangeArray.msg"
  "msg/TaskStatus.msg"
  # Actions
  "action/ApproachTarget.action"
  "action/NavigatePath.action"
  # Services
  "srv/SetTask.srv"
  "srv/StartStop.srv"
  DEPENDENCIES std_msgs vision_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
