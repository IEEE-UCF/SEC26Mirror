cmake_minimum_required(VERSION 3.8)
project(secbot_jh_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


#install worlds, launch , and the jh_testing(models) directory
install(DIRECTORY
	worlds
	launch
	Coke
	# worlds/Proper-field
	# secbot_jh_test
	DESTINATION share/${PROJECT_NAME}
)
ament_package()
#[==[
When you run colcon build, ROS2 creates an install/folder
	-ROS2 does NOT run code directly from your src folder. It runs it from the  install/(I don't know where this is located, all i know is that it needs to be run there)
	-install(DIRECTORY..): this command tells the build tool to copy your folders(worlds,launch,jh_testing) into the same area as the install/ folder(through something called a share directory)

]==]