cmake_minimum_required(VERSION 3.8)
project(abu_nav)

set(CMAKE_C_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(ABU_NAV_DEPS
	rclcpp
	rclcpp_components
	geometry_msgs
	nav_msgs
	std_msgs
	tf2
	tf2_ros
)

add_library(vl53lib
	src/vl53l1_linux_platform.c
        src/VL53L1X_api.c
        src/VL53L1X_calibration.c
	include/vl53l1_platform.h
	include/vl53l1_types.h
	include/VL53L1X_api.h
	include/VL53L1X_calibration.h
)
target_include_directories(
	vl53lib PRIVATE
	include
)

add_executable(abu_nav_node 
	src/abu_nav_node.cpp 
	include/vl53l1_platform.h
	include/vl53l1_types.h
	include/VL53L1X_api.h
	include/VL53L1X_calibration.h
)
target_include_directories(
	abu_nav_node PRIVATE
	include/
)

target_link_libraries(
	abu_nav_node
	vl53lib
)

ament_target_dependencies(
        abu_nav_node
        ${ABU_NAV_DEPS}
)

install(
	TARGETS abu_nav_node
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY launch params
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
