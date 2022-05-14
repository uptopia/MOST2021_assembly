cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(motor_pose)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/motor_pose.cpp)

target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES} pcl_visualization)
