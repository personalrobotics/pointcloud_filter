cmake_minimum_required(VERSION 2.8.3)
project(filter_pointcloud)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITONS})

add_executable(filter_pointcloud src/filter_pointclouds.cpp)
target_link_libraries(filter_pointcloud ${catkin_LIBRARIES})
target_link_libraries(filter_pointcloud ${PCL_LIBRARIES})
