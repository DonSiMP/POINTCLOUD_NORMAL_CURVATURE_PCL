cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_NORMAL)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCL_NORMAL PCL_NORMAL.cpp)
target_link_libraries (PCL_NORMAL ${PCL_LIBRARIES})