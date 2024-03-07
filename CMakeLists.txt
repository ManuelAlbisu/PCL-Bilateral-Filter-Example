cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(bilateral_filter)

find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bilateral_filter bilateral_filter.cpp)
target_link_libraries(bilateral_filter ${PCL_LIBRARIES})
