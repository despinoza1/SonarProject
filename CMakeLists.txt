cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(detection)
LIST(APPEND CMAKE_MODULE_PATH "C:\\Program Files\\CMake\\share\\cmake-3.13\\Modules")
find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions( -DNOMINMAX )

add_executable (detection detection.cpp)
target_link_libraries (detection ${PCL_LIBRARIES})
