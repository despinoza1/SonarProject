cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(converter)

find_package(PCL 1.7 REQUIRED)
find_package(EIGEN3 3.0 REQUIRED NO_MODULE)

#set(EIGEN3_DIR /home/vizlab/Desktop/eigen-eigen-5a0156e40feb/build/)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (converter converter.cpp)
target_link_libraries (converter ${PCL_LIBRARIES} Eigen3::Eigen)
