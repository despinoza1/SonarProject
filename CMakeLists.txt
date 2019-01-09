cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(converter)
LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-3.2/Modules/")

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3)
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (converter converter.cpp)
target_link_libraries (converter ${PCL_LIBRARIES} Eigen3::Eigen)
