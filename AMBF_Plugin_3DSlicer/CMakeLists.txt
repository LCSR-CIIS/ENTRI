cmake_minimum_required(VERSION 3.16)
project(AMBF_3DSlicer_Plugin)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)

# find_package(Eigen3 REQUIRED PATH ~/eign-3.4.0/Eigen)

# TODO: what are those packages?
find_package(Boost COMPONENTS program_options filesystem)

# TODO: what does this do?
add_definitions(-DC_ENABLE_AMBF_COMM_SUPPORT)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${Eigen3_INCLUDE_DIRS})


add_definitions(${AMBF_DEFINITIONS})
add_subdirectory(plugins)
