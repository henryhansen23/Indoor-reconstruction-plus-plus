cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

cmake_policy(SET CMP0074 NEW) # search packages based on find_package pkg_ROOT hints AND pkg_ROOT env variables

project(Indoor-reconstruction-plus-plus-master)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.5 REQUIRED)

find_package(Boost 1.53.0 REQUIRED COMPONENTS filesystem program_options)

include_directories(include/)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(reconstruct)

add_subdirectory(interpolation_data_collection)

add_subdirectory(pcd2nicp)
add_subdirectory(plane_distances)

