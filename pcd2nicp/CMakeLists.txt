cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

cmake_policy(SET CMP0074 NEW) # search packages based on find_package pkg_ROOT hints AND pkg_ROOT env variables

project(pcd2nicp)

set(CMAKE_CXX_STANDARD 11)

# Find package boost

# set(BOOST_ROOT "/usr/local/Cellar/boost/")
find_package(Boost COMPONENTS system)

set(HAVE_BOOST)
if(Boost_FOUND)
    set(HAVE_BOOST "-DHAVE_BOOST")
endif()

# Find package PCL

find_package(PCL 1.5 REQUIRED)

# Additional include directories

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Add definitions

add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
add_definitions(${HAVE_BOOST})
add_definitions(${PCL_DEFINITIONS})

################################################################################
# Converter from PCD to NICP
################################################################################
add_executable(pcd2nicp
               pcd2nicp.cpp)
target_link_directories(pcd2nicp
                        PUBLIC ${Boost_LIBRARY_DIRS}
			${PCL_LIBRARY_DIRS})
target_link_libraries(pcd2nicp
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES})

################################################################################
# Converter from binary NICP to textualNICP
################################################################################

add_executable(nicpb2nicp
               nicpb2nicp.cpp)
target_link_directories(nicpb2nicp
                        PUBLIC ${Boost_LIBRARY_DIRS}
			${PCL_LIBRARY_DIRS})
target_link_libraries(nicpb2nicp
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES})

