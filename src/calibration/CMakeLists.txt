cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

cmake_policy(SET CMP0074 NEW) # search packages based on find_package pkg_ROOT hints AND pkg_ROOT env variables

project(calibration)

set(CMAKE_CXX_STANDARD 11)

add_executable(calibration
               Tinkerforge_IMU2.0/brick_imu_v2.cpp
               Tinkerforge_IMU2.0/brick_imu_v2.h
               Tinkerforge_IMU2.0/ip_connection.cpp
               Tinkerforge_IMU2.0/ip_connection.h
               calibration.cpp)

# Find package thread

find_package(Threads REQUIRED)

# Find package boost

set(BOOST_ROOT "/usr/local/Cellar/boost/")
find_package(Boost COMPONENTS system date_time regex)

# If direct capture from sensors, VelodyneCapture are required Boost.Asio.

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

# Other definitions

add_definitions(-DNO_EMPTY_RETURNS) # Don't push null returns to the lases queue
add_definitions(-DPUSH_SINGLE_PACKETS) # Push data packets as soon as possible

# Additional library directories

target_link_directories(calibration
                        PUBLIC ${Boost_LIBRARY_DIRS}
			${PCL_LIBRARY_DIRS})

# Additional dependencies

target_link_libraries(calibration ${CMAKE_THREAD_LIBS_INIT}
                                        ${Boost_LIBRARIES}
                                        ${PCL_LIBRARIES})

set_target_properties(calibration
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

