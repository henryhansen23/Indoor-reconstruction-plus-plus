include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(get_plane_distances get_plane_distances.cpp)

target_link_libraries(get_plane_distances ${PCL_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(get_plane_distances
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

