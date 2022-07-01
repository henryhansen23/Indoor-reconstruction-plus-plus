include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(reconstruct reconstruction++.cpp 
                           load_data.cpp 
                           quaternion_interpolation.cpp   
                           combine_datapackets.cpp 
                           transformation.cpp 
                           registration.cpp
                           registration_estimation.cpp
			   cmdline.cpp)

target_link_libraries(reconstruct ${PCL_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(reconstruct
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

