include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(reconstruct reconstruction++.cpp 
                           load_data.cpp                load_data.h
                           quaternion_interpolation.cpp quaternion_interpolation.h
                           combine_datapackets.cpp      combine_datapackets.h
                           transformation.cpp           transformation.h
			   reco_types.cpp               reco_types.h
                           registration.cpp             registration.h
                           registration_estimation.cpp  registration_estimation.h
			   cmdline.cpp                  cmdline.h )

target_link_libraries(reconstruct ${PCL_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(reconstruct
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

