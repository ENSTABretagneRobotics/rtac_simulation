
find_package(rosbag REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(narval_oculus REQUIRED)
find_package(oculus_sonar REQUIRED)
find_package(rtac_display REQUIRED)
list(APPEND rosbag_helpers_headers
    include/rtac_simulation/helpers/OculusRosbagIterator.h
)
add_library(rosbag_helpers
    src/helpers/OculusRosbagIterator.cpp
)
target_include_directories(rosbag_helpers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rosbag_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${oculus_sonar_INCLUDE_DIRS}
)
target_link_libraries(rosbag_helpers PUBLIC
    rtac_simulation
    narval_oculus
    ${rosbag_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${oculus_sonar_LIBRARIES}
    rtac_display
)

rtac_install_target(rosbag_helpers
    HEADER_FILES ${rosbag_helpers_headers}
)


if(NOT TARGET rtac_display)
    find_package(rtac_display REQUIRED)
endif()

if(OPTIX_SUPPORT_ENABLED)
    list(APPEND test_files
        # src/algebra_test.cpp
        # src/emitter_test.cu
        # src/kernel_test.cu
    )
endif()
foreach(filename ${test_files})
    get_filename_component(test_name ${filename} NAME_WE)
    set(target_name ${test_name}_${PROJECT_NAME})
    add_executable(${target_name} ${filename})
    target_link_libraries(${target_name}
        rtac_simulation
        rtac_display
    )
endforeach()



# rtac_generate_asm(index_test_asm
#     SOURCES src/algebra_test_indexes.cpp
#     LINK_LIBRARIES rtac_simulation
# )
# rtac_generate_ptx(index_test_ptx
#     SOURCES src/algebra_test_indexes.cu
#     LINK_LIBRARIES rtac_simulation
# )

if(OPTIX_SUPPORT_ENABLED)
    # add_subdirectory(oculus_replay)
    add_subdirectory(ray_caster01)
    add_subdirectory(gl_sim01)
endif()
add_subdirectory(gl_only01)

# add_subdirectory(config_files)
