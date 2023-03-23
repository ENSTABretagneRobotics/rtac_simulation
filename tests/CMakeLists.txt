
find_package(rosbag REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(narval_oculus REQUIRED)
find_package(oculus_sonar REQUIRED)
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
)

rtac_install_target(rosbag_helpers
    HEADER_FILES ${rosbag_helpers_headers}
)

list(APPEND test_files
    src/waveform01.cpp
    src/directivity01.cpp
    src/filefinder01.cpp
    src/sensor_info_factory01.cpp
    src/simulation_factory01.cpp
)
if(${WITH_OPENGL})
    list(APPEND test_files src/glsim_factory.cpp)
endif()
foreach(filename ${test_files})
    get_filename_component(test_name ${filename} NAME_WE)
    set(target_name ${test_name}_${PROJECT_NAME})
    add_executable(${target_name} ${filename})
    target_link_libraries(${target_name}
        rtac_simulation
    )
    # target_compile_definitions(${target_name} PRIVATE
    #                            RTAC_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../examples")
endforeach()

if(${WITH_OPENGL})
    if(${WITH_OPTIX})
        add_subdirectory(ray_caster02)
        add_subdirectory(gl_sim01)
    endif()
    add_subdirectory(gl_only01)
endif()

