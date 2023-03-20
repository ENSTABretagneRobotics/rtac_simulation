
if(NOT TARGET rtac_display)
    find_package(rtac_display REQUIRED)
endif()

find_package(narval_oculus REQUIRED)

set(target_name "gl_only01_${PROJECT_NAME}")
add_executable(${target_name}
    src/acoustics_test0.cpp
)
target_link_libraries(${target_name} PRIVATE
    rtac_simulation
    rtac_display
    narval_oculus
    rosbag_helpers
)
target_compile_definitions(${target_name} PRIVATE
                           RTAC_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../../examples")
