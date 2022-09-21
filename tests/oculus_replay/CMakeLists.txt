
if(NOT TARGET rtac_display)
    find_package(rtac_display REQUIRED)
endif()

# find_package(rosbag REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(oculus_sonar REQUIRED)
find_package(narval_oculus REQUIRED)

set(target_name "oculus_replay_${PROJECT_NAME}")
add_executable(${target_name}
    src/acoustics_test0.cpp
)
target_link_libraries(${target_name} PRIVATE
    rtac_simulation
    rtac_display
    # ${rosbag_LIBRARIES}
    # ${nav_msgs_LIBRARIES}
    # ${oculus_sonar_LIBRARIES}
    narval_oculus
    rosbag_helpers
)
# target_include_directories(${target_name} PRIVATE
#     ${rosbag_INCLUDE_DIRS}
#     ${nav_msgs_INCLUDE_DIRS}
#     ${oculus_sonar_INCLUDE_DIRS}
# )
set_target_properties(${target_name} PROPERTIES CUDA_ARCHITECTURES native)
target_add_ptx(${target_name}
    CUDA_SOURCES src/oculus_sim.cu
)

