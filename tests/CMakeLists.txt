

if(NOT TARGET rtac_display)
    find_package(rtac_display REQUIRED)
endif()

list(APPEND test_files
    src/algebra_test.cpp
    src/emitter_test.cu
)
foreach(filename ${test_files})
    get_filename_component(test_name ${filename} NAME_WE)
    set(target_name ${test_name}_${PROJECT_NAME})
    add_executable(${target_name} ${filename})
    target_link_libraries(${target_name}
        rtac_simulation
        rtac_display
    )
    set_target_properties(${target_name} PROPERTIES
        CUDA_ARCHITECTURES native
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
