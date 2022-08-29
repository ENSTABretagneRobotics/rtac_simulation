

list(APPEND test_files
    src/wavefield.cu
)
foreach(filename ${test_files})
    get_filename_component(test_name ${filename} NAME_WE)
    set(target_name ${test_name}_${PROJECT_NAME})
    add_executable(${target_name} ${filename})
    target_link_libraries(${target_name}
        rtac_simulation
    )
    set_target_properties(${target_name} PROPERTIES
        CUDA_ARCHITECTURES native
    )
endforeach()
