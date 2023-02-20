

list(APPEND factory_test_files
    src/psf01_test.cpp
)

foreach(filename ${factory_test_files})
    get_filename_component(name ${filename} NAME_WE)
    set(target_name ${name}_factory_${PROJECT_NAME})
    add_executable(${target_name} ${filename})
    target_link_libraries(${target_name} PRIVATE rtac_simulation)
endforeach()


