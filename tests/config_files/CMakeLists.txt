

set(target_name config_file_${PROJECT_NAME})
add_executable(${target_name}
    src/config01.cpp
)
target_link_libraries(${target_name} PRIVATE rtac_simulation)
target_compile_definitions(${target_name} PRIVATE 
    "-DRTAC_SIMULATION_TESTS_CONFIG_FILE=\"${CMAKE_SOURCE_DIR}/config_examples/config01.yaml\""
)
