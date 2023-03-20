cmake_minimum_required(VERSION 3.10)
project(rtac_simulation VERSION 0.1 LANGUAGES CUDA CXX)

option(BUILD_TESTS "Build unit tests" ON)

if(NOT TARGET rtac_base)
    find_package(rtac_base REQUIRED)
endif()

if(NOT TARGET rtac_cuda)
    find_package(rtac_cuda REQUIRED)
endif()

# These package are not mandatory. it only allows for easy setup in oculus
# example.
find_package(rtac_display REQUIRED) # Display utilities
find_package(rtac_optix   REQUIRED)


list(APPEND rtac_simulation_headers
    include/rtac_simulation/Directivity.h
    include/rtac_simulation/SensorInfo.h
    include/rtac_simulation/Receiver.h
    include/rtac_simulation/Emitter.h

    include/rtac_simulation/Sample.h

    include/rtac_simulation/Waveform.h
    include/rtac_simulation/SensorInstance.h
    include/rtac_simulation/Binner.h
    include/rtac_simulation/RayCaster.h

    include/rtac_simulation/Simulation.h
    include/rtac_simulation/OptixSimulation.h

    include/rtac_simulation/EmitterGL.h
    include/rtac_simulation/RayCasterGL.h
    include/rtac_simulation/SimulationGL.h

    include/rtac_simulation/factories/utilities.h
    include/rtac_simulation/factories/EmitterFactory.h
    include/rtac_simulation/factories/SensorInfoFactory.h
)

add_library(rtac_simulation SHARED
    src/Directivity.cpp
    src/Emitter.cu
    src/Receiver.cu
    src/SensorInstance.cu
    src/Binner.cu
    src/RayCaster.cpp

    src/OptixSimulation.cpp

    src/EmitterGL.cpp
    src/RayCasterGL.cpp
    src/SimulationGL.cu

    src/factories/utilities.cpp
    src/factories/EmitterFactory.cpp
    src/factories/SensorInfoFactory.cpp
)
target_include_directories(rtac_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rtac_simulation PUBLIC
    rtac_base
    rtac_cuda
    rtac_optix
    rtac_display
    yaml-cpp
)
target_compile_options(rtac_simulation PUBLIC
    # -DRTAC_CONFIG_FILES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/examples/"
    -DRTAC_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/examples/"
)
target_add_ptx(rtac_simulation
    CUDA_SOURCES src/RayCaster.cu
)
# target_compile_definitions(rtac_simulation PUBLIC RTAC_WITH_OPTIX)

find_package(narval_oculus REQUIRED)
get_target_property(include_dir narval_oculus INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(rtac_simulation PUBLIC ${include_dir})

rtac_install_target(rtac_simulation
    HEADER_FILES ${rtac_simulation_headers}
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

