cmake_minimum_required(VERSION 3.10)
project(rtac_simulation VERSION 0.2 LANGUAGES CUDA CXX)

option(BUILD_TESTS "Build unit tests" ON)

if(NOT TARGET rtac_base)
    find_package(rtac_base 0.2 REQUIRED)
endif()

if(NOT TARGET rtac_cuda)
    find_package(rtac_cuda 0.2 REQUIRED)
endif()

# These package are not mandatory. it only allows for easy setup in oculus
# example.
find_package(rtac_display 0.2 QUIET) # Display utilities
find_package(rtac_optix   0.2 QUIET)

if(NOT TARGET rtac_optix)
    message(WARNING "Warning : could not find the rtac_optix library. "
                    "Ray-tracing components are disabled")
endif()

if(NOT TARGET rtac_display)
    message(WARNING "Warning : could not find the rtac_display library. "
        "Rasterization components and graphical display are disabled")
endif()

if(NOT TARGET rtac_display AND NOT TARGET rtac_optix)
    message(FATAL_ERROR "Neither rtac_optix nor rtac_display libraries were found."
        " No simulation may run without one or/and the other.")
endif()

list(APPEND rtac_simulation_headers
    include/rtac_simulation/Directivity.h
    include/rtac_simulation/SensorInfo.h
    include/rtac_simulation/Receiver.h
    include/rtac_simulation/Emitter.h

    include/rtac_simulation/Sample.h

    include/rtac_simulation/Waveform.h
    include/rtac_simulation/SensorInstance.h
    include/rtac_simulation/SensorInstance1D.h
    include/rtac_simulation/SensorInstance2D.h
    include/rtac_simulation/Simulation.h
    include/rtac_simulation/Binner.h

    include/rtac_simulation/sinks/Sink.h
    include/rtac_simulation/sinks/FileSink.h
    include/rtac_simulation/PoseSource.h

    include/rtac_simulation/factories/utilities.h
    include/rtac_simulation/factories/EmitterFactory.h
    include/rtac_simulation/factories/SensorInfoFactory.h
    include/rtac_simulation/factories/SimulationFactory.h
    include/rtac_simulation/factories/SinkFactory.h
)

add_library(rtac_simulation SHARED
    src/Directivity.cpp
    src/Emitter.cu
    src/Receiver.cu
    src/SensorInstance.cu
    src/SensorInstance1D.cu
    src/SensorInstance2D.cu
    src/Binner.cu

    src/sinks/FileSink.cpp
    src/PoseSource.cpp

    src/factories/utilities.cpp
    src/factories/EmitterFactory.cpp
    src/factories/SensorInfoFactory.cpp
    src/factories/SimulationFactory.cpp
    src/factories/SinkFactory.cpp
)
target_include_directories(rtac_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rtac_simulation PUBLIC
    rtac_base
    rtac_cuda
    yaml-cpp
    m
)
target_compile_options(rtac_simulation PUBLIC
    -DRTAC_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/examples/"
)

if(TARGET rtac_optix)
    list(APPEND rtac_simulation_headers
        include/rtac_simulation/RayCaster.h
        include/rtac_simulation/OptixSimulation.h
    )
    target_sources(rtac_simulation PRIVATE
        src/RayCaster.cpp
        src/OptixSimulation.cpp
    )
    target_link_libraries(rtac_simulation PUBLIC
        rtac_optix
    )
    target_add_ptx(rtac_simulation
        CUDA_SOURCES src/RayCaster.cu
    )
    target_compile_definitions(rtac_simulation PUBLIC
        RTAC_SIMULATION_OPTIX_ENABLED
    )
    set(WITH_OPTIX ON)
endif()

if(TARGET rtac_display)
    list(APPEND rtac_simulation_headers
        include/rtac_simulation/EmitterGL.h
        include/rtac_simulation/RayCasterGL.h
        include/rtac_simulation/SimulationGL.h
        include/rtac_simulation/sinks/DisplaySink.h
    )
    target_sources(rtac_simulation PRIVATE
        src/EmitterGL.cpp
        src/RayCasterGL.cpp
        src/SimulationGL.cu
        src/sinks/DisplaySink.cpp
    )
    target_link_libraries(rtac_simulation PUBLIC
        rtac_display
    )
    target_compile_definitions(rtac_simulation PUBLIC
        RTAC_SIMULATION_DISPLAY_ENABLED
        RTAC_SIMULATION_OPENGL_ENABLED
    )
    set(WITH_OPENGL ON)
endif()

# TODO remove this
find_package(narval_oculus REQUIRED)
get_target_property(include_dir narval_oculus INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(rtac_simulation PUBLIC ${include_dir})

rtac_install_target(rtac_simulation
    HEADER_FILES ${rtac_simulation_headers}
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

