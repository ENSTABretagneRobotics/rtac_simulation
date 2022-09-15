cmake_minimum_required(VERSION 3.10)
project(rtac_simulation VERSION 0.1 LANGUAGES CUDA CXX)

option(BUILD_TESTS "Build unit tests" ON)

if(NOT TARGET rtac_base)
    find_package(rtac_base REQUIRED)
endif()

if(NOT TARGET rtac_cuda)
    find_package(rtac_cuda REQUIRED)
endif()

if(NOT TARGET rtac_optix)
    find_package(rtac_optix REQUIRED)
endif()

list(APPEND rtac_simulation_headers
    include/rtac_simulation/utils.hcu
    include/rtac_simulation/common.h
    include/rtac_simulation/geometry.h
    include/rtac_simulation/RangeBinner.h
    include/rtac_simulation/Directivity.h
    include/rtac_simulation/Antenna.h
    include/rtac_simulation/Receiver.h
    include/rtac_simulation/Emitter.h
    include/rtac_simulation/Sample.h
    include/rtac_simulation/ConvolutionKernel.h
    include/rtac_simulation/reductions.hcu
    include/rtac_simulation/Target2D.h

    include/rtac_simulation/PolarKernel2D.h

    include/rtac_simulation/helpers/receiver_factories.h
)

add_library(rtac_simulation SHARED
    src/Directivity.cpp
    src/RangeBinner.cu
)
target_include_directories(rtac_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rtac_simulation PUBLIC
    rtac_base
    rtac_cuda
)
target_compile_features(rtac_simulation PUBLIC cxx_std_17)
rtac_install_target(rtac_simulation
    HEADER_FILES ${rtac_simulation_headers}
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
    
