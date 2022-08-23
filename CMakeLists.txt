cmake_minimum_required(VERSION 3.10)
project(rtac_simulation VERSION 0.1 LANGUAGES CUDA CXX)

if(NOT TARGET rtac_base)
    find_package(rtac_base REQUIRED)
endif()

if(NOT TARGET rtac_cuda)
    find_package(rtac_cuda REQUIRED)
endif()

list(APPEND rtac_simulation_headers
    include/rtac_simulation/utils.hcu
    include/rtac_simulation/ConvolutionKernel.h
    include/rtac_simulation/reductions.hcu
    include/rtac_simulation/Target2D.h

    # include/rtac_simulation/WaveSamples.h
)

add_library(rtac_simulation INTERFACE)
set_target_properties(rtac_simulation PROPERTIES
    RTAC_PUBLIC_HEADERS "${rtac_simulation_headers}"
)
target_link_libraries(rtac_simulation INTERFACE
    rtac_base
    rtac_cuda
)
target_compile_features(rtac_simulation INTERFACE cxx_std_17)
rtac_install_target(rtac_simulation)

add_subdirectory(tests)
    
