cmake_minimum_required(VERSION 3.10)
project(rtac_simulation VERSION 0.1 LANGUAGES CUDA CXX)

option(BUILD_TESTS "Build unit tests" ON)

if(NOT TARGET rtac_base)
    find_package(rtac_base REQUIRED)
endif()

if(NOT TARGET rtac_cuda)
    find_package(rtac_cuda REQUIRED)
endif()

find_package(yaml-cpp REQUIRED)

if(NOT TARGET rtac_optix)
    find_package(rtac_optix QUIET)
endif()

# These package are not mandatory. it only allows for easy setup in oculus
# example.
find_package(rtac_display  QUIET) # Display utilities


list(APPEND rtac_simulation_headers
    include/rtac_simulation/utils.hcu
    include/rtac_simulation/RangeBinner.h
    include/rtac_simulation/Directivity.h
    include/rtac_simulation/Antenna.h
    include/rtac_simulation/Receiver.h
    include/rtac_simulation/Emitter.h
    include/rtac_simulation/Sample.h
    include/rtac_simulation/ConvolutionKernel.h
    include/rtac_simulation/reductions.hcu
    include/rtac_simulation/Target2D.h

    include/rtac_simulation/RayCaster.h

    include/rtac_simulation/PolarKernel2D.h
    include/rtac_simulation/PolarTarget2D.h
    include/rtac_simulation/PolarReceiver2D.h

    include/rtac_simulation/helpers/receiver_factories.h

    include/rtac_simulation/examples/blueprint_oculus.h
)

add_library(rtac_simulation SHARED
    src/Directivity.cpp
    src/Emitter.cu
    src/RangeBinner.cu

    src/Receiver.cu
    src/RayCaster.cpp

    src/PolarReceiver2D.cu

    src/examples/blueprint_oculus.cu
)
target_include_directories(rtac_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rtac_simulation PUBLIC
    rtac_base
    rtac_cuda
    yaml-cpp
)
target_compile_features(rtac_simulation PUBLIC cxx_std_17)

# if optix found adding optix dependent modules.
if(rtac_optix_FOUND)
    set(OPTIX_SUPPORT_ENABLED ON)
    target_compile_definitions(rtac_simulation PUBLIC RTAC_WITH_OPTIX)
    target_sources(rtac_simulation PRIVATE src/PolarRayCaster.cpp)
    list(APPEND rtac_simulation_headers include/rtac_simulation/PolarRayCaster.h)
    target_link_libraries(rtac_simulation PUBLIC rtac_optix)
    target_add_ptx(rtac_simulation
        CUDA_SOURCES src/PolarRayCaster.cu src/RayCaster.cu
    )
else()
    message(WARNING "The rtac_optix library was not found on your system.\n"
                    "\tRay tracing support is disabled.")
endif()

find_package(narval_oculus REQUIRED)
get_target_property(include_dir narval_oculus INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(rtac_simulation PUBLIC ${include_dir})

if(TARGET rtac_display)
    set(DISPLAY_SUPPORT_ENABLED ON)
    list(APPEND rtac_simulation_headers
        include/rtac_simulation/helpers/PolarTargetRenderer.h
    )
    target_sources(rtac_simulation PRIVATE src/helpers/PolarTargetRenderer.cu)
    target_link_libraries(rtac_simulation PUBLIC rtac_display)
endif()

rtac_install_target(rtac_simulation
    HEADER_FILES ${rtac_simulation_headers}
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

