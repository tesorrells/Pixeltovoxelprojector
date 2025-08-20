cmake_minimum_required(VERSION 3.10)
project(PixelToVoxelRealTime)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find pthreads (required for std::thread)
find_package(Threads REQUIRED)

# Find OpenMP for parallel processing
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find DepthAI (optional) for OAK-D Pro camera support
find_package(depthai QUIET)
if(depthai_FOUND)
    message(STATUS "DepthAI found - enabling OAK-D Pro camera support")
    set(HAVE_DEPTHAI ON)
else()
    # Try to find DepthAI in common installation paths
    find_path(DEPTHAI_INCLUDE_DIR 
        NAMES depthai/depthai.hpp
        PATHS 
            /usr/local/include
            /usr/include
            ~/.local/include
            $ENV{HOME}/.local/include
    )
    
    find_library(DEPTHAI_LIBRARY
        NAMES depthai-core
        PATHS
            /usr/local/lib
            /usr/lib
            ~/.local/lib
            $ENV{HOME}/.local/lib
    )
    
    if(DEPTHAI_INCLUDE_DIR AND DEPTHAI_LIBRARY)
        message(STATUS "DepthAI found at ${DEPTHAI_INCLUDE_DIR} - enabling OAK-D Pro camera support")
        set(HAVE_DEPTHAI ON)
        set(DEPTHAI_LIBRARIES ${DEPTHAI_LIBRARY})
        set(DEPTHAI_INCLUDE_DIRS ${DEPTHAI_INCLUDE_DIR})
    else()
        message(STATUS "DepthAI not found - OAK-D Pro camera support disabled")
        message(STATUS "To enable OAK-D Pro support, install DepthAI C++ library")
        message(STATUS "For now, you can build without OAK-D Pro support")
    endif()
endif()

# Find nlohmann_json package - prefer system install, fall back to FetchContent
find_package(nlohmann_json 3.11.2 QUIET)

if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    message(STATUS "nlohmann_json not found system-wide; fetching via FetchContent")
    FetchContent_Declare(json
      URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
      # Updated SHA256 on 2024-02-05 (upstream re-packed):
      URL_HASH SHA256=8c4b26bf4b422252e13f332bc5e388ec0ab5c3443d24399acb675e68278d341f
    )
    FetchContent_MakeAvailable(json)
endif()

# Include directories
include_directories(src)

# Executable for RTSP processor
add_executable(rtsp_processor src/processing/rtsp_processor.cpp)
target_link_libraries(rtsp_processor ${OpenCV_LIBS} nlohmann_json::nlohmann_json Threads::Threads stdc++fs)
if(OpenMP_CXX_FOUND)
    target_link_libraries(rtsp_processor OpenMP::OpenMP_CXX)
endif()

# Add DepthAI support if available
if(HAVE_DEPTHAI)
    target_compile_definitions(rtsp_processor PRIVATE HAVE_DEPTHAI)
    if(depthai_FOUND)
        target_link_libraries(rtsp_processor depthai::core depthai::opencv)
    else()
        target_include_directories(rtsp_processor PRIVATE ${DEPTHAI_INCLUDE_DIRS})
        target_link_libraries(rtsp_processor ${DEPTHAI_LIBRARIES})
    endif()
endif()

# zlib needed for voxel compression
find_package(ZLIB REQUIRED)
target_include_directories(rtsp_processor PRIVATE ${ZLIB_INCLUDE_DIRS})
target_link_libraries(rtsp_processor ${ZLIB_LIBRARIES})

# Install targets
install(TARGETS rtsp_processor DESTINATION bin)

# Original offline ray_voxel implementation
add_executable(ray_voxel src/processing/ray_voxel.cpp)
target_include_directories(ray_voxel PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ray_voxel nlohmann_json::nlohmann_json Threads::Threads stdc++fs)
install(TARGETS ray_voxel DESTINATION bin)

# -------------------------- ZeroMQ (optional live publisher) -------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ QUIET libzmq)

if(ZeroMQ_FOUND)
    message(STATUS "ZeroMQ found – enabling live publisher")
    target_compile_definitions(rtsp_processor PRIVATE HAVE_ZMQ)
    target_include_directories(rtsp_processor PRIVATE ${ZeroMQ_INCLUDE_DIRS})
    target_link_libraries(rtsp_processor ${ZeroMQ_LIBRARIES})
else()
    message(STATUS "ZeroMQ not found – live publisher disabled")
endif()

# Executable for Drone Detector
add_executable(drone_detector
    src/main.cpp
    src/cot/cot_message_generator.cpp
    src/cot/cot_network_handler.cpp
    src/processing/voxel_drone_detector.cpp
)
target_link_libraries(drone_detector 
    ${OpenCV_LIBS} 
    nlohmann_json::nlohmann_json
    Threads::Threads
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(drone_detector OpenMP::OpenMP_CXX)
endif()

install(TARGETS drone_detector DESTINATION bin)

# Add a README with building instructions
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/README.md.in ${CMAKE_CURRENT_BINARY_DIR}/README.md @ONLY) 