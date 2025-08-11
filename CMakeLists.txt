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
target_link_libraries(rtsp_processor ${OpenCV_LIBS} nlohmann_json::nlohmann_json Threads::Threads)
if(OpenMP_CXX_FOUND)
    target_link_libraries(rtsp_processor OpenMP::OpenMP_CXX)
endif()

# zlib needed for voxel compression
find_package(ZLIB REQUIRED)
target_include_directories(rtsp_processor PRIVATE ${ZLIB_INCLUDE_DIRS})
target_link_libraries(rtsp_processor ${ZLIB_LIBRARIES})

# Install targets
install(TARGETS rtsp_processor DESTINATION bin)

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