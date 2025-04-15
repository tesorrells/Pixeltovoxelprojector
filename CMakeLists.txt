cmake_minimum_required(VERSION 3.10)
project(PixelToVoxelRealTime)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find OpenMP for parallel processing
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find nlohmann_json package - fetch it if not found
include(FetchContent)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
  URL_HASH SHA256=8c586f118abf3d3a19c1454c40dddf02f736dbdb8ae939ba113d8c7f1a824048
)
FetchContent_MakeAvailable(json)

# Include directories
include_directories(src)

# Executable for RTSP processor
add_executable(rtsp_processor src/processing/rtsp_processor.cpp)
target_link_libraries(rtsp_processor ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
if(OpenMP_CXX_FOUND)
    target_link_libraries(rtsp_processor OpenMP::OpenMP_CXX)
endif()

# Install targets
install(TARGETS rtsp_processor DESTINATION bin)

# Executable for Drone Detector
add_executable(drone_detector
    src/main.cpp
    src/cot/cot_message.cpp
    src/cot/cot_message_generator.cpp
    src/cot/cot_network_handler.cpp
    src/processing/voxel_drone_detector.cpp
)
target_link_libraries(drone_detector 
    ${OpenCV_LIBS} 
    nlohmann_json::nlohmann_json
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(drone_detector OpenMP::OpenMP_CXX)
endif()

install(TARGETS drone_detector DESTINATION bin)

# Add a README with building instructions
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/README.md.in ${CMAKE_CURRENT_BINARY_DIR}/README.md @ONLY) 