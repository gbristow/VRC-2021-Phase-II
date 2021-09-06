cmake_minimum_required (VERSION 2.8.11)
project (vrcapriltags)

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(./libraries/json)

include_directories("/usr/include/opencv4/")
include_directories("/usr/local/cuda-10.2/include")
include_directories("/opt/nvidia/vpi1/include")
include_directories("../libraries/json/include")



link_directories("../libraries/lib_aarch64_jetpack44")
link_directories("/usr/local/cuda-10.2/lib64")
link_directories("/opt/nvidia/vpi1/lib64")


link_libraries(april_tagging cudart cuda cublas opencv_core opencv_videoio opencv_imgproc opencv_highgui nvvpi)

set(SOURCE_FILES src/vrcapriltags.cpp src/apriltags.cpp src/undistort.cpp src/cam_properties.cpp)

add_executable (vrcapriltags ${SOURCE_FILES})
target_link_libraries(vrcapriltags PRIVATE nlohmann_json::nlohmann_json)