cmake_minimum_required (VERSION 2.8.11)
project (vrcapriltags)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_COMPILER "g++")

include_directories("/usr/include/opencv4/")
include_directories("/usr/local/cuda-10.2/include")
include_directories("/opt/nvidia/vpi1/include/")

link_directories("./libraries/lib_aarch64_jetpack44")
link_directories("/usr/local/cuda-10.2/lib64")
link_directories("/opt/nvidia/vpi1/lib64")


link_libraries(april_tagging cudart cuda cublas opencv_core opencv_videoio opencv_imgproc opencv_highgui nvvpi)

set(SOURCE_FILES vrcapriltags.cpp)

add_executable (vrcapriltags ${SOURCE_FILES})

#RUN g++ -std=c++11 -I/usr/include/opencv4/ -I/usr/local/cuda-10.2/include -I/opt/nvidia/vpi1/include/ -I./libraries/json/inlcude  -L./libraries/lib_aarch64_jetpack44 -L/usr/local/cuda-10.2/lib64  -L/opt/nvidia/vpi1/lib64 -L/opt/nvidia/vpi/lib vrcapriltags.cpp  -lapril_tagging -lcudart -lcuda -lcublas -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lnvvpi
