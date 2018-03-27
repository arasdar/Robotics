# collect source files
cmake_minimum_required(VERSION 2.8)

project(stereo)

find_package(CUDA)
include_directories("/usr/include/opencv2")

set(CUDA_NVCC_FLAGS "-arch=sm_30")
#set(CUDA_HOST_COMPILER "/usr/bin/gcc-4.6")
#set(CMAKE_CXX_COMPILER "g++-4.6")
#set(CMAKE_CC_COMPILER "gcc-4.6")


file( GLOB  hdr *.hpp *.h )
file( GLOB  cu  *.cu)

SET (stereo_main_files main.cpp)
CUDA_ADD_EXECUTABLE(stereo_main ${stereo_main_files} ${hdr} ${cu})
#add_executable(stereo_main main.cpp)
target_link_libraries(stereo_main opencv_core opencv_highgui opencv_imgproc)


add_executable(stereo_test_cpu cpu/test_cpu.cpp)
target_link_libraries(stereo_test_cpu opencv_core opencv_highgui opencv_imgproc)

SET(stereo_test_gpu_files gpu/test_gpu.cpp)
CUDA_ADD_EXECUTABLE(stereo_test_gpu ${stereo_test_gpu_files} ${hdr} ${cu})
target_link_libraries(stereo_test_gpu opencv_core opencv_highgui opencv_imgproc)