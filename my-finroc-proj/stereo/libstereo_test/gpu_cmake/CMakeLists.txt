# collect source files
cmake_minimum_required(VERSION 2.8)

project(stereo)

find_package(CUDA)
include_directories("/usr/include/opencv2")

set(CUDA_NVCC_FLAGS "-arch=sm_30")

file( GLOB  hdr *.hpp *.h )
file( GLOB  cu  *.cu)

SET(stereo_test_gpu_part_files test_gpu_part.cpp)
CUDA_ADD_EXECUTABLE(stereo_test_gpu_part ${stereo_test_gpu_part_files} ${hdr} ${cu})
target_link_libraries(stereo_test_gpu_part opencv_core opencv_highgui opencv_imgproc)