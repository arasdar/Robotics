cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stereo_aras)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package (OpenCV 2.4.4 REQUIRED)
link_directories(${OpenCV_LIB_DIR})
include_directories(${OpenCV_INCLUDE_DIR})

add_executable (stereo_capture stereo_vision_processing-capture.cpp)
target_link_libraries (stereo_capture ${PCL_LIBRARIES} ${OpenCV_LIBS})
