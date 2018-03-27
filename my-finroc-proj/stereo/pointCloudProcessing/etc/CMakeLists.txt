cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stereo_aras)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package (OpenCV 2.4.4 REQUIRED)
link_directories(${OpenCV_LIB_DIR})
include_directories(${OpenCV_INCLUDE_DIR})

add_executable (kinect_offline kinect_vision_processing-offline.cpp)
target_link_libraries (kinect_offline ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable (kinect_offline_original kinect_vision_processing-offline_original.cpp)
#target_link_libraries (kinect_offline_original ${PCL_LIBRARIES} ${OpenCV_LIBS})
