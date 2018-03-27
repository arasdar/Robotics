cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( stereo_calibration )

FIND_PACKAGE(OpenCV 2.4.4 REQUIRED )
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR} )
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIB_DIR} )
add_definitions(${PCL_DEFINITIONS})

#add_executable( stereo_calib stereo_calib.cpp)
#target_link_libraries( stereo_calib ${OpenCV_LIBS} ${PCL_LIBRARIES} )

#add_executable( stereo_capture-chessboard   stereo_vision_processing-capture-chessboard.cpp )
#target_link_libraries ( stereo_capture-chessboard  ${OpenCV_LIBS}  ${PCL_LIBRARIES} )

add_executable( test surf_keypoint_matcher.cpp)
target_link_libraries( test ${OpenCV_LIBS} ${PCL_LIBRARIES} )
