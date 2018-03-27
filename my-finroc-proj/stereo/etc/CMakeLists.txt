cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project( sensorProcessing )

find_package(PCL 1.7 REQUIRED)
FIND_PACKAGE(OpenCV 2.4.4 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR} /home/aras/libv4l2cam_trunk/libv4l2cam/)
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})

#stereo
add_executable (stereo_offline stereo/offline/stereo_vision_processing-offline.cpp)
target_link_libraries (stereo_offline ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_executable ( stereo_capture stereo/capture/stereo_vision_processing-capture.cpp)
target_link_libraries ( stereo_capture ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (stereo_calib_capture-chessboard stereo/calibration/stereo_vision_processing-capture-chessboard.cpp)
target_link_libraries (stereo_calib_capture-chessboard ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (stereo_calib_offline stereo/calibration/stereo_calib.cpp)
target_link_libraries (stereo_calib_offline ${PCL_LIBRARIES} ${OpenCV_LIBS})


#kinect
add_executable ( kinect_offline kinect/offline/kinect_vision_processing-offline.cpp)
target_link_libraries ( kinect_offline ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_executable ( kinect_capture kinect/capture/kinect_vision_processing-capture.cpp)
target_link_libraries ( kinect_capture ${PCL_LIBRARIES} ${OpenCV_LIBS})

#webcam_calibration
add_executable( stereo_webcam_capture stereo_webcam/calibration/stereo_capture-chessboard.cpp)
target_link_libraries( stereo_webcam_capture ${OpenCV_LIBS} ${PCL_LIBRARIES} /home/aras/libv4l2cam_trunk/libv4l2cam/libcam.a)
add_executable( stereo_webcam_calib_offline stereo_webcam/calibration/stereo_calib.cpp)
target_link_libraries( stereo_webcam_calib_offline ${OpenCV_LIBS} ${PCL_LIBRARIES})
