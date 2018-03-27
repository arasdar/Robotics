cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(upd_outdoor)

find_package(PCL 1.7 REQUIRED)
#set(CMAKE_C_FLAGS "-Wall")

  include_directories(${PCL_INCLUDE_DIRS} include)
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable(	main test.cpp UPD.cpp)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${USB_INCLUDE_DIRS})


  target_link_libraries (main ${PCL_IO_LIBRARIES})
#  target_link_libraries (main ${PCL_RANGE_IMAGE_LIBRARIES})
  target_link_libraries (main ${PCL_VISUALIZATION_LIBRARIES})
  target_link_libraries (main ${PCL_KEYPOINTS_LIBRARIES})
#  target_link_libraries (main ${PCL_RANGE_IMAGE_BORDER_EXTRACTOR_LIBRARIES})
  target_link_libraries (main ${PCL_KDTREE_LIBRARIES})
  target_link_libraries (main ${PCL_LIBRARIES})
  

######################################################################################
# ......................
######################################################################################

