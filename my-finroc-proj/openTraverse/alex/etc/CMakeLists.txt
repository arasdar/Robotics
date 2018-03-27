cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(alex)

find_package(PCL 1.7 REQUIRED)
#set(CMAKE_C_FLAGS "-Wall")

  include_directories(${PCL_INCLUDE_DIRS} include)
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable(	main stereo_ground_segmentation.cpp)
target_link_libraries (main ${PCL_LIBRARIES})
  

######################################################################################
# ......................
######################################################################################

