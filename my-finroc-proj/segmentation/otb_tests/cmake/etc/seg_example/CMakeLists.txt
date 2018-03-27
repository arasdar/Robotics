cmake_minimum_required(VERSION 2.8)

project(Watershed_Example)
include_regular_expression("^.*$")

find_package(OTB)
if(OTB_FOUND)
  include(${OTB_USE_FILE})
else()
  message(FATAL_ERROR
          "Cannot build HelloWorld application without OTB.  Please set OTB_DIR.")
endif()

add_executable(WatershedSegmentation WatershedSegmentation.cxx )
target_link_libraries(WatershedSegmentation OTBCommon OTBIO ${ITK_LIBRARIES})

