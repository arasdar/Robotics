cmake_minimum_required(VERSION 2.6)
project(Segmentation_App)

find_package(OTB)
if(OTB_FOUND)
  include(${OTB_USE_FILE})
else()
  message(FATAL_ERROR
          "Cannot build HelloWorld application without OTB.  Please set OTB_DIR.")
endif()


OTB_CREATE_APPLICATION(NAME           ConnectedComponentSegmentation
                       SOURCES        otbConnectedComponentSegmentation.cxx
                       LINK_LIBRARIES OTBIO;OTBCommon;OTBBasicFilters)


OTB_CREATE_APPLICATION(NAME           Segmentation
                       SOURCES        otbSegmentation.cxx
                       LINK_LIBRARIES OTBBasicFilters;OTBOBIA;OTBOGRAdapters)
