cmake_minimum_required(VERSION 2.6)
project(Haralick_App)

find_package(OTB)
if(OTB_FOUND)
  include(${OTB_USE_FILE})
else()
  message(FATAL_ERROR
          "Cannot build HelloWorld application without OTB.  Please set OTB_DIR.")
endif()


OTB_CREATE_APPLICATION(NAME Haralickbla
                       SOURCES otbHaralickTextureExtraction.cxx
                       LINK_LIBRARIES OTBIO;OTBCommon;OTBBasicFilters)
#add_executable(extract_haralick_app otbHaralickTextureExtraction.cxx )

#target_link_libraries(extract_haralick_app OTBIO OTBCommon OTBBasicFilters)
