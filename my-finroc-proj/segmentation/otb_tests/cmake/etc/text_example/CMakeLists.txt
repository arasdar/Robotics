cmake_minimum_required(VERSION 2.8)

project(FeatureExtractionExamples)
include_regular_expression("^.*$")

find_package(OTB)
if(OTB_FOUND)
  include(${OTB_USE_FILE})
else()
  message(FATAL_ERROR
          "Cannot build HelloWorld application without OTB.  Please set OTB_DIR.")
endif()

add_executable(TextureExample TextureExample.cxx)
target_link_libraries(TextureExample OTBIO OTBCommon OTBFeatureExtraction ${ITK_LIBRARIES}  )

add_executable(AdvancedTextureExample AdvancedTextureExample.cxx)
target_link_libraries(AdvancedTextureExample OTBIO OTBCommon OTBFeatureExtraction ${ITK_LIBRARIES}  )

add_executable(RunlengthTextureExample Runlength_Statistics.cxx)
target_link_libraries(RunlengthTextureExample OTBIO OTBCommon OTBFeatureExtraction ${ITK_LIBRARIES}  )

add_executable(PanTexExample PanTexExample.cxx)
target_link_libraries(PanTexExample OTBIO OTBCommon OTBFeatureExtraction ${ITK_LIBRARIES}  )

add_executable(SFSExample SFSExample.cxx)
target_link_libraries(SFSExample OTBIO OTBCommon OTBFeatureExtraction ${ITK_LIBRARIES}  )

