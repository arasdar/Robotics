cmake_minimum_required(VERSION 2.8)

project(FeatureExtractionExamples)

find_package(OTB)
include(${OTB_USE_FILE})

add_executable(TextureExample test.cpp)
target_link_libraries(TextureExample OTBIO OTBCommon OTBFeatureExtraction ${ITK_LIBRARIES}  )