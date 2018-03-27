/*
 * gpu_part.h
 *
 *  Created on: Oct 10, 2014
 *      Author: aras
 */

//Udacity HW2 Driver

#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "projects/stereo_traversability_experiments/aras/libstereo_test/timer.h"
#include "projects/stereo_traversability_experiments/aras/libstereo_test/utils.h"

using namespace cv;
using namespace std;

#define PI 3.141592653589793238463

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace aras
{
namespace libstereo_test
{
namespace gpu
{

class gpu_part
{

public:
  gpu_part();
  ~gpu_part();

public:
  cv::Mat G_imageInputGray_1;
  cv::Mat G_imageInputGray_2;
  cv::Mat G_imageOutputGray;

  uchar *d_inputImageGray_1__;
  uchar *d_inputImageGray_2__;
  uchar *d_outputImageGray__;//_1__;

  float *GPU_gw_average_color_1_;
  float *GPU_gw_auto_correlation_1_;
  float *GPU_gw_average_color_2_;
  float *GPU_gw_auto_correlation_2_;
  float *GPU_gw_cross_correlation_3_;
  float *GPU_gw_normalized_score_4_;

  int *match_matrix_;
  int *disparity_map_;
  int *depth_map_;

  int numRows;
  int numCols;

  float *h_filter__;

  /*******  DEFINED IN kernel_func.cu *********/
  void your_gaussian_blur(const uchar* const h_inputImageGray_1,
                          uchar* const d_inputImageGray_1,
                          uchar* const h_inputImageGray_2,
                          uchar* const d_inputImageGray_2,
                          uchar* const d_outputImageGray,
                          float* const GPU_gw_average_color_1,
                          float* const GPU_gw_auto_correlation_1,
                          float* const GPU_gw_average_color_2,
                          float* const GPU_gw_auto_correlation_2,
                          float* const GPU_gw_cross_correlation_3,
                          float* const GPU_gw_normalized_score_4,
                          int* match_matrix,
                          int* disparity_map,
                          int* depth_map,
                          const int numRows, const int numCols,
                          const int filterWidth);

  void allocateMemoryAndCopyToGPU(const int numRows, const int numCols,
                                  const float* const h_filter, const int filterWidth);

  /************postprocess*******/
  void GPU_postProcess(const std::string& output_file, uchar* data_ptr);


  //************************************
  void GPU_preProcess(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2, uchar **h_outputImageGray,
                      uchar **d_inputImageGray_1, uchar **d_inputImageGray_2, uchar **d_outputImageGray,
                      float **GPU_gw_average_color_1, float **GPU_gw_auto_correlation_1,
                      float **GPU_gw_average_color_2, float **GPU_gw_auto_correlation_2,
                      float **GPU_gw_cross_correlation_3, float **GPU_gw_normalized_score_4,
                      int **match_matrix, int **disparity_map, int **depth_map,
                      //unsigned char **d_redBlurred,
                      //unsigned char **d_redBlurred_1,
                      //unsigned char **d_greenBlurred,
                      //unsigned char **d_greenBlurred_1,
                      //unsigned char **d_blueBlurred,
                      //unsigned char **d_blueBlurred_1,
                      float **h_filter, int *filterWidth,
                      const std::string &filename_1, const std::string &filename_2);

  /**********cleanUp**********/
  void GPU_cleanUp(void);

  //***********************
  void GPU_run(int argc, char **argv);

};

}
}
}
}
}
