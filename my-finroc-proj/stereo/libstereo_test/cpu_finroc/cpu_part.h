/*
 * cpu_part.h
 *
 *  Created on: Oct 9, 2014
 *      Author: aras
 */

#ifndef CPU_PART_H_
#define CPU_PART_H_


#include <stdio.h>

#include <opencv2/opencv.hpp>

//#include "projects/stereo_traversability_experiments/aras/libstereo_test/utils.h"

using namespace cv;
using namespace std;

#define PI 3.141592653589793238463

cv::Mat imageInputGray_1;
cv::Mat imageInputGray_2;
cv::Mat imageOutputGray_1;
cv::Mat imageOutputGray_2;

int numRows;
int numCols;

float *h_filter__;

/************reference calculation2***************/
void CPU_referenceCalculation(const uchar* const grayImage_1, const uchar* const grayImage_2, uchar *const outputImage_1,
                              uchar *const outputImage_2, int numRows, int numCols,
                              const float* const filter, const int filterWidth);

/************CPU_postprocess*******/
void CPU_postProcess(const std::string& output_file, uchar* data_ptr);

/************preprocess**********/
void CPU_Preprocess(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2,
                    uchar **h_outputImageGray_1, uchar **h_outputImageGray_2,
                    float **h_filter, int *filterWidth, const std::string &filename_1, const std::string &filename_2);

/**********cleanUp**********/
void CPU_cleanUp(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2, uchar **h_outputImageGray_1, uchar **h_outputImageGray_2);

//***********************
void CPU_Part(int argc, char **argv);

#endif /* CPU_PART_H_ */
