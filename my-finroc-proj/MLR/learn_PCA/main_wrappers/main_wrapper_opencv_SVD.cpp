/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/EigenValueDecomposition.hpp"
#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

int main(int argc, const char *argv[])
{

  // The complete eigen analysis algorithm from input data
  // the MLR class and this is PCA implementation of another kind
  finroc::stereo_traversability_experiments::mlr::tMLR mlr;
//
//  // (example taken from: http://www.bytefish.de/wiki/pca_lda_with_gnu_octave)
//  double d[11][2] =
//  {
//    {2, 3},
//    {3, 4},
//    {4, 5},
//    {5, 6},
//    {5, 7},
//    {2, 1},
//    {3, 2},
//    {4, 2},
//    {4, 3},
//    {6, 4},
//    {7, 6}
//  };
//
//  // convert into OpenCV representation
//  cv::Mat I = cv::Mat(11 /*rows*/, 2 /*cols*/, CV_64FC1 /*DOUBLE precision*/, d).clone()/*.t()*/; // this is transposed
//  mlr.LOG_PRINT(I, "I", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//  std::cout << I << std::endl;

  double a[3][5] =
  {
    { 1.96 , -6.49, -0.47, -7.20, -0.65},
    { -6.49,  3.80, -6.39,  1.50, -6.34},
    { -0.47, -6.39,  4.17, -1.51,  2.67}//,
//    { -7.20,  1.50, -1.51,  5.70,  1.80},
//    { -0.65, -6.34,  2.67,  1.80, -7.10}
  };

  // convert into OpenCV representation
  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone().t(); // this is transposed
  mlr.LOG_PRINT(I, "I", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << I << std::endl;

  // get sample size, dimension
  int n = I.rows; // number of feature dimensions
  int t = I.cols; // number of I samples or observations

  // holds the mean over all classes
  cv::Mat mean = cv::Mat::zeros(n /*rows*/, 1 /*cols*/, I.type());

  // calculate sums
  for (int j = 0; j < t; j++)
  {
    cv::Mat instance = I.col(j);
    cv::add(mean, instance, mean);
  }
  // calculate total mean
  mean.convertTo(mean, mean.type(), 1.0 / static_cast<double>(t));
  mlr.LOG_PRINT(mean, "mean (Mu)", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << mean << std::endl;

  // subtract from every single sample
  cv::Mat Fi = cv::Mat::zeros(n, t, I.type());
  for (int j = 0; j < t; j++)
  {
    Fi.col(j) = I.col(j) - mean;
  }// for j
  mlr.LOG_PRINT(Fi, "Fi", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << Fi << std::endl;

  /* Fi = V.S.U_t
  Parameters:
  Fi – decomposed matrix
  S – calculated singular values
  V – calculated left singular vectors
  U – calculated right singular vectors
  U_t – transposed matrix of right singular values
  flags – operation flags:
  SVD::MODIFY_A use the algorithm to modify the decomposed matrix; it can save space and speed up processing.
  SVD::NO_UV indicates that only a vector of singular values w is to be processed, while u and vt will be set to empty matrices.
  SVD::FULL_UV when the matrix is not square, by default the algorithm produces u and vt matrices of sufficiently large size for the further A reconstruction;
  if, however, FULL_UV flag is specified, u and vt will be full-size square orthogonal matrices.
  */

  cv::Mat S, V, U_trans;
  cv::SVD::compute(Fi, S, V, U_trans, cv::SVD::FULL_UV);
  mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << S << std::endl;
  mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << V << std::endl;
  mlr.LOG_PRINT(U_trans, "U_trans", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << U_trans << std::endl;

  return 0;
}// end main function
