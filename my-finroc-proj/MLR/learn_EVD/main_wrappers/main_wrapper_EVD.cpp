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
  // the MLR class and this is PCA implementation of another kind
  finroc::stereo_traversability_experiments::mlr::tMLR mlr;
  {
    double a[5][5] =
    {
      { 1.96 , -6.49, -0.47, -7.20, -0.65},
      { -6.49,  3.80, -6.39,  1.50, -6.34},
      { -0.47, -6.39,  4.17, -1.51,  2.67},
      { -7.20,  1.50, -1.51,  5.70,  1.80},
      { -0.65, -6.34,  2.67,  1.80, -7.10}
    };
    // convert into OpenCV representation
    cv::Mat A = cv::Mat(5 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone(); // this is transposed
    mlr.LOG_PRINT(A, "A", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << A << std::endl;

    // EigenValueDecomposition recults - not sorted
    finroc::stereo_traversability_experiments::mlr::EigenValueDecomposition evd(A);
    mlr.LOG_PRINT(evd.eigenvalues(), "eigenvalues", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << evd.eigenvalues() << std::endl;
    mlr.LOG_PRINT(evd.eigenvectors(), "eigenvectors", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << evd.eigenvectors() << std::endl;

    //    // sorting the results
    //    // get sorted indices descending by their eigenvalue
    //    vector<int> sorted_indices = finroc::stereo_traversability_experiments::mlr::argsort(evd.eigenvalues(), false);
    //    // now sort eigenvalues and eigenvectors accordingly
    //    cv::Mat eigenvalues = finroc::stereo_traversability_experiments::mlr::sortMatrixColumnsByIndices(evd.eigenvalues(), sorted_indices);
    //    cv::Mat eigenvectors = finroc::stereo_traversability_experiments::mlr::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
    //    // and now take only the num_components and we're out!
    //    unsigned int num_components = A.rows;
    //    eigenvalues = cv::Mat(eigenvalues, cv::Range::all(), cv::Range(0, num_components));
    //    eigenvectors = cv::Mat(eigenvectors, cv::Range::all(), cv::Range(0, num_components));
    //    mlr.LOG_PRINT(eigenvalues, "eigenvalues sorted", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    //    std::cout << eigenvalues << std::endl;
    //    mlr.LOG_PRINT(eigenvectors, "eigenvectors sorted", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    //    std::cout << eigenvectors << std::endl;

  }

  return 0;
}// end main function
