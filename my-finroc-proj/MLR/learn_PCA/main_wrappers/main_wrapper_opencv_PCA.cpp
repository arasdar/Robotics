//#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

int main(int argc /*argument counter*/, const char** argv /*argument variable*/)
{

  // the MLR class and this is PCA implementation of another kind
  finroc::stereo_traversability_experiments::mlr::tMLR mlr;

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

//  double a[5][5] =
//  {
//    { 1.96 , -6.49, -0.47, -7.20, -0.65},
//    { -6.49,  3.80, -6.39,  1.50, -6.34},
//    { -0.47, -6.39,  4.17, -1.51,  2.67},
//    { -7.20,  1.50, -1.51,  5.70,  1.80},
//    { -0.65, -6.34,  2.67,  1.80, -7.10}
//  };
//
//  // convert into OpenCV representation
//  cv::Mat I = cv::Mat(5 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone()/*.t()*/; // this is transposed
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
//  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone(); // this is transposed
  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone().t(); // this is transposed
  mlr.LOG_PRINT(I, "I", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << I << std::endl;

  // perform PCA
  //int maxComponents(0); //– maximum number of components that PCA should retain; by default, all the components are retained.
  double retainedVariance(1.00); /*meaning all*/ /*these two are the same*/
  cv::PCA pca(I, cv::Mat(), CV_PCA_DATA_AS_COL, retainedVariance);
  mlr.LOG_PRINT(pca.mean, "pca.mean", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << pca.mean << std::endl;
  mlr.LOG_PRINT(pca.eigenvalues, "pca.eigenvalues", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << pca.eigenvalues << std::endl;
  mlr.LOG_PRINT(pca.eigenvectors, "pca.eigenvectors", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << pca.eigenvectors << std::endl;

  return 0;
}// end main

