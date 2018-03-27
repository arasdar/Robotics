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
#include "projects/stereo_traversability_experiments/mlr/learn_EVD/helper.hpp"
#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

int main(int argc, const char *argv[])
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
  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone();
//  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone().t(); // this is transposed
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

  // Square matrix or matrix of CoVariance
  cv::Mat A;

  // d = min(n, t) --> if d == n else d == t
  if (n <= t)   // Fi = {Fi_1,.............., Fi_t} , Fi_i has n Dimensions or variables ===> Fi_nxt
  {
    //cv::calcCovarMatrix(Fi, covar, mean)
    /*cv::Mat*/ A = Fi * Fi.t(); // A_nxn

//    // sorting the results
//    // get sorted indices descending by their eigenvalue
//    vector<int> sorted_indices = finroc::stereo_traversability_experiments::mlr::learn_EVD::argsort(evd.eigenvalues(), false);
//    // now sort eigenvalues and eigenvectors accordingly
//    cv::Mat D /*Diagnol*/ /*eigenvalues kxk*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvalues(), sorted_indices);
//    cv::Mat V /*eigenvectors nxk for A_nxn*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
//    // and now take only the num_components and we're out!
//
//    // choosing the number of principle components to keep or ratio of them
//    unsigned int num_components = A.rows; // A_nxn
//    D = cv::Mat(D, cv::Range::all(), cv::Range(0, num_components));
//    V = cv::Mat(V, cv::Range::all(), cv::Range(0, num_components));
//    mlr.LOG_PRINT(D, "eigenvalues sorted", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << D << std::endl;
//    mlr.LOG_PRINT(V, "eigenvectors sorted", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << V << std::endl;
  }// if n < t
  else // t < n
  {
    /*cv::Mat*/ A = Fi.t() * Fi; //A_txt

//    // sorting the results
//    // get sorted indices descending by their eigenvalue
//    vector<int> sorted_indices = finroc::stereo_traversability_experiments::mlr::learn_EVD::argsort(evd.eigenvalues(), false);
//    // now sort eigenvalues and eigenvectors accordingly
//    cv::Mat D /*Diagnol*/ /*eigenvalues kxk*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvalues(), sorted_indices);
//    cv::Mat U /*eigenvectors nxk for A_nxn*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
//
//    // keeping only non-zero elements -- based on the eigenvalues  -- eigenvalues should be always positive when they are sorted (descending) - scales and scalar
//    // choosing the number of principle components to keep or ratio of them
//    unsigned int num_components = 0;
//    while (D.at<double>(num_components) > 0.0)
//    {
//      num_components++;
//    };
//    D = cv::Mat(D, cv::Range::all(), cv::Range(0, num_components));
//    U = cv::Mat(U, cv::Range::all(), cv::Range(0, num_components));
//    mlr.LOG_PRINT(D, "D sorted", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << D << std::endl;
//    mlr.LOG_PRINT(U, "U sorted", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << U << std::endl;
//
//    // S == Sigma matrix for Singular Values  --> required to calculate V_nxk  --> k <= min(n,t) and in this case t
//    cv::Mat S = cv::Mat::zeros(D.cols, D.cols, D.type());
//
//    // we have to assign elements by element since singular values are the SQUARE ROOT of eigenvalues
//    for (int j = 0; j < D.cols; j++)
//    {
//      S.at<double>(j, j) = std::sqrt(std::abs(D.at<double>(j)));
//    }
//    mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << S << std::endl;
//    cv::Mat S_inversed = S.inv();
//    mlr.LOG_PRINT(S_inversed, "S_inersed", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << S_inversed << std::endl;
//
//    // SVD using EVD
//    // Fi * Fi_transposed = V * S * V_trans (EVD) ==> Fi = V * S * U_trans (SVD) ==> Fi * U * S_inversed = V (eigenvectors)
//    cv::Mat V = Fi * U * S_inversed; // or Fi * U / abs(Fi)
//    mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
//    std::cout << V << std::endl;
  }// if t < n

  mlr.LOG_PRINT(A, "A", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << A << std::endl;
  // EigenValueDecomposition recults - not sorted
  finroc::stereo_traversability_experiments::mlr::EigenValueDecomposition evd(A);
  mlr.LOG_PRINT(evd.eigenvalues(), "eigenvalues", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << evd.eigenvalues() << std::endl;
  mlr.LOG_PRINT(evd.eigenvectors(), "eigenvectors", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << evd.eigenvectors() << std::endl;


  return 0;
}// end main function




//octave-matlab_eigen.m
//%disp("Hello World");
//%% EigenValueDecomposition
//I = [2 3;3 4;4 5;5 6;5 7;2 1;3 2;4 2;4 3;6 4;7 6];
//mu = mean(I)
//Fi = bsxfun(@minus, I, mu)
//
//%%%%%%%%%%%%%%%%% EVD
//%C = cov(Xm)
//A = Fi'*Fi
//[V,D] = eig(A)
//
//%%%%%%%%%%%%%%%%% SVD
//Fi = bsxfun(@minus, I, mu)
//[V,D,U] = svd(Fi)
//octave:1> source("EVD.m")
//V =
//
//  -0.79801   0.60265
//   0.60265   0.79801
//
//D =
//
//Diagonal Matrix
//
//    8.9813         0
//         0   52.8368



//===========================================
//the EigenvalueSolver in JAMA
//(http://math.nist.gov/javanumerics/jama),
//This software is a cooperative product of The MathWorks (MATLAB) and
//the National Institute of Standards and Technology (NIST)
// (co-product of NIST & MATLAB) and ported by www.bytefish.de (Philip Wagner)
//EVD.cpp
//eigenvalues
//[8.981335750821934, 52.83684606735989]
//
//eigenvectors
//[-0.7980055828881055, -0.602650055736673;
//  0.602650055736673, -0.7980055828881055]


//===========================================
//Eigen library jacoianSVD
//SVD.cpp
//m_S_vector:
//7.2689 ===> 52.83690721
//2.99689 ===> 8.981349672
//m_S:
// 7.2689       0
//      0 2.99689
//m_V:
// -0.60265 -0.798006
//-0.798006   0.60265



//=========================================
//OpenCV PCA class
//I
//[2, 3, 4, 5, 5, 2, 3, 4, 4, 6, 7;
//  3, 4, 5, 6, 7, 1, 2, 2, 3, 4, 6]
//pca.mean
//[4.090909090909091; 3.909090909090909]
//pca.eigenvalues
//[4.803349642487262; 0.8164850682565392]
//pca.eigenvectors
//[0.6026500557366729, 0.7980055828881054;
//  0.7980055828881054, -0.6026500557366729]

//========================================
//OpenCV SVD class
//S
//[7.268895794228988; 2.996887677378305]
//V
//[0.6026500557366726, -0.7980055828881056;
//  0.7980055828881056, 0.6026500557366726]

