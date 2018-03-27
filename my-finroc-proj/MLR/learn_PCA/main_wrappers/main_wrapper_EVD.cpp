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

  double a[3][5] =
  {
    { 1.96 , -6.49, -0.47, -7.20, -0.65},
    { -6.49,  3.80, -6.39,  1.50, -6.34},
    { -0.47, -6.39,  4.17, -1.51,  2.67}
  };

  // convert into OpenCV representation
//  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone();
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

  // Square matrix or matrix of CoVariance
  cv::Mat A;

  // These two are the cases for efficient EVD
  if (n <= t)   // Fi = {Fi_1,.............., Fi_t} , Fi_i has n Dimensions or variables ===> Fi_nxt
  {
    A = Fi * Fi.t(); // A_nxn
  }// if n < t
  else // if t < n
  {
    A = Fi.t() * Fi; //A_txt
  }// if t < n

  mlr.LOG_PRINT(A, "A", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << A << std::endl;
  // EigenValueDecomposition recults - not sorted
  finroc::stereo_traversability_experiments::mlr::EigenValueDecomposition evd(A);
  mlr.LOG_PRINT(evd.eigenvalues(), "eigenvalues", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << evd.eigenvalues() << std::endl;
  mlr.LOG_PRINT(evd.eigenvectors(), "eigenvectors", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << evd.eigenvectors() << std::endl;

  // instantiate, create and initialize with zero
  cv::Mat singularvalues = cv::Mat::zeros(evd.eigenvalues().rows /* should be ONE*/, evd.eigenvalues().cols, evd.eigenvalues().type()); // V_1,....., V_k --> D_1,....,D_k
  for (int j = 0; j < singularvalues.cols; j++)
  {
    singularvalues.at<double>(j) = std::abs(std::sqrt(std::abs(evd.eigenvalues().at<double>(j))));
  }
  mlr.LOG_PRINT(singularvalues, "singular values", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << singularvalues << std::endl;

  // sort them in DESCENDING fashion
  // get sorted indices descending by their singular values
  vector<int> sorted_indices = finroc::stereo_traversability_experiments::mlr::learn_EVD::argsort(singularvalues, false /*this means DESCENDING*/);
  // now sort singular values and eigen vectors accordingly
  cv::Mat S /*singular values 1xk*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(singularvalues, sorted_indices);
  mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << S << std::endl;
  cv::Mat V, U;
  // These two are the cases for efficient EVD
  if (n <= t)   // Fi = {Fi_1,.............., Fi_t} , Fi_i has n Dimensions or variables ===> Fi_nxt
  {
    //A = Fi * Fi.t(); // A_nxn
    /*cv::Mat*/ V /*V_nxk for A_nxn*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
    mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << V << std::endl;
  }// if n < t
  else // if t < n
  {
    //A = Fi.t() * Fi; //A_txt
    /*cv::Mat*/ U /*U_txk for A_txt*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
    mlr.LOG_PRINT(U, "U", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << U << std::endl;
  }// if t < n

  // keep only the PC with S >= 1
  // choosing the number of principle components to keep or ratio of them
  unsigned int num_components = 0;
  while (S.at<double>(num_components) >= 1.0)
  {
    num_components++;
  };
  S = cv::Mat(S, cv::Range::all(), cv::Range(0, num_components));
  mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << S << std::endl;
  // These two are the cases for efficient EVD
  if (n <= t)   // Fi = {Fi_1,.............., Fi_t} , Fi_i has n Dimensions or variables ===> Fi_nxt
  {
    V = cv::Mat(V, cv::Range::all(), cv::Range(0, num_components));
    mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << V << std::endl;
  }// if n < t
  else // if t < n
  {
    U = cv::Mat(U, cv::Range::all(), cv::Range(0, num_components));
    mlr.LOG_PRINT(U, "U", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << U << std::endl;

    //    // SVD using EVD
    //    // Fi * Fi_transposed = V * S * V_trans (EVD) ==> Fi = V * S * U_trans (SVD) ==> Fi * U * S_inversed = V (eigenvectors)
    // we should calculate the V and for this calculation, we need inversed version of diagonal singular values or D_inversed
    // instantiate, create and initialize with zero
    cv::Mat S_diag_inv /*Diagonal singular values or Diagonal version of S */ =
      cv::Mat::zeros(S.cols /*rows*/, S.cols /*columns*/, S.type()); // V_1,....., V_k --> S_1,....,S_k --> D_1x1,...., D_kxk
    for (int j = 0; j < S_diag_inv.cols; j++)
    {
      S_diag_inv.at<double>(j, j) = double(1) / double(S.at<double>(j)); // in order to preserve the double precision while inversing
    }
    mlr.LOG_PRINT(S_diag_inv, "S_diag_inv", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << S_diag_inv << std::endl;

    // calculating the V --->    Fi * U * S_inversed = V
    cv::Mat V = Fi * U * S_diag_inv; // since SVD looks like has DOUBLE precision base on learn_opencv_SVD exprience hier
    mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    std::cout << V << std::endl;

  }// if t < n

  return 0;
}// end main function
