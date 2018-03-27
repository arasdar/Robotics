#include <iostream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/EigenValueDecomposition.hpp"
#include "projects/stereo_traversability_experiments/mlr/learn_EVD/helper.hpp"

#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

using namespace finroc::stereo_traversability_experiments::mlr::learn_PCA;

void tPCA::compute_EVD(const cv::Mat& I) // basically this calculates SVD using EVD which is normally done as usual -- u have to do it if u want to calculate PC normally using DOUBLE precision and good enough accuracy
{
  // Make sure I.type == DOUBLE
  if (I.type() == CV_64FC1)
  {

    // get sample size, dimension
    int n = I.rows; // number of feature dimensions
    int t = I.cols; // number of I samples or observations

    // holds the mean over all classes
    /*cv::Mat */this->mean = cv::Mat::zeros(n /*rows*/, 1 /*cols*/, I.type());

    // calculate sums
    for (int j = 0; j < t; j++)
    {
      //      cv::Mat instance = I.col(j);
      //      cv::add(mean, instance, mean);
      mean += I.col(j);
    }

    // calculate total mean
    //mean.convertTo(mean, mean.type(), 1.0 / static_cast<double>(t));  // c++ style
    mean.convertTo(mean, mean.type(), double(1.0) / double(t)); // c style
    //mlr.LOG_PRINT(mean, "mean (Mu)", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // subtract from every single sample
    cv::Mat Fi = cv::Mat::zeros(n, t, I.type()); //instantiate cv::Mat, create cv::Mat and initialize with zeros
    for (int j = 0; j < t; j++)
    {
      Fi.col(j) = I.col(j) - mean;
    }// for j
    //mlr.LOG_PRINT(Fi, "Fi", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // Square matrix or matrix of CoVariance or Scatter matrix or Area matrix (can also be somehow referred as Correlation or CoRelation matrix as well!!)
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
    //mlr.LOG_PRINT(A, "A", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // EigenValueDecomposition recults - not sorted
    finroc::stereo_traversability_experiments::mlr::EigenValueDecomposition evd(A);
    //mlr.LOG_PRINT(evd.eigenvalues(), "eigenvalues", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
    //mlr.LOG_PRINT(evd.eigenvectors(), "eigenvectors", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // instantiate, create and initialize with zero
    /*cv::Mat*/ this->S = cv::Mat::zeros(evd.eigenvalues().rows /* should be ONE*/,
                                         evd.eigenvalues().cols, evd.eigenvalues().type()); // V_1,....., V_k --> D_1,....,D_k
    for (int j = 0; j < this->S.cols; j++)
    {
      this->S.at<double>(j) = double(std::abs(std::sqrt(std::abs(evd.eigenvalues().at<double>(j)))));
    }
    //mlr.LOG_PRINT(S, "singular values", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // sort them in DESCENDING fashion
    // get sorted indices descending by their singular values
    vector<int> sorted_indices = finroc::stereo_traversability_experiments::mlr::learn_EVD::argsort(this->S, false /*this means DESCENDING*/);

    // now sort singular values and eigen vectors accordingly
    /*cv::Mat*/ this->S /*singular values 1xk*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(this->S, sorted_indices);
    //mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // These two are the cases for efficient EVD
    if (n <= t)   // Fi = {Fi_1,.............., Fi_t} , Fi_i has n Dimensions or variables ===> Fi_nxt
    {
      /*cv::Mat*/ V /*V_nxk for A_nxn*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
      //mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
    }// if n < t
    else // if t < n
    {
      /*cv::Mat*/ U /*U_txk for A_txt*/ = finroc::stereo_traversability_experiments::mlr::learn_EVD::sortMatrixColumnsByIndices(evd.eigenvectors(), sorted_indices);
      //mlr.LOG_PRINT(U, "U", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
    }// if t < n

    // keep only the PC with S >= 1
    // choosing the number of principle components to keep or ratio of them
    unsigned int num_components = 0;
    while (S.at<double>(num_components) >= 1.0)
    {
      num_components++;
    };
    S = cv::Mat(S, cv::Range::all(), cv::Range(0, num_components));
    //mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // SVD using EVD --> Fi * Fi_transposed = V * S * V_trans (EVD) ==> Fi = V * S * U_trans (SVD) ==> Fi * U * S_inversed = V
    // instantiate, create and initialize with zero
    this->S_diag_inv = cv::Mat::zeros(S.cols /*rows*/, S.cols /*columns*/, S.type()); // V_1,....., V_k --> S_1,....,S_k --> D_1x1,...., D_kxk
    for (int j = 0; j < S_diag_inv.cols; j++)
    {
      S_diag_inv.at<double>(j, j) = double(1.0) / S.at<double>(j); // in order to preserve the double precision while inversing
    }
    //mlr.LOG_PRINT(S_diag_inv, "S_diag_inv", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // These two are the cases for efficient EVD
    if (n <= t)   // Fi = {Fi_1,.............., Fi_t} , Fi_i has n Dimensions or variables ===> Fi_nxt
    {
      V = cv::Mat(V, cv::Range::all(), cv::Range(0, num_components));
      //mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

      // calculating U --> Fi = V . S . U_t --> U_t = S_inv. V_t . Fi  ==> this is OPTIONAL but is done in order to complete the process and com up with a unique and general answer
      cv::Mat U_tran = S_diag_inv * V.t() * Fi;
      U = U_tran.t();
    }// if n < t
    else // if t < n  --> MANDATORY and we didn't have any other OPTIONS --> this here leads to SVD conclusion at the end.
    {
      U = cv::Mat(U, cv::Range::all(), cv::Range(0, num_components));
      //mlr.LOG_PRINT(U, "U", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

      // calculating the V --->    Fi * U * S_inversed = V
      /*cv::Mat */ V = Fi * U * S_diag_inv; // since SVD looks like has DOUBLE precision base on learn_opencv_SVD exprience hier
      //mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
    }// if t < n
  }// I.type == CV_64C1

}// end main function
