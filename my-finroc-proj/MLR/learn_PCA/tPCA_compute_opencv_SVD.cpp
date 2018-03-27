#include <iostream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/helper.hpp"

#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

using namespace finroc::stereo_traversability_experiments::mlr::learn_PCA;

void tPCA::compute_opencv_SVD(const cv::Mat& I)
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
    cv::SVD::compute(Fi, S, V, U_trans, cv::SVD::FULL_UV); // very slow OK
//    cv::SVD::compute(Fi, S, V, U_trans, cv::SVD::MODIFY_A); // fast OK
    this->S = S.t();
    this->V = V;
    this->U = U_trans.t();
    //    mlr.LOG_PRINT(S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    //    std::cout << S << std::endl;
    //    mlr.LOG_PRINT(V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    //    std::cout << V << std::endl;
    //    mlr.LOG_PRINT(U_trans, "U_trans", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
    //    std::cout << U_trans << std::endl;
  }// I.type == CV_64C1

}// end main function
