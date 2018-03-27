#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/EigenValueDecomposition.hpp"
#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

int main(int argc, const char *argv[])
{

  ////////// EVD for decomposition accuracy with the same input for all classes and functions
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
    cv::Mat Fi = cv::Mat(5 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone(); // this is transposed
    finroc::stereo_traversability_experiments::mlr::tMLR mlr;
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
  }




  return 0;
}// end main function
