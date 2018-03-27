#include <iostream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/helper.hpp"
#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

//#include "projects/stereo_traversability_experiments/mlr/learn_EVD/EigenValueDecomposition.hpp"
#include <Eigen/Dense> // eigen stuff
#include <opencv2/core/eigen.hpp> //opencv to eigen conversion

using namespace finroc::stereo_traversability_experiments::mlr::learn_PCA;

void tPCA::compute_eignelib_SVD(const cv::Mat& I)
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

    // opencv to eigen
    Eigen::MatrixXd m_Fi;
    cv::cv2eigen(Fi, m_Fi);

    /* Fi = V.S.U_t
    Parameters:
    Fi – decomposed matrix
    S – calculated singular values
    V – calculated left singular vectors
    U – calculated right singular vectors
    U_t – transposed matrix of right singular values
    */

    /*! FI = V . S . U_transposed */
    Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::FullPivHouseholderQRPreconditioner> svd(m_Fi, Eigen::ComputeFullU /*m_V*/ | Eigen:: ComputeFullV /*m_U*/); // very slow OK
    //Eigen::JacobiSVD <Eigen::MatrixXd> svd(m_Fi, Eigen::ComputeThinU /*m_V*/ | Eigen:: ComputeThinV /*m_U*/); // OK
    Eigen::MatrixXd m_V(svd.matrixU()), m_S/*_vector*/(svd.singularValues()), m_U/*_transposed*/(svd.matrixV()/*.transpose()*/);
    std::cout << "m_V.size(): " << m_V.size() << std::endl;
    std::cout << "m_S.size(): " << m_S.size() << std::endl;
    std::cout << "m_U.size(): " << m_U.size() << std::endl;

    // eigen to opencv conversion
    cv::eigen2cv(m_V, this->V);
    cv::eigen2cv(m_S, this->S);
    this->S = this->S.t();
    cv::eigen2cv(m_U, this->U); /*m_U and U are not gonna be usefull therefore we don't need them*/

    //    startttttttttttttttttttttttttttttt=============================================================================================
    //    S:
    //    Mat:::::::::::::::::::
    //    Mat size: (width, height) =  (cols, rows) =  (X, Y)=   (j, i) = (w, h) ------------->
    //    Mat size: (width, height) || (cols, rows) || (X, Y) || (j, i) ------------>
    //    size: [1 x 400]
    //    rows: 400  height, Y, i
    //    cols: 1  width, X, j
    //    type: 6
    //    depth: 6
    //    channels; 1
    //    elemSize: 8
    //    step1: 1
    //    isContinuous: 1
    //    total: 400
    //    endddddddddddddddddddddddd=============================================================================================
    //    startttttttttttttttttttttttttttttt=============================================================================================
    //    V:
    //    Mat:::::::::::::::::::
    //    Mat size: (width, height) =  (cols, rows) =  (X, Y)=   (j, i) = (w, h) ------------->
    //    Mat size: (width, height) || (cols, rows) || (X, Y) || (j, i) ------------>
    //    size: [400 x 10304]
    //    rows: 10304  height, Y, i
    //    cols: 400  width, X, j
    //    type: 6
    //    depth: 6
    //    channels; 1
    //    elemSize: 8
    //    step1: 400
    //    isContinuous: 1
    //    total: 4121600
    //    endddddddddddddddddddddddd=============================================================================================
    //    startttttttttttttttttttttttttttttt=============================================================================================
    //    U:
    //    Mat:::::::::::::::::::
    //    Mat size: (width, height) =  (cols, rows) =  (X, Y)=   (j, i) = (w, h) ------------->
    //    Mat size: (width, height) || (cols, rows) || (X, Y) || (j, i) ------------>
    //    size: [400 x 400]
    //    rows: 400  height, Y, i
    //    cols: 400  width, X, j
    //    type: 6
    //    depth: 6
    //    channels; 1
    //    elemSize: 8
    //    step1: 400
    //    isContinuous: 1
    //    total: 160000

  }// I.type == CV_64C1

}// end main function
