#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp> //cv2eigen and reverse

using namespace std;
using namespace Eigen;

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

int main(int argc /*argument counter*/, const char** argv /*argument variable*/)
{

  // the MLR class and this is PCA implementation of another kind
  finroc::stereo_traversability_experiments::mlr::tMLR mlr;

//  //  // (example taken from: http://www.bytefish.de/wiki/pca_lda_with_gnu_octave)
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
//  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone();
  cv::Mat I = cv::Mat(3 /*rows*/, 5 /*cols*/, CV_64FC1 /*DOUBLE precision*/, a).clone().t(); // this is transposed
  mlr.LOG_PRINT(I, "I", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << I << std::endl;


  // get sample size, dimension
  size_t n = I.rows; // number of feature dimensions
  size_t t = I.cols; // number of I samples or observations

  //    // holds the mean over all classes
  //    cv::Mat mean = cv::Mat::zeros(n /*rows*/, 1 /*cols*/, I.type());
  //
  //    // calculate sums
  //    for (int j = 0; j < t; j++)
  //    {
  //      cv::Mat instance = I.col(j);
  //      cv::add(mean, instance, mean);
  //    }
  //    // calculate total mean
  //    mean.convertTo(mean, mean.type(), 1.0 / static_cast<double>(t));
  //    mlr.LOG_PRINT(mean, "mean (Mu)", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  //    std::cout << mean << std::endl;
  //
  //    // subtract from every single sample
  //    cv::Mat Fi = cv::Mat::zeros(n, t, I.type());
  //    for (int j = 0; j < t; j++)
  //    {
  //      Fi.col(j) = I.col(j) - mean;
  //    }// for j
  //    mlr.LOG_PRINT(Fi, "Fi", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  //    std::cout << Fi << std::endl;
  //
  //  // eigen2cv
  //  MatrixXd m_FI;
  //  cv::cv2eigen(Fi, m_FI);

  // eigen2cv
  MatrixXd matrix_input_data;
  cv::cv2eigen(I, matrix_input_data);

  /*! 111111111111111111111111111111111111111111111111111111111111 FIRST
   * N*T = DataGeneratorFeatures resampling or bottom to top sampling data original feature space using
   * N feature values(Dimension) & T feature vectors (data samples)
   * */
  /*! mean=(I1+..............+It)/t & FI=I-mean={I1-mean,...................., It-mean}*/
  VectorXd vector_input_data_mean(n);
  vector_input_data_mean.setZero();
  for (unsigned int i = 0; i < t; ++i)
  {
    vector_input_data_mean += matrix_input_data.col(i);
  }
  vector_input_data_mean = vector_input_data_mean / t;
  cout << "vector_input_data_mean:\n" << vector_input_data_mean << endl;

  //FI= I-mean= {I1-mean,...................., It-mean}
  //original input samples
  MatrixXd m_FI(n, t);
  for (unsigned /*because the default is signed*/ int i = 0; i < t; ++i)
  {
    m_FI.col(i) << matrix_input_data.col(i) - vector_input_data_mean;
  }
  cout << "m_FI:\n" << m_FI << endl;

  /*! FI = V . S . U_transposed */
  Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::FullPivHouseholderQRPreconditioner> svd(m_FI, Eigen::ComputeFullU | Eigen:: ComputeFullV);
  MatrixXd m_V(svd.matrixU()), m_S_vector(svd.singularValues()), m_U_transposed(svd.matrixV().transpose());

  /*! normalizing singular values to use them as weights(percentage) of feature vectors in eigenfeatures space */
  cout       << "m_S_vector:\n" << m_S_vector << endl;
  MatrixXd m_S(m_S_vector.size() , m_S_vector.size());
  m_S.setZero();
  for (unsigned int i = 0; i < m_S_vector.size(); ++i)
  {
    m_S(i, i) = m_S_vector(i, 0);
  }
  cout        << "m_S:\n" << m_S << endl
              << "m_V:\n" << m_V << endl
              << "m_U_transposed:\n" << m_U_transposed << endl;

  return 0; // output of main function
}
