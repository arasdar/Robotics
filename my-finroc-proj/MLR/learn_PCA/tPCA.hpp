#ifndef __tPCA_HPP__
#define __tPCA_HPP__

#include <opencv2/opencv.hpp>

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace mlr
{
namespace learn_PCA
{

//! Performs PCA using the implemented EVD by Philip Wagner(bytefish.de) using JAMA (NIST && MathWorks co-product)
class tPCA
{

//private: /*EMPTY*/
//protected: /*EMPTY*/
public:
  cv::Mat V, U; // left and right singular vectors corresponding to singular values >= 1
  cv::Mat S; // singular values >= 1
  cv::Mat S_diag_inv; /*Diagonal singular values or Diagonal version of S */
  cv::Mat mean;

  //! initialize with 0 components and data given in rows
  tPCA() {};
  ~tPCA() {};

  void compute_EVD(const cv::Mat& I); // compute V, S using implemented EVD
  void compute_opencv_PCA(const cv::Mat& I); // using OpenCV SVD
  void compute_opencv_SVD(const cv::Mat& I); // using OpenCV SVD
  void compute_eignelib_SVD(const cv::Mat& I); // using Eigen library and JacobianSVD -- proved to be excellent


  // recompute to keep certain ration or number of PCs (Principle Components)
  void recompute(const unsigned int& num_PCs); // constant number of PCs
  void recompute(const double& ratio_PCs); // a ratio or percentage of PCs
  cv::Mat project(const cv::Mat& I_vec)
  {

    cv::Mat I_vec_proj = V.t() * (I_vec - mean);  // GEMM and this (difference???)
    return I_vec_proj;
  }
};

}
}
}
}
#endif
