#include <iostream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/EigenValueDecomposition.hpp"
#include "projects/stereo_traversability_experiments/mlr/learn_EVD/helper.hpp"

#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

using namespace finroc::stereo_traversability_experiments::mlr::learn_PCA;

void tPCA::compute_opencv_PCA(const cv::Mat& I)
{
  // Make sure I.type == DOUBLE
  if (I.type() == CV_64FC1)
  {
    // Also this is basically designed for FLOAT precision CV32FC1 and now I am passing DOUBLE precision data vectors or CV64FC1
    cv::PCA pca(I, cv::Mat() /*this means calculate mean too*/, CV_PCA_DATA_AS_COL, 1.0 /*retainedVariance or number of PCs to keep == all*/);
    this->mean = pca.mean;
    //    size: [1 x 10304]
    //    rows: 10304  height, Y, i
    //    cols: 1  width, X, j

    this->V = pca.eigenvectors.t();
    //  pca.eigenvectors
    //    size: [10304 x 400]
    //    rows: 400  height, Y, i
    //    cols: 10304  width, X, j

    cv::Mat eigenvalues = pca.eigenvalues.t();
    //  pca.eigenvalues
    //    size: [1 x 400]
    //    rows: 400  height, Y, i
    //    cols: 1  width, X, j

    // instantiate, create and initialize with zero
    /*cv::Mat*/ this->S = cv::Mat::zeros(eigenvalues.rows /* should be ONE*/,
                                         eigenvalues.cols, eigenvalues.type()); // V_1,....., V_k --> D_1,....,D_k
    for (int j = 0; j < this->S.cols; j++)
    {
      this->S.at<double>(j) = double(std::abs(std::sqrt(std::abs(eigenvalues.at<double>(j)))));
    }

  }// I.type == CV_64C1

}// end main function
