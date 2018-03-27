#include <iostream>

#include "projects/stereo_traversability_experiments/mlr/learn_EVD/EigenValueDecomposition.hpp"
#include "projects/stereo_traversability_experiments/mlr/learn_EVD/helper.hpp"

#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

using namespace finroc::stereo_traversability_experiments::mlr::learn_PCA;

void tPCA::recompute(const unsigned int& num_PCs)
{
  S = cv::Mat(S, cv::Range::all(), cv::Range(0, num_PCs));
  V = cv::Mat(V, cv::Range::all(), cv::Range(0, num_PCs));
  //U = cv::Mat(U, cv::Range::all(), cv::Range(0, num_PCs));
}// end main function

void tPCA::recompute(const double& ratio_PCs)
{
  unsigned int num_PCs = ratio_PCs * this->S.cols;
  S = cv::Mat(S, cv::Range::all(), cv::Range(0, num_PCs));
  V = cv::Mat(V, cv::Range::all(), cv::Range(0, num_PCs));
  //U = cv::Mat(U, cv::Range::all(), cv::Range(0, num_PCs));
}// end recompute()
