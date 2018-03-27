#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::compute_mean(/*INPUT*/const cv::Mat& I, /*OUTPUT*/cv::Mat& mean)
{
  // Make sure I.type == DOUBLE
  if (I.type() == CV_64FC1)
  {

    // get sample size, dimension
    int n = I.rows; // number of feature dimensions
    int t = I.cols; // number of I samples or observations

    // holds the mean over all classes
    mean = cv::Mat::zeros(n /*rows*/, 1 /*cols*/, I.type());

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
  }
}// end main function
