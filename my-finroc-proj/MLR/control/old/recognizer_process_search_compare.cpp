//#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

#include "projects/stereo_traversability_experiments/MLR/control/mRecognition.h"
using namespace finroc::stereo_traversability_experiments::control;

using namespace cv;

///////////////////////
void mRecognition::recognizer_compare(/*INPUT*/const Mat& I_vec_projected,
    /*INPUT*/const Mat& I_new_vec_projected,
    /*OUTPUT*/ std::vector<double>& comparison_results)
{

  // compare them based on difference in distance with scaling -- should be a value at the end
  Mat diff = I_vec_projected - I_new_vec_projected;
  Mat diff_scaled(diff.size(), diff.type()); // scaled with float
  Mat sim(diff.size(), diff.type());
  Mat sim_scaled(diff.size(), diff.type());

  double I_vec_projected_norm = norm(I_vec_projected, NORM_L2); // this is should be MIN
  double I_new_vec_projected_norm = norm(I_new_vec_projected, NORM_L2);

  double sim_cosine(0.0);
  double sim_scaled_cosine(0.0);

  // scale every Weight using S
  for (/*unsigned*/ int i = 0; i < diff.rows; i++) // rows, i, Y, height
  {

    //diff_scaled.row(i) = diff.row(i) / S.row(i);
    //diff_scaled.at<double>(i) = diff.at<double>(i) / double(std::pow(S.at<double>(i), 2.0));
    diff_scaled.at<double>(i) = diff.at<double>(i) / pca->S.at<double>(i);


    //    sim.row(i) = (I_vec_projected.row(i) / I_vec_projected_norm) /*--> Fi / |Fi| normalization*/
    //                 *  /*this is basically DOT product*/
    //                 (I_new_vec_projected.row(i) / I_new_vec_projected_norm); /*--> Fi_new / |Finew| normalization*/
    ////    sim_cosine += sim.at<double>(i); //.row(i);
    //    sim_cosine += sim.at<float>(i); //.row(i);

    sim.at<double>(i) = (I_vec_projected.at<double>(i) / I_vec_projected_norm) *
                        (I_new_vec_projected.at<double>(i) / I_new_vec_projected_norm);
    sim_cosine += sim.at<double>(i);

    //    sim_scaled.row(i) = (I_vec_projected.row(i) * I_new_vec_projected.row(i)) / S.row(i);
    ////    sim_scaled_cosine += sim_scaled.at<double>(i); //.row(i);
    //    sim_scaled_cosine += sim_scaled.at<float>(i); //.row(i);
    // scaling the MCC
    //sim_scaled.at<double>(i) = (I_vec_projected.at<double>(i) * I_new_vec_projected.at<double>(i)) / double(std::pow(S.at<double>(i), 2.0));
    sim_scaled.at<double>(i) = (I_vec_projected.at<double>(i) / pca->S.at<double>(i)) *
                               (I_new_vec_projected.at<double>(i) / pca->S.at<double>(i));
    sim_scaled_cosine += sim_scaled.at<double>(i);

  }// for diff rows iterating

  //  const unsigned int norm_type = NORM_L1; // SUMMATION OF ABSOLUTE // like euclidean without SQRT or square root
  //  double diff_norm = norm(diff, norm_type); // this should be MIN
  //  double diff_scaled_norm = norm(diff_scaled, norm_type); // this should be MIN
  //  double sim_norm = norm(sim, norm_type);
  //  double sim_scaled_norm = cv::norm(sim_scaled, norm_type);
  double diff_norm = norm(diff, NORM_L2); // this should be MIN
  double diff_scaled_norm = norm(diff_scaled, NORM_L2); // this should be MIN

  const unsigned int /*size_t*/ cNUMBER_OF_MTRICS(4);
  comparison_results.resize(cNUMBER_OF_MTRICS);
  comparison_results[0] = diff_norm;
  comparison_results[1] =  diff_scaled_norm;
  comparison_results[2] = sim_cosine;
  comparison_results[3] = sim_scaled_cosine;
}// end of compare
