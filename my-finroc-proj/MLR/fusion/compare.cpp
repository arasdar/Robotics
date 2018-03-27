#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

//hierarchy
//I vs I_new
//I-mean vbs I_new -mean
//V(I-mean) vs V(I_new -mean)
//S_inv.V.(I-mean) vs S_inv.V.(I_new-mean)

// This comparison function has been tested offline or can be tested offline first and then apply to online intelligent control
void tMLR::compare(/*INPUT*/const Mat& I, /*INPUT*/const Mat& I_new, /*INPUT*/const cv::Mat& mean,
                            /*OUTPUT*/std::vector<double>& vec_diff_norms/*comparison_results*/, /*OUTPUT*/std::vector<double>& vec_sim_values/*comparison_results*/)
{

  // compare them using DISTANCE function min DIFF and max SIM
  // Distance function and Similarity function using metrics
  double I_norm = norm(I, NORM_L2); // NORM_L2 == sqrt(sum(X^2))
  double I_new_norm = norm(I_new, NORM_L2);
  cv::Mat diff = I - I_new; // Min Square Diff or Euclidean
  cv::Mat sim = (I.t() / I_norm) * (I_new / I_new_norm); //      // similarity measurement (Cross Correlation)
  // make sure this Mat has only one member or index
  LOG_PRINT(sim, "sim", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  double diff_norm = norm(diff, NORM_L2); // this should be MIN
  double sim_value = sim.at<double>(0);
  //  vec_diff_norms.push_back(diff_norm);
  //  vec_sim_values.push_back(sim_value);

  // After applying mean and bringing them to the ORIGIN
  // The rest after mean, V
  cv::Mat Fi/*I_Mu*/ = (I - /*pca.*/mean);
  cv::Mat Fi_new/*I_new_Mu*/ = (I_new - /*pca.*/mean);
  double Fi_norm = norm(Fi, NORM_L2); // NORM_L2 == sqrt(sum(X^2))
  double Fi_new_norm = norm(Fi_new, NORM_L2);
  cv::Mat diff_mu = Fi - Fi_new; // Min Square Diff or Euclidean
  cv::Mat sim_mu = (Fi.t() / Fi_norm) * (Fi_new / Fi_new_norm); //      // similarity measurement (Cross Correlation)
  // make sure this Mat has only one member or index
  LOG_PRINT(sim_mu, "sim_mu", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  double diff_mu_norm = norm(diff_mu, NORM_L2); // this should be MIN
  double sim_mu_value = sim_mu.at<double>(0);
  //  vec_diff_norms/*_mu_norm*/.push_back(diff_mu_norm);
  //  vec_sim_values/*mu_value*/.push_back(sim_mu_value);

  //  // after projecting the eigenspace or feature transformation or transforming their features to the Principle ones
  //  cv::Mat Fi_V = pca.V.t() * Fi/*I_new_Mu*/;
  //  cv::Mat Fi_new_V = pca.V.t() * Fi_new/*I_new_Mu*/;
  //  double Fi_V_norm = norm(Fi_V, NORM_L2); // NORM_L2 == sqrt(sum(X^2))
  //  double Fi_new_V_norm = norm(Fi_new_V, NORM_L2);
  //  cv::Mat diff_mu_V = Fi_V - Fi_new_V; // Min Square Diff or Euclidean
  //  cv::Mat sim_mu_V = (Fi_V.t() / Fi_V_norm) * (Fi_new_V / Fi_new_V_norm); //      // similarity measurement (Cross Correlation)
  //  // make sure this Mat has only one member or index
  //  LOG_PRINT(sim_mu_V, "sim_mu_V", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  //  double diff_mu_V_norm = norm(diff_mu_V, NORM_L2); // this should be MIN
  //  double sim_mu_V_value = sim_mu_V.at<double>(0);
  //  //  vec_diff_norms/*_mu_V_norm*/.push_back(diff_mu_V_norm);
  //  //  vec_sim_values/*_mu_V_value*/.push_back(sim_mu_V_value);
  //
  //  // applying the scales or singular values
  //  cv::Mat Fi_V_S = /*pca.*/S_diag_inv * Fi_V;
  //  cv::Mat Fi_new_V_S = /*pca.*/S_diag_inv * Fi_new_V;
  //  double Fi_V_S_norm = norm(Fi_V_S, NORM_L2); // NORM_L2 == sqrt(sum(X^2))
  //  double Fi_new_V_S_norm = norm(Fi_new_V_S, NORM_L2);
  //  cv::Mat diff_mu_V_S = Fi_V_S - Fi_new_V_S; // Min Square Diff or Euclidean
  //  cv::Mat sim_mu_V_S = (Fi_V_S.t() / Fi_V_S_norm) * (Fi_new_V_S / Fi_new_V_S_norm); //      // similarity measurement (Cross Correlation)
  //  // make sure this Mat has only one member or index
  //  LOG_PRINT(sim_mu_V_S, "sim_mu_V_S", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  //  double diff_mu_V_S_norm = norm(diff_mu_V_S, NORM_L2); // this should be MIN
  //  double sim_mu_V_S_value = sim_mu_V_S.at<double>(0);
  //  //  vec_diff_norms/*_mu_V_S_norm*/.push_back(diff_mu_V_S_norm);
  //  //  vec_sim_values/*_mu_V_S_value*/.push_back(sim_mu_V_S_value);


  // OUTPUT --> comparison results of diff and sim
  vec_diff_norms.resize(2); // 0,1,2,3
  vec_sim_values.resize(2); // 0,1,2,3

  vec_diff_norms[0] = diff_norm;
  vec_diff_norms[1] = diff_mu_norm;
  //  vec_diff_norms[2] = diff_mu_V_norm;
  //  vec_diff_norms[3] = diff_mu_V_S_norm;

  vec_sim_values[0] = sim_value;
  vec_sim_values[1] = sim_mu_value;
  //  vec_sim_values[2] = sim_mu_V_value;
  //  vec_sim_values[3] = sim_mu_V_S_value;

}// compare(I, I_new, pca)
