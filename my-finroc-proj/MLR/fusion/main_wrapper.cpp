#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace std;
using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

int main(int argc, const char** argv)
{
  // memory, learn and recognition
  tMLR mlr;

  //  if (user_input == "fusion") --> distance data
  {
    // reading all the written sensor data in the memory
    std::vector<std::vector<cv::Mat>> vec_I = mlr.read_data(argv);

    // image data learning and recognition separately
    cv::Mat I_all_img = mlr.vectorize_input_data(vec_I[0]/*_img*/);
    const cv::Mat I_img_sample = vec_I[0][0]; // this is used only to show images for the ROW number
    mlr.learn(I_all_img, I_img_sample, eDATA_TYPE::eIMAGE_DATA, argv);
    //    mlr.recognize_test(I_all_img/*_norm*/, I_all_img, I_img_sample, argv, eDATA_TYPE::eIMAGE_DATA);
    //    I_all_img.release(); // release some RAM and CACHE


    // distance data learning and recognition
    cv::Mat I_all_dist = mlr.vectorize_input_data(vec_I[1]/*_dist*/);
    mlr.learn(I_all_dist, eDATA_TYPE::eDISTANCE_DATA, argv);
    //    mlr.recognize_test(I_all_dist/*_norm*/, I_all_img, I_img_sample, argv, eDATA_TYPE::eDISTANCE_DATA);
    //    I_all_dist.release(); // release some RAM and CACHE


    // localization data or pose data learning and recognition
    cv::Mat I_all_loc = mlr.vectorize_input_data(vec_I[2]/*_loc*/);
    //I_all_loc *= 1000; // mili meter precision
    //mlr.learn(I_all_loc, eDATA_TYPE::eLOCALIZATION_DATA, argv); // NOT working with normalizing and normalization also not make any sense and no theoretical justification
    vec_I.clear();
    cv::Mat mean_loc;
    mlr.compute_mean(I_all_loc, mean_loc);
    mlr.write_mean(mean_loc, argv, "mean_loc");
    //mlr.recognize_test(I_all_loc, I_all_img, I_img_sample, argv, eDATA_TYPE::eLOCALIZATION_DATA);
    I_all_loc.release();

    // fusing all the sensor data and learning them and recognition
    cv::Mat I_all;
    mlr.fuse_data(argv, I_all);
//    cv::Mat I_all_norm;
//    mlr.toNormalize(I_all, I_all_norm, eDATA_TYPE::eALL_DATA);
    mlr.learn(I_all/*_norm*/, eDATA_TYPE::eALL_DATA, argv);
    mlr.recognize_test(I_all/*_norm*/, I_all_img, I_img_sample, argv, eDATA_TYPE::eALL_DATA);
  }

  return 0; //exit
}// end of main function
