#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace std;
using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

int main(int argc, const char** argv)
{
  // memory, learn and recognition
  tMLR mlr;

  // reading all the sensors data separately first and concatenate them all at the end
  std::vector<cv::Mat> vec_I_img_proj_all, vec_I_dist_all, vec_I_pose_all;

  cv::Mat image = cv::Mat(10 /*rows*/, 1/*cols*/, CV_64FC1/*type*/, cv::Scalar(1)/*intial value or content*/);
  std::cout << image << std::endl;
  mlr.LOG_PRINT(image, "image", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat ir_distance = cv::Mat(8/*rows*/, 1/*cols*/, CV_64FC1/*Opencv type, double, one channel mat*//*type*/, cv::Scalar(2)/*initial content or value or initialization*/);
  std::cout << ir_distance << std::endl;
  mlr.LOG_PRINT(ir_distance, "ir_distance", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat pose = cv::Mat(3, 1, CV_64FC1, cv::Scalar(3));
  std::cout << pose << std::endl;
  mlr.LOG_PRINT(pose, "pose", eLOG_PRINT_STATE::eIS_ACTIVE);

  for (unsigned int j /*columns*/ = 0; j < 10; j++)
  {
    cv::Mat I_img_proj(image);
    vec_I_img_proj_all.push_back(I_img_proj);

    cv::Mat I_dist(ir_distance);
    vec_I_dist_all.push_back(I_dist);

    cv::Mat I_pose(pose);
    vec_I_pose_all.push_back(I_pose);
  }// for

  // vectorizing them
  cv::Mat I_img_proj_all = mlr.vectorize_input_data(vec_I_img_proj_all);
  cv::Mat I_dist_all = mlr.vectorize_input_data(vec_I_dist_all);
  cv::Mat I_pose_all = mlr.vectorize_input_data(vec_I_pose_all);

  // concatenating the vectors
  std::vector<cv::Mat> vec_I_all = {I_img_proj_all, I_dist_all, I_pose_all};
  cv::Mat I_all;
  cv::vconcat(vec_I_all, I_all);
  std::cout << I_all << std::endl;
  mlr.LOG_PRINT(I_all, "I_all", eLOG_PRINT_STATE::eIS_ACTIVE);

  return 0; //exit
}// end of main function
