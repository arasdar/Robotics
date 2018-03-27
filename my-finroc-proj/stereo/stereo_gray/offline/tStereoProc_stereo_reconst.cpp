/*
 * tStereoProc_stereo_reconst.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */



#include "projects/stereo_traversability_experiments/aras/stereo_gray/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::stereo_gray::offline;

void tStereoProcessing::stereo_reconst()
{


  /*! initializing stereo module - AdaptiveCostSOStereoMatching */
  stereo.setXOffset(0); //dmin=50, zmax=5 scale=250
  stereo.setMaxDisparity(60); //dmax=250 //dnum=200 zmin=1 scale=250
  stereo.setRadius(5); //original
  smooth_weak = 20;
  smooth_strong = 100;
  stereo.setSmoothWeak(smooth_weak); //20 original
  stereo.setSmoothStrong(smooth_strong); //original
  stereo.setGammaC(25); //original
  stereo.setGammaS(10); //10 original was original
  stereo.setPreProcessing(true);

  stereo_rectify(left_images[images_idx], right_images[images_idx]);

  /*process the disparity map and point cloud*/
  stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
  stereo.medianFilter(4);  // better but slower //optimal

  /*stereo images camera calibration parameters for ptgrey grayscale camera*/
  /*
        float u_c = P2_calib.at<double>(0, 2); //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
        float v_c = P2_calib.at<double>(1, 2); //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
        float focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
        float baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator
  */

  float u_c = P2_calib.at<double>(0, 2) - pt_top_left_crop.x; //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  float v_c = P2_calib.at<double>(1, 2) - pt_top_left_crop.y; //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
  float focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  float baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator

  //FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "X_c: ", u_c, " Y_c:", v_c, " focal: ", focal, " baseline: ", baseline, " point_crop: ", pt_top_left_crop);

  CloudPtr cloud(new Cloud);
  stereo.getPointCloud(u_c, v_c, focal, baseline, cloud);
  prev_cloud = cloud;

  /*! Disparity map visualizing in pcl*/
  pcl::PointCloud<pcl::RGB>::Ptr img_disp(new pcl::PointCloud<pcl::RGB>);
  stereo.getVisualMap(img_disp);
  prev_img_disp = img_disp;

  CloudPtr cloud_disp(new Cloud);
  stereo.getPointCloud(u_c, v_c, focal, baseline, cloud_disp, img_disp);
  prev_cloud_disp = cloud_disp;

}
