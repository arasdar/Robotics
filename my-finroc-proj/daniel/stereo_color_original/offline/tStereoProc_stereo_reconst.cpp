/*
 * tStereoProc_stereo_reconst.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */



#include "projects/stereo_traversability_experiments/daniel/stereo_color_original/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_original::offline;

void tStereoProcessing::stereo_reconst()
{



  input_images_left = left_images[images_idx];
  input_images_right = right_images[images_idx];

//  stereo_rectify(); //color images with calib
  stereo_rectify_test(); //color images aleady rectified :D with no caib parametrs  //getPOintCloud

  stereo_computePointCloud();
}

void tStereoProcessing::stereo_computePointCloud()
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

  /*process the disparity map and point cloud*/
  stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
  stereo.medianFilter(4);  // better but slower //optimal

//  stereo_getPointCloudParams(); //with calib
  stereo_getPointCloudParams_test(); //already rectified and no calib parameters

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

void tStereoProcessing::stereo_getPointCloudParams()
{
  /*stereo images camera calibration parameters for ptgrey grayscale camera*/
  /*
        float u_c = P2_calib.at<double>(0, 2); //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
        float v_c = P2_calib.at<double>(1, 2); //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
        float focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
        float baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator
  */

  u_c = P2_calib.at<double>(0, 2) - pt_top_left_crop.x; //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  v_c = P2_calib.at<double>(1, 2) - pt_top_left_crop.y; //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
  focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  float baseline_pixel = P2_calib.at<double>(0, 3);
  baseline = abs(baseline_pixel) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator
//  baseline = abs(baseline_pixel) * 0.10 / focal;
  //FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "X_c: ", u_c, " Y_c:", v_c, " focal: ", focal, " baseline: ", baseline, " point_crop: ", pt_top_left_crop);
}

void tStereoProcessing::stereo_getPointCloudParams_test()
{
  /**
    * * after stereo matching a PCL point cloud can be computed, given the stereo intrinsic (focal, principal point
    *   coordinates) and extrinsic (baseline) calibration parameters
    */

  /*stereo images camera calibration parameters for haris camera color*/
  u_c = 644.292; //dx //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  v_c = 491.472; //dy //358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
  focal = 961.074; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  baseline = 0.12; //0.46; //meter for unit //0.359294689f; //real one using calculator

}
