/*
 * tStereoProc_stereo_reconst.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */



#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void mStereoProcessing::stereo_reconst()
{
  stereo_rectify(); //color images aleady rectified :D with no caib parametrs  //getPOintCloud

  stereo_computePointCloud();
}

void mStereoProcessing::stereo_computePointCloud()
{
  /*! initializing stereo module - AdaptiveCostSOStereoMatching */
  stereo.setXOffset(10); //dmin=50, zmax=5 scale=250
  stereo.setMaxDisparity(50); //dmax=250 //dnum=200 zmin=1 scale=250

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

  /*! Disparity map visualizing in pcl*/
  pcl::PointCloud<pcl::RGB>::Ptr img_disp(new pcl::PointCloud<pcl::RGB>);
  stereo.getVisualMap(img_disp);
  prev_img_disp = img_disp;

  /*! displaying disparity map*/
  /*! visualizing in opencv*/
  /*
        Mat vmap_opencv(left_img_rect.rows, left_img_rect.cols, CV_8UC3);
        stereo.getVisualMap(vmap_opencv);
        imshow("vmap_opencv", vmap_opencv);
        waitKey(1);
  */


  //without calib parameters
  stereo_getPointCloudParams(); //already rectified and no calib parameters
  CloudPtr cloud(new Cloud);
//  stereo.getPointCloud(u_c, v_c, focal, baseline, cloud);
  stereo.getPointCloud(u_c, v_c, focal, baseline, cloud, input_images_left);
  prev_cloud = cloud;
}

void mStereoProcessing::stereo_getPointCloudParams()
{
  /**
    * * after stereo matching a PCL point cloud can be computed, given the stereo intrinsic (focal, principal point
    *   coordinates) and extrinsic (baseline) calibration parameters
    */

  /*stereo images camera calibration parameters for haris camera color*/
  u_c = left_img_rect.cols / 2; //pixel //dx //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  v_c = left_img_rect.rows / 2; //pixel //dy //358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
  focal = 277.3f; //pixel //simvis camera //left_img_rect.cols / 2; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  baseline = 0.2f; //meter //0.46; //meter for unit //0.359294689f; //real one using calculator

}
