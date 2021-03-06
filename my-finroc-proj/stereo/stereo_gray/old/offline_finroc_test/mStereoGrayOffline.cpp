//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/sensor_processing/stereo_gray/offline/finroc/mStereoGrayOffline.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-15
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace stereo_gray
{
namespace offline_test
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStereoGrayOffline> cCREATE_ACTION_FOR_M_STEREOGRAYOFFLINE("StereoGrayOffline");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mStereoGrayOffline constructor
//----------------------------------------------------------------------
mStereoGrayOffline::mStereoGrayOffline(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  viewer_disparity(new pcl::visualization::PCLVisualizer("3D Viewer disparity")),
  viewer_disparity_processed(new pcl::visualization::PCLVisualizer("3D Viewer debugging")),
  viewer_disp_proc(new pcl::visualization::PCLVisualizer("3D Viewer disparity processed")),
  viewer_original(new pcl::visualization::PCLVisualizer("3D Viewer test")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
  image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
  image_viewer_disparity_processed(new visualization::ImageViewer("Image Viewer Disparity processed")),
  image_viewer_right(new visualization::ImageViewer("Image Viewer right")),
  image_viewer_original(new visualization::ImageViewer("Image Viewer Original"))
  ,
  road_comparator(new GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
  ,
  nominal_road_normal(0.0, -1.0, 0.0)
{
  trigger = true;
  continuous = false;
  display_normals = false;

  this->left_images = left_images;
  this->right_images = right_images;
  images_idx = 0;
  this->img_pairs_num = img_pairs_num;
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->registerKeyboardCallback(&mStereoGrayOffline::keyboardCallback, *this, 0);
  viewer->registerPointPickingCallback(&mStereoGrayOffline::pointPickingcallback, *this, (void*)&cb_args);

  viewer_disparity->setBackgroundColor(0, 0, 0);
  viewer_disparity->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disparity->registerKeyboardCallback(&mStereoGrayOffline::keyboardCallback, *this, 0);
  viewer_disparity->registerPointPickingCallback(&mStereoGrayOffline::pointPickingcallback, *this, (void*)&cb_args);

  viewer_original->setBackgroundColor(0, 0, 0);
  viewer_original->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_original->registerKeyboardCallback(&mStereoGrayOffline::keyboardCallback, *this, 0);
  viewer_original->registerPointPickingCallback(&mStereoGrayOffline::pointPickingcallback, *this, (void*)&cb_args);

  viewer_disp_proc->setBackgroundColor(0, 0, 0);
  viewer_disp_proc->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disp_proc->registerKeyboardCallback(&mStereoGrayOffline::keyboardCallback, *this, 0);
  viewer_disp_proc->registerPointPickingCallback(&mStereoGrayOffline::pointPickingcallback, *this, (void*)&cb_args);

  viewer_disparity_processed->setBackgroundColor(0, 0, 0);
  viewer_disparity_processed->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disparity_processed->registerKeyboardCallback(&mStereoGrayOffline::keyboardCallback, *this, 0);
  viewer_disparity_processed->registerPointPickingCallback(&mStereoGrayOffline::pointPickingcallback, *this, (void*)&cb_args);


  /*! AdaptiveCostSOStereoMatching*/
  stereo.setXOffset(50); //dmin=50, zmax=5 scale=250
  stereo.setMaxDisparity(200); //dmax=250 //dnum=200 zmin=1 scale=250

  stereo.setRadius(5); //original

  /*
    smooth_weak = 10; //20;
    smooth_strong = 25; //100;
  */
  smooth_weak = 20;
  smooth_strong = 100;

  stereo.setSmoothWeak(smooth_weak); //20 original
  stereo.setSmoothStrong(smooth_strong); //original
  stereo.setGammaC(25); //original
  stereo.setGammaS(10); //10 original was original

  stereo.setPreProcessing(true);

  /*! Set up normal extraction*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

  bool use_depth_dependent_smoothing = false;
  ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

  /*! Set up segmentation using the ground plane comparator */
  const float camera_angle = pcl::deg2rad(11.4818f); //TODO call back from run_all and finding the normal differences
  tilt_road_normal = Eigen::AngleAxisf(camera_angle, Eigen::Vector3f::UnitX()) * nominal_road_normal;
  road_comparator->setExpectedGroundNormal(tilt_road_normal);

  const float ground_angular_threshold = pcl::deg2rad(45.0f);
  road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original
  float angular_threshold = ground_angular_threshold; //pcl::deg2rad(45.0f); //the tolerance in radians
  road_comparator->setAngularThreshold(angular_threshold); //3.0f original

  /*
   * reading the input folders and calibration files
   */
  /*variable initial*/
  int img_number_left = 0, img_number_right = 0 ;

  /*
    string argv_1 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/left_selected";
    string argv_2 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/right_selected";
    string argv_3 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/calib_1-ok/intrinsics.yml";
    string argv_4 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/calib_1-ok/extrinsics.yml";

    string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/left",
           argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/right",
           argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/intrinsics.yml",
           argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/extrinsics.yml";

    string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/left_ramp-down",
           argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/right_ramp-down",
           argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/intrinsics.yml",
           argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/extrinsics.yml";

    string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/left_ramp",
           argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/right_ramp",
           argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/intrinsics.yml",
           argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/extrinsics.yml";
  */

  /*! test with only one image*/
  /*
    string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/test/left",
           argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/test/right",
           argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/calib_5_great/intrinsics.yml",
           argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/calib_5_great/extrinsics.yml";
  */

  /*
    string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/8th/left",
           argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/8th/right",
           argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/calib_5_great/intrinsics.yml",
           argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-05-16_pipeline_capturing-indoor-in-office/calib_5_great/extrinsics.yml";
  */
  /*
    smooth_weak = 10; //20;
    smooth_strong = 25; //100;
  */


  /*
    string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/1st/left",
           argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/1st/right",
           argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/calib_5_great/intrinsics.yml",
           argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/calib_5_great/extrinsics.yml";
  */

  string argv_1 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/2nd/left",
         argv_2 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/2nd/right",
         argv_3 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/calib_5_great/intrinsics.yml",
         argv_4 = "/home/aras/stereo_images/stereo_sugv/2014-06-03_buro-test-new-pose/calib_5_great/extrinsics.yml";

  /*Get list of stereo files from left folder*/
  //std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv_1); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  /*reading right images from folder*/
  //std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr(argv_2); itr != end_itr; ++itr)
  {
    right_images.push_back(itr->path().string());
    img_number_right++;
  }
  sort(right_images.begin(), right_images.end());
  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  /*showing the input images*/
  cout << "img_number_left: " << img_number_left << std::endl;
  cout << "img_number_right: " << img_number_right << std::endl;
  if (img_number_left == img_number_right)
    img_pairs_num = img_number_left;

  /*calibration parameters*/
  input_intrinsic_filename = argv_3;
  input_extrinsic_filename = argv_4;

}// constructor

//----------------------------------------------------------------------
// mStereoGrayOffline destructor
//----------------------------------------------------------------------
mStereoGrayOffline::~mStereoGrayOffline()
{}

////----------------------------------------------------------------------
//// mStereoGrayOffline OnStaticParameterChange
////----------------------------------------------------------------------
//void mStereoGrayOffline::OnStaticParameterChange()
//{
////  if (this->static_parameter_1.HasChanged())
////  {
////    //As this static parameter has changed, do something with its value!
////  }
//}
//
////----------------------------------------------------------------------
//// mStereoGrayOffline OnParameterChange
////----------------------------------------------------------------------
//void mStereoGrayOffline::OnParameterChange()
//{
//  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
//}

//----------------------------------------------------------------------
// mStereoGrayOffline Update
//----------------------------------------------------------------------
void mStereoGrayOffline::Update()
{
  if (this->InputChanged())
  {
    //At least one of your input ports has changed. Do something useful with its data.
    //However, using the .HasChanged() method on each port you can check in more detail.
  }

  //Do something each cycle independent from changing ports.
  run();

  //this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}
