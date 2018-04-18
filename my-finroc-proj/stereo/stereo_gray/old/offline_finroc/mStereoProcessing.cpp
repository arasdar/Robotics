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
/*!\file    projects/icarus/sensor_processing/libstereo_test/finroc/mStereoProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-06
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc/mStereoProcessing.h"

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
namespace offline
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStereoProcessing> cCREATE_ACTION_FOR_M_STEREOPROCESSING("StereoProcessing");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mStereoProcessing constructor
//----------------------------------------------------------------------
mStereoProcessing::mStereoProcessing(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  ,
  image_viewer(new visualization::ImageViewer("Image Viewer")),
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  road_comparator(new GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  trigger = true;
  continuous = false;

  this->left_images = left_images;
  this->right_images = right_images;
  images_idx = 0;
  this->img_pairs_num = img_pairs_num;
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->registerKeyboardCallback(&mStereoProcessing::keyboardCallback, *this, 0);

//    viewer_disparity->setBackgroundColor(0, 0, 0);
//    viewer_disparity->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
//    viewer_disparity->registerKeyboardCallback(&mStereoGrayOffline::keyboardCallback, *this, 0);


  /*! initialization for AdaptiveCostSOStereoMatching*/
  stereo.setMaxDisparity(60); //original
  stereo.setXOffset(0); //original
  stereo.setRadius(5); //original
  smooth_weak = 20;
  smooth_strong = 100;
  stereo.setSmoothWeak(smooth_weak); //20 original
  stereo.setSmoothStrong(smooth_strong); //original
  stereo.setGammaC(25); //original
  stereo.setGammaS(10); //10 original was original
  stereo.setPreProcessing(true);

  /*Set up normal extraction*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

  bool use_depth_dependent_smoothing = false;
  ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

  /*// Set up segmentation using the ground plane comparator -- If the camera was pointing straight out, the normal would be:*/
  Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)

  Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
  road_comparator->setExpectedGroundNormal(tilt_road_normal);

  float   ground_angular_threshold = pcl::deg2rad(20.0f);
  road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original

  float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
  road_comparator->setAngularThreshold(angular_threshold); //3.0f original

  float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
  bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
  road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);

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
  */


  /*  "aras_icarus_sensorProcessing_libstereo_test
   * ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/left/
   * ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/right/
   * ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/intrinsics.yml
   * ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/extrinsics.yml\n "*/

  string argv_1 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/left";
  string argv_2 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/right";
  string argv_3 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/intrinsics.yml";
  string argv_4 = "/home/aras/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/extrinsics.yml";

  /*Get list of stereo files from left folder*/
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv_1); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  /*reading right images from folder*/
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
}

//----------------------------------------------------------------------
// mStereoProcessing destructor
//----------------------------------------------------------------------
mStereoProcessing::~mStereoProcessing()
{}

void
mStereoProcessing::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
    case ' ':
      trigger = true;
      break;
    case 'c':
      continuous = !continuous;
      break;
    case '1':
      smooth_strong -= 10;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '2':
      smooth_strong += 10;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '3':
      smooth_weak -= 10;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    case '4':
      smooth_weak += 10;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    }
  }
}

////----------------------------------------------------------------------
//// mStereoProcessing OnStaticParameterChange
////----------------------------------------------------------------------
//void mStereoProcessing::OnStaticParameterChange()
//{
//  if (this->static_parameter_1.HasChanged())
//  {
//    /*As this static parameter has changed, do something with its value!*/
//  }
//}
//
////----------------------------------------------------------------------
//// mStereoProcessing OnParameterChange
////----------------------------------------------------------------------
//void mStereoProcessing::OnParameterChange()
//{
//  /*If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.*/
//}

//----------------------------------------------------------------------
// mStereoProcessing Update
//----------------------------------------------------------------------
void mStereoProcessing::Update()
{
//  if (this->InputChanged())
//  {
//    /*At least one of your input ports has changed. Do something useful with its data.
//    However, using the .HasChanged() method on each port you can check in more detail.*/
//  }

  /*Do something each cycle independent from changing ports.*/
  run();
  gridmap();
  secletmap();

  /*this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.*/
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}