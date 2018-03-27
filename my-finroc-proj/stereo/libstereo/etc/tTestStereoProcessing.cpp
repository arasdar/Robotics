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
/*!\file    projects/icarus/sensor_processing/libstereo_test/tTestStereoProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoProcessing.h"

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
namespace stereo_traversability_experiments
{
namespace aras
{
namespace libstereo
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tTestStereoProcessing constructors
//----------------------------------------------------------------------
tTestStereoProcessing::tTestStereoProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
    const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename) :
  image_viewer(new visualization::ImageViewer("Image Viewer")),
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  road_comparator(new GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{

  trigger = false;
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
  viewer->registerKeyboardCallback(&tTestStereoProcessing::keyboardCallback, *this, 0);


  /* only kept for more info -- the more documented stereo, feature and segmentation old version
   //initializing AdaptiveCostSOStereoMatching
    stereo.setMaxDisparity(60); //original


     * setter for horizontal offset, i.e. number of pixels to shift the disparity range over the target image

    stereo.setXOffset(0); //original
    stereo.setRadius(5); //original

    smooth_weak = 20;
    smooth_strong = 100;
    stereo.setSmoothWeak(smooth_weak); //20 original
    stereo.setSmoothStrong(smooth_strong); //original
    stereo.setGammaC(25); //original
    stereo.setGammaS(10); //10 original was original

    *
      * * preprocessing of the image pair, to improve robustness against photometric distortions
      *   (wrt. to a spatially constant additive photometric factor)

    stereo.setPreProcessing(true);


    Set up the normal estimation based on kinematic capability and the terrain for GroundPlaneComparator

     * COVARIANCE_MATRIX - creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood.
    AVERAGE_3D_GRADIENT - creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients.
    AVERAGE_DEPTH_CHANGE - creates only a single integral image and computes the normals from the average depth changes.

    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);

    The depth change threshold for computing object borders.
    max_depth_change_factor  the depth change threshold for computing object borders based on depth changes
     *
    ne.setMaxDepthChangeFactor(0.03f);

    normal_smoothing_size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
     *
    ne.setNormalSmoothingSize(40.0f); //20.0f

    BORDER_POLICY_IGNORE  BORDER_POLICY_MIRROR
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

      decides whether the smoothing is depth dependent
    bool use_depth_dependent_smoothing = false;
    ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

    // Set up the ground plane comparator -- If the camera was pointing straight out, the normal would be:
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)

    Adjust for camera tilt for traversability analysis
    Set the expected ground plane normal with respect to the stereo camera.
     * Pixels labeled as ground must be within ground_angular_threshold radians of this normal to be labeled as ground.
     *
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);

    slope threshold and Set the tolerance in radians for difference in normal direction between a point and the expected ground normal.
    float   ground_angular_threshold = pcl::deg2rad(20.0f);
    road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original


        Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
    road_comparator->setAngularThreshold(angular_threshold); //3.0f original

    step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);*/

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

}

//----------------------------------------------------------------------
// tTestStereoProcessing destructor
//----------------------------------------------------------------------
tTestStereoProcessing::~tTestStereoProcessing()
{}

inline void
tTestStereoProcessing::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
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

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
