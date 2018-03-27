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
/*!\file    projects/icarus/sensor_processing/libstereo_test/tTestStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 * \brief   Contains tTestStereoProcessing
 *
 * \b tTestStereoProcessing
 *
 * This is stereoProcessing class for grabbing images, rectifying and disparity map generation.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__libstereo__tTestStereoProcessing_h__
#define __projects__stereo_traversability_experiments__aras__libstereo__tTestStereoProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/io/io.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

/*//terrain features*/
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/centroid.h> ////computeMeanAndCovarianceMatrix

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestACSO.h"

/*// traversability using segmentation*/
#include "projects/stereo_traversability_experiments/aras/libsegmentation/tGroundPlaneComparator.h"
#include "projects/stereo_traversability_experiments/aras/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

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

using namespace std;
using namespace pcl;
using namespace cv;
using namespace libsegmentation;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is stereoProcessing class for grabbing images, rectifying and disparity map generation.
 */
class tTestStereoProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tTestStereoProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                        const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename);

  ~tTestStereoProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  inline void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  stereo_rectify(const string input_images_left, const string input_images_right);

  void processCloud(const CloudConstPtr& cloud);

  void processCloud_segmentation(const CloudConstPtr& cloud);

  void run();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool continuous;

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename;
  string input_extrinsic_filename ;
  Mat left_img_rect;
  Mat right_img_rect;

  tTestACSO stereo;
  int smooth_weak;
  int smooth_strong;

  boost::mutex cloud_mutex;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
