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
/*!\file    projects/icarus/sensor_processing/stereo_sugv/capture/tStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-12
 *
 * \brief   Contains tStereoProcessing
 *
 * \b tStereoProcessing
 *
 * This is the class for stereo processing online or stereo capture.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__stereo_sugv__capture__tStereoProcessing_h__
#define __projects__icarus__sensor_processing__stereo_sugv__capture__tStereoProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/io/io.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <FlyCapture2.h>

#include "projects/icarus/sensor_processing/libstereo_test/tTestACSO.h"

/*//terrain features*/
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/centroid.h> ////computeMeanAndCovarianceMatrix

/*// traversability using segmentation*/
#include "projects/icarus/sensor_processing/libsegmentation/tGroundPlaneComparator.h"
#include "projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
namespace stereo_sugv
{
namespace capture
{

using namespace std;
using namespace pcl;
using namespace cv;
using namespace FlyCapture2;
using namespace libsegmentation;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the class for stereo processing online or stereo capture.
 *
 * StereoVisioProcessing is an application for processing stereo images to classify the terrain based on
 * traversability estimation using stereo camera.
 *
 */
class tStereoProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tStereoProcessing();

  ~tStereoProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  IplImage* ConvertImageToOpenCV(Image* pImage);

  void stereoCapture();

  void stereoRect();

  void processCloud(const CloudConstPtr& cloud);
  void run();
//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  int images_idx = 0;

  //stereo capture
  BusManager bus_manager;
  PGRGuid pointgrey_guid;

  //Camera camera;
  GigECamera camera;
  Image* flycap_img;

  //Camera camera_2;
  GigECamera camera_2;
  Image* flycap_img_2;

  // changing the resolution of images
  GigEImageSettings imageSettings;

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/intrinsics.yml", //aras@pasithee:~/stereo_images/stereo_sugv/2_on-sugv/calib_8_good
         input_extrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/extrinsics.yml";
  Mat input_images_left, input_images_right;
  Mat left_img_rect;
  Mat right_img_rect;

  finroc::icarus::sensor_processing::libstereo_test::tTestACSO stereo;
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
  Eigen::Vector3f tilt_road_normal;
  Eigen::Vector3f nominal_road_normal;

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
}


#endif
