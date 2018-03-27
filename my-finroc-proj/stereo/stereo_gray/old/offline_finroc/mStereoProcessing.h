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
/*!\file    projects/icarus/sensor_processing/libstereo_test/finroc/mStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-06
 *
 * \brief Contains mStereoProcessing
 *
 * \b mStereoProcessing
 *
 * This is the module for stereo processing pipeline.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo_test__finroc__mStereoProcessing_h__
#define __projects__icarus__sensor_processing__libstereo_test__finroc__mStereoProcessing_h__

#include "plugins/structure/tModule.h"

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

#include "rrlib/coviroa/tImage.h"
#include "rrlib/mapping/definitions.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo_test/tTestACSO.h"

/*// traversability using segmentation*/
#include "projects/icarus/sensor_processing/libsegmentation/tGroundPlaneComparator.h"
#include "projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

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
 * This is the module for stereo processing pipeline.
 */
class mStereoProcessing : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:


  tOutput<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap;
  tOutput<rrlib::mapping::tMapGridPolar2D<double>> output_secletmap;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStereoProcessing(core::tFrameworkElement *parent, const std::string &name = "StereoProcessing");

  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  stereo_rectify(const string input_images_left, const string input_images_right);

  void cropping(Mat* img_rect, const Rect* validRoi);

  void processCloud(const CloudConstPtr& cloud);

  void run();

  void gridmap();

  void secletmap();

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
  Mat P2_calib;
  Point pt_top_left_crop;

  libstereo_test::tTestACSO stereo;
  int smooth_weak;
  int smooth_strong;

  boost::mutex cloud_mutex;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  Eigen::VectorXf prev_ground_normal;
  Eigen::VectorXf prev_ground_centroid;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mStereoProcessing();

//  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!
//
//  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update(); //override;

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
