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
/*!\file    projects/icarus/sensor_processing/stereo_color/offline/mStereoTestColorOffline.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-02-25
 *
 * \brief Contains mStereoTestColorOffline
 *
 * \b mStereoTestColorOffline
 *
 * this is only for testing and debugging.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__stereo_color__offline__mStereoTestColorOffline_h__
#define __projects__icarus__sensor_processing__stereo_color__offline__mStereoTestColorOffline_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>    // std::min_element, std::max_element

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/stereo_matching.h" //#include <pcl/stereo/stereo_matching.h>
#include "rrlib/mapping/definitions.h"
#include "projects/icarus/mapping/tSectorMap.h"
#include "rrlib/math/tVector.h"
//#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace stereo_color
{
namespace offline
{

using namespace std;
using namespace pcl;
using namespace cv;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

struct callback_args
{
  /*structure used to pass arguments to the callback function*/
  Cloud::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_2;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_3;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_4;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_5;
};

#define PI 3.14159265

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this is only for testing and debugging.
 */
class mStereoTestColorOffline : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tOutput<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap;


//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStereoTestColorOffline(core::tFrameworkElement *parent, const std::string &name = "StereoTestColorOffline");


  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  pp_callback(const pcl::visualization::PointPickingEvent& event, void* args);

  void
  stereo_rectify(const string input_images_left, const string input_images_right);

  PointCloud<RGB>::Ptr
  img2pcd(const Mat image_rect);

  void
  processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud, pcl::PointCloud<pcl::RGB>::Ptr& texture);

  void
  processCloud(const CloudConstPtr& cloud);

  void run();

  void gridmap();

  void sectormap();


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity_processed;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_right;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_original;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_processed;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_right;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_original;
  boost::mutex cloud_mutex;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
  Eigen::VectorXf prev_ground_normal;  //Vector4f was original but buggy in finroc
  Eigen::VectorXf prev_ground_centroid;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool continuous;
  bool display_normals;
  bool detect_obstacles;
  bool displayed;

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename;
  string input_extrinsic_filename ;
  Mat left_img_rect;
  Mat right_img_rect;

  pcl::AdaptiveCostSOStereoMatching stereo;
  int smooth_weak;
  int smooth_strong;

  /*save cloud parameters*/
  pcl::PCDWriter writer_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;

  const int bound_limit = 100; //100; //500 //80 //40 //20
  const double cell_resolution = 0.5; //0.05; //1; //0.13; //0.05; //meter //0.1 //1
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds bounds_stereo_env;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mStereoTestColorOffline();

  virtual void OnStaticParameterChange();   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update();

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
