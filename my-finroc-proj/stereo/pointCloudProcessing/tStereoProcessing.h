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
/*!\file    projects/icarus/sensor_processing/stereo_gray/offline/tStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-08-03
 *
 * \brief   Contains tStereoProcessing
 *
 * \b tStereoProcessing
 *
 * This is the stereo processing module to generate point cloud using stereo images.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__pointCloudProcessing__tStereoProcessing_h__
#define __projects__stereo_traversability_experiments__aras__pointCloudProcessing__tStereoProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>

#include "projects/stereo_traversability_experiments/aras/libfeature/tIntegralImageNormalEstimation.h"

#include "projects/stereo_traversability_experiments/aras/libsegmentation/tGroundPlaneComparator.h"
#include "projects/stereo_traversability_experiments/aras/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
namespace pointCloudProcessing
{

using namespace pcl;
using namespace std;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGBA PointT;
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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the stereo processing module to generate point cloud using stereo image.
 */
class tStereoProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tStereoProcessing(const std::vector<std::string> left_images, const int img_pairs_num);

  ~tStereoProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*! Here is the right place for your public methods. Replace this line by your declarations!*/
  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  pointPickingcallback(const pcl::visualization::PointPickingEvent& event, void* args);

  void processCloud_normals();
  void processCloud_segm();

  void processCloud_trav();
  void processCloud_trav_segm2plane();
  void processCloud_trav_slopeAnalysis();
  void processCloud_trav_dominGroundPlane();
  void processCloud_trav_stepAnalysis();

  void run_initialize();
  void run();
  void run_proceed_callbacks();
  void run_visualize();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity_processed;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_proc_segm;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_proc_trav;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_stepAnalysis;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_slopeAnalysis;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_processed;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_segm;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_trav;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_stepAnalysis;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_slopeAnalysis;

  std::vector<std::string> left_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool bwd; //trigger backward
  bool continuous;
  bool display_normals;

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;

  PointCloud<RGB> prev_img_disp;
  CloudPtr prev_cloud_disp;

  libfeature::tIntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  PointCloud<Normal>::ConstPtr prev_normal_cloud;
  CloudPtr prev_cloud;

  libsegmentation::GroundPlaneComparator<PointT, Normal>::Ptr  comp;
  libsegmentation::OrganizedConnectedComponentSegmentation<PointT, Label> segm;
  CloudPtr prev_cloud_segm;
  PointCloud<pcl::Label> prev_cloud_segment_labels;
  vector<pcl::PointIndices> prev_cloud_segments;
  unsigned int thresh_segments = 1000;

  unsigned prev_idx = 0;
  vector<Eigen::Vector4f> prev_cloud_planes;
  vector<PointXYZ> prev_cloud_planes_cent_pt;

  CloudPtr prev_cloud_trav_slopeAnalysis;
  vector<unsigned> prev_segm_sizes_trav;
  vector<unsigned> prev_segm_sizes_semi;
  vector<unsigned> prev_segm_sizes_non;
  vector<unsigned> prev_segm_sizes;

  CloudPtr prev_cloud_trav_dominGroundPlane;
  Eigen::Vector4f prev_dominant_ground_plane, prev_dominant_obstacle_plane;
  bool prev_frame_isBad = false;
  PointXYZ prev_dominant_ground_plane_cent;
  CloudPtr prev_cloud_trav_stepAnalysis;

//
// UPD
//

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif