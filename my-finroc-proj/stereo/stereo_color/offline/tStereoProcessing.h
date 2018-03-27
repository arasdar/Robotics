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
#ifndef __projects__stereo_traversability_experiments__aras__stereo_color__offline__tStereoProcessing_h__
#define __projects__stereo_traversability_experiments__aras__stereo_color__offline__tStereoProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "projects/stereo_traversability_experiments/aras/libstereo/tTestACSO.h"
#include "projects/stereo_traversability_experiments/aras/libstereo/tBlockBasedStereoMatching.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //pcdwriter

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
namespace stereo_color
{
namespace offline
{

using namespace pcl;
using namespace std;
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

  tStereoProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                    const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename);

  ~tStereoProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*! Here is the right place for your public methods. Replace this line by your declarations!*/
  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  pointPickingcallback(const pcl::visualization::PointPickingEvent& event, void* args);

  void stereo_rectify();
  void stereo_rectify_test();
  void stereo_cropping(Mat* img_rect, const Rect* validRoi);
  void stereo_reconst();
  void stereo_getPointCloudParams();
  void stereo_getPointCloudParams_test();
  void stereo_computePointCloud();

  void processCloud_normals(const CloudConstPtr& cloud);
  void processCloud_segm();

  void processCloud_trav();
  void processCloud_trav_segm2plane();
  void processCloud_trav_slopeAnalysis();
  void processCloud_trav_dominGroundPlane();
  void processCloud_trav_stepAnalysis();

  void run_initialize();
  void run();
  void run_proceed_callbacks();
  void run_proceed_callbacks_test();
  void run_visualize();
  void run_saveCloud(const CloudConstPtr& cloud);

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
  visualization::PCLVisualizer::Ptr viewer_proc_trav_slopeAnalysis;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_stepAnalysis;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_final;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_processed;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_segm;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_trav;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_slopeAnalysis;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_stepAnalysis;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_final;

  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool bwd; //trigger backward
  bool continuous;
  bool display_normals;
//  bool display_all;

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;

  /*! Stereo capture, rectification initialization*/
  string input_intrinsic_filename;
  string input_extrinsic_filename;
  string input_images_left;
  string input_images_right;
  Mat left_img_rect;
  Mat right_img_rect;
  Mat P2_calib;
  Point pt_top_left_crop;

  libstereo::tTestACSO stereo;
  int smooth_weak;
  int smooth_strong;

//  libstereo::BlockBasedStereoMatching stereo;

  PointCloud<RGB>::Ptr prev_img_disp;
  CloudPtr prev_cloud_disp;
  float u_c, v_c, focal, baseline;

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
//  vector<Eigen::Vector4f> prev_cloud_planes;
  vector<Eigen::VectorXf> prev_cloud_planes;
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
  CloudPtr prev_cloud_trav_final;

  /*save cloud parameters*/
  PCDWriter writer_;
  std::string file_name_ = "left";
  std::string dir_name_ = "/home/aras/Desktop/frames_pcd_acso";
//  unsigned format_ = 1; //binary
  unsigned format_ = 2; //compressedBinary
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
